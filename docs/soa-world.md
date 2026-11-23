# Headless SoA world

A headless 3D Box3D world with a structure-of-arrays state view, for Lodur and Bad Day Simulator.
Box3D decides every physical outcome; Ymir creates bodies, steps the world and exposes state.

## Why it is not the retained session

`ymir_box3d_session_*` (`ymir_box3d.c`) is the planar Aetheria session: gravity is zero, Y is locked,
only yaw rotates (`motionLocks.linearY/angularX/angularZ`), and state leaves by `copy_bodies` into an
array of structs. A body cannot fall or tumble there, and every read is a copy. The SoA world is a
separate native object (`ymir_box3d_soa.c`, ABI 9) so the Aetheria session's contract is untouched.

## Contract

- Native (ABI 9): `ymir_box3d_soa_create(capacity, joint_capacity, gravity, fact_capacity, hit_threshold, particle_capacity, chain_capacity, attachment_capacity) /
  add_body / add_joint / step / get_view / get_facts / set_anchor_target / destroy_joint / free_joint_slots / apply_impulse / cast_ray_closest / destroy`. `add_body` takes one descriptor (sphere, capsule, box, static box, anchor; position,
  rotation, velocity, size, density, friction, restitution, rolling resistance, material id, parent, limb). Capacities are fixed at creation,
  so lane pointers never move.
- Body lanes (`ymir_box3d_soa_view`): `stable_id`, `kind`, `parent` and `limb` (set at creation, not read by
  Box3D: the limb tree for the consumer), `size_0`, `size_1`, `size_2` (read-only shape size set at creation,
  never changed by `step`: sphere {radius}, capsule {radius, half_length} along local Y, box and static box
  {half_x, half_y, half_z}; unused components 0; shapes are centred on the body origin, so a renderer draws
  every body from kind and size), position xyz, rotation quaternion xyzw, linear and angular
  velocity xyz. One contiguous array per component, indexed by body.
- Write lanes: per-body world `force` (at the centre of mass) and `torque`, xyz, mutable through the view.
  `step` applies them to the dynamic bodies and clears them, so a lane holds one step's intent. A
  non-finite value refuses the whole step before any body is touched.
- Joints: `add_joint` takes two body indices (B is the child), a world anchor and axis, and a kind
  (revolute, spherical, weld). The joint frame has its z axis on the axis (the hinge axis); the creation
  pose is the joint's zero. Optional limits: revolute angle, spherical swing cone and twist. Joint lanes:
  `joint_kind`, `joint_body_a/b`, `joint_angle` (Box3D's revolute angle or spherical twist), `joint_frame`
  (world orientation of frame A, quaternion), `joint_origin` (world position of frame A's origin, the anchor), `joint_rotation` (frame B relative to frame A, quaternion,
  w >= 0) and `joint_angular_velocity` (B minus A, in frame A). A consumer can compute a joint torque from
  these and write it as equal and opposite torques on the two body lanes.
- Contact lanes (read-only, rewritten by every `step`): `contact_impulse` (N s: the normal impulse the
  body's touching contacts delivered to it over the whole step, the sum of Box3D's `stepNormalImpulse` over
  its manifold points; 0 for a body touching nothing; exact, so a jointed limb's reading is what the contacts
  really delivered, not a mass formula's guess. It is a sum of magnitudes over all the body's contacts, not the
  net: a ball under a ball reads support plus load, 3 m g dt, not m g dt. At rest it equals the weight only on
  average: heavy balls in a pile alternate between about 1.5 and 0.5 m g dt from step to step, so a consumer
  must not threshold a single step; average over steps or use `contact_impulse / dt` as a load estimate over a
  window), `contact_count`
  (touching contacts) and `material_id` (the body's `material_id`, also set as the shape's Box3D user
  material). `stepNormalImpulse` is the GameCult Box3D fork's observed field (see `box3d-parity.md`); it is
  the normal channel only, since Box3D exposes no tangent impulse. Sleep is off in SoA worlds, so manifolds
  hold no stale values; if sleep is ever enabled, zero the lanes of sleeping bodies. The hit fact's `impulse`
  (N s) is the same sum over the manifold of the hit's two shapes, read after the step.
- Contact facts: every shape is created with contact and hit events on (Box3D's default is off). After each
  `step`, `ymir_box3d_soa_get_facts` returns pointers and counts into buffers fixed at create (`fact_capacity`
  per kind, no allocation in the step): Begin and End `{lane_a, lane_b, stable_id_a, stable_id_b}` and Hit
  those plus `point` xyz, `normal` xyz (A to B), `approach_speed` (a touch faster than `hit_threshold` m/s)
  and `material_a`, `material_b`. Facts are in Box3D's event order and last one step: the next `step`
  rewrites them. `*_dropped` counts the facts a step could not keep past `fact_capacity`. The lane in a fact
  is the body's user data (lane + 1) and the stable id is the shape's user data. The facts report; they
  decide nothing.
- Grab and poke input (immediate commands; Ymir decides nothing about grabbing, so a player grab, a scripted
  hook and a cursor tool are all the same three calls):
  - Anchor (body kind 4): a kinematic body with no shape. It never collides or is hit, its size lanes are 0 and
    size, density, friction and material are ignored. `set_anchor_target(lane, position, rotation, time_step)`
    gives it the velocity that reaches the pose after `time_step`, so passing the step's dt lands it on the pose
    at the end of the next step, where `step` stops it: after every step an anchor's velocity is zeroed, so with no
    new target it holds still at the last pose (a skipped frame flings nothing) and its velocity lanes read 0. It is
    refused for a lane that is not an anchor, for non-finite or non-unit input and for a time step <= 0. The force and torque lanes never move an anchor. Its pose reads from the state lanes.
  - Motor joint (kind 3): `add_joint` with the descriptor's `linear_hertz`, `linear_damping`, `max_force`,
    `angular_hertz`, `angular_damping`, `max_torque` (finite, >= 0, ignored by the other kinds; zero hertz turns that
    spring off; no velocity target). The frame is built from the world anchor and axis like every joint, so the
    linear spring pulls body B's anchor point onto A's (A an anchor, B the body to grab). `max_force` and
    `max_torque` bound the pull: with a max force below the weight the body hangs below the target. `joint_angle`
    reads 0.
  - Release: the cursor's anchor is long-lived (a client creates one per cursor and keeps it; bodies are never
    removed here). `destroy_joint(joint, generation)` releases a grab by destroying the motor joint only; the anchor and the
    grabbed body stay. Only motor joints can be destroyed (general joint and body removal belongs to
    `ymir-break-and-remove`); anything else is refused. The joint's lane becomes a tombstone: `joint_kind` 4
    (Removed), `joint_body_a/b` -1 and every joint lane 0. `joint_count` counts slots ever used, tombstones included.
    `add_joint` reuses the lowest tombstone only after a step has completed since the destroy, so lanes read within a
    step stay valid; until then the slot is not free. `free_joint_slots` (managed `FreeJointSlots`) is what
    `add_joint` could create right now: check it, and `FreeBodySlots`, before adding an anchor for a grab, so a full
    world leaves no orphan anchor. Repeated grab and release on one slot do not grow the world.
    A joint's handle is its lane and the slot's generation (`joint_generation` lane, counts destroys of the slot;
    managed `Box3DSoaJointHandle`, returned by `AddJoint` and `AddMotorJoint`, converts to the lane). `destroy_joint`
    refuses a generation that is not the slot's, so a second release click with an old handle never destroys a later
    grab that reused the slot. Anchor creation must be followed by a joint that validates (its spring fields are
    checked there), or the anchor is left behind: with one long-lived anchor per cursor that is accepted, and an atomic
    `AddGrab` is a follow-up only if cursors become dynamic.
  - `apply_impulse(lane, impulse, point)`: a one-shot world impulse (N s) at a world point; off the centre of mass it
    also spins the body. Dynamic bodies only (a static body or anchor is refused), non-finite input refused.
  - `cast_ray_closest(origin, translation)` fills a `ymir_box3d_soa_ray_hit` {hit, lane, stable_id, point, normal,
    fraction}: Box3D's closest hit along origin + t * translation, t in [0, 1], default filter; a miss is success with
    `hit` 0 and every field 0; a non-finite or zero translation is refused. Anchors are never hit. A ray that starts
    inside a shape skips that shape (Box3D's behaviour, it ignores initial overlap) and reports what lies beyond.
  - Managed: `AddAnchor`, `SetAnchorTarget`, `AddMotorJoint`, `DestroyJoint`, `FreeJointSlots`, `FreeBodySlots`, `ApplyImpulse`, `CastRayClosest` (returns
    `Box3DSoaRayHit`) on `Box3DSoaWorld`; refusals throw `InvalidOperationException`.
- State lanes change only inside `step`. Between steps a caller may read any state lane in place.
- Managed (`src/Ymir.Box3D/Box3DSoaWorld.cs`, public: `Box3DSoaWorld`, `Box3DSoaView`, `Box3DSoaFacts`, `Box3DSoaKind`, `Box3DSoaJointKind`; the native structs and the view constructor stay internal): `Box3DSoaView` exposes each lane as a span over
  native memory (`ReadOnlySpan` for state, `Span` for write lanes). Spans are invalid after `Dispose`.
  `Count` and `JointCount` are as of when the view was taken, so take it after the last add.
  `Box3DSoaWorld.Facts` is a `ref struct` of `ReadOnlySpan`s (`Begin`, `End`, `Hit`) with the same dispose
  guard; the constructor takes `factCapacity` (default 4 x capacity, 0 refused) and `hitThreshold` (default 1);
  the `Add*` methods take an optional `materialId`.
- Determinism: one Box3D worker, bodies stepped in insertion order, same inputs give identical lane
  bytes on the same build. It is not claimed across platforms or CPUs.
- Particles (`ymir_particles.c`, C17, calls no Box3D; `ymir_box3d_soa.c` stays the only Box3D caller and the only caller
  of `ymir_particles_begin`, `_substep` and `_end`). A world created with `particle_capacity`, `chain_capacity`, `attachment_capacity` > 0 owns fixed
  lanes that never move; with all three 0 it behaves byte-identically to ABI 8. Nothing writes a particle's position or
  velocity: particles move only by gravity, contact and attachment.
  - Chains: `create_chain(desc{stable_id, count >= 2, positions, masses (0 = immovable), radius, stretch and bend compliance,
    friction, material_id})` returns lane and generation; link and bend rest lengths are the creation distances.
    `split_chain(chain, gen, link, tail_stable_id)` cuts the link after particle `link`: the head keeps lane and id, the tail
    takes a new lane; no particle changes lane or address. `destroy_chain(chain, gen)` also detaches every attachment that
    reaches it. Ymir never splits, destroys or detaches on a threshold: link tension and attachment force are sensors.
  - Attachments: `attach(desc{kind, particle, body, point, compliance})`: particle-body (`body` is the body lane, `point` in its
    frame), particle-static (`point` in the world) or particle-particle (`body` is the other particle's lane); `detach(lane,
    gen)`. A body or static attachment also gives the chain long-range tethers (a unilateral distance bound at the chain's rest
    path length) from its point to every particle of the chain and to the chain's other attachment points; they bear a hung
    load past the links. Attach where the particle already is. A particle does not collide with the one body it is attached
    to (the last body attachment of that particle).
  - Lanes: particle `x y z vx vy vz radius inv_mass link_tension chain` (a free slot reads chain -1, every lane 0), chain
    `stable_id kind generation first size removed_at material_id stretch_compliance bend_compliance friction`, attachment
    `kind generation particle body point compliance force`, body `particle_impulse xyz`. Counts are high-water marks.
    `link_tension` (N, link from this particle to the next, mean over the substeps) and attachment `force` (N exerted on the
    chain) are sensors, tether load included (below). `particle_impulse` (N s, world) is the tick's total of the impulses
    Box3D applied to the body inside its substeps, derived from them and the owner of nothing.
  - Slots: chain and attachment lanes are tombstones (kind Removed, generation + 1) reused only once a step has passed, lowest
    first; a destroyed chain's particle range is held for the same step and reused first fit, lowest index. Stale generations
    and non-finite input are refused before anything changes. `free_particle_slots` is the largest free run.
  - Coupling, in Box3D's substep. A correction applied to a body after `b3World_Step` is late by a substep or gravity-wrong
    by (n + 1) / (2 n), and a tied body then rings or loses momentum, so the particle phase runs inside Box3D's substep:
    `ymir_box3d_soa_create` registers `b3World_SetSubStepCallback(ymir_soa_particle_substep)` (GameCult's fork of Box3D, branch
    `substep-callback`) for a particle world. The callback runs after the biased solve and before positions integrate, exactly
    `sub_step_count` times per `b3World_Step` on the orchestrating thread, also when no body is awake. Each substep:
    `b3Body_GetSubStepState` refreshes each proxy's pose (at the substep's start) and solved velocity (0 and inverse mass 0
    for a body outside the awake set); `ymir_particles_substep` predicts proxies (`x + h v`, rotation by `h w`, as Box3D
    integrates) and particles, builds the grid, then solves particle contacts, attachments, chain stretch and bend,
    tethers, and last the particle-body contacts, each correction moving both sides by inverse mass; each dynamic proxy's
    correction over `h` returns as `b3Body_ApplySubStepImpulse` (a velocity change, no wake, no clamp) and Box3D integrates the
    body to where the constraint put it. Contacts run last: with contacts first a box pushed into a chain reads 14% of a radius
    inside. Step order in `ymir_box3d_soa_step`: write lanes, gather proxies (shape, size, inverse mass, world inverse
    inertia, step-start pose), `ymir_particles_begin`, `b3World_Step` (the substeps run in it), `ymir_particles_end` and the
    `particle_impulse` lane, anchor stop, facts, contact lanes, refresh. Nothing is carried between steps; proxies are rebuilt
    every step. `detach`, `destroy_chain` and `split_chain` take effect in the next step's first substep. Kinematic anchors are
    seen at their substep pose and velocity with inverse mass 0. Bodies are never teleported; no `b3Body_Apply*Impulse` or
    `SetTransform` runs from the particle path.
  - Sensors. Every correction is `lambda / h^2` in newtons, one scale. Attachment `force` is the sum of the corrections at
    the point, tethers included. `link_tension[i]` is read by momentum balance: links, bends and tethers book their position
    impulses on the particles they move (each booking has an equal and opposite one), and the link's load is the sum of the
    bookings on the particles past it, along the link, over `h^2`, mean over the substeps. It reads the weight below each link
    within 1% from 1 to 70 kg, whichever way the load is routed.
- Particle determinism: single thread; chains Gauss-Seidel in link order; the one Jacobi pass is gather-form (each particle
  writes only its own correction, neighbours read in cell then index order from a stable counting-sort grid, a bucket shared
  by two cells is told apart by the cells' coordinates); proxy impulses are summed per body in constraint order, per substep; only `+ - * /`
  and `sqrtf`; the file builds with `-ffp-contract=off` (`/fp:precise` on MSVC). Same bytes on the same build, serial or with other
  worlds stepping on other threads. No threads in this cut.
- Concurrency: see Threading below; world creation and destruction are serialized by Ymir's native lock.

## Tests

`tests/Ymir.Box3D.Soa.Tests`, run serially: gravity and rest read through one held view, address stability
across steps, two runs byte-equal at six checkpoints (25 spheres colliding on a slab), a one-ulp input
difference diverging, argument refusal, dispose; and for joints: hinge angle and the relative-rotation
lanes agreeing, an off-axis hinge, a limit and a weld, a spherical joint leaving the swing plane, force and
torque lanes (exact acceleration, cleared after the step, static bodies ignoring them, NaN refusing the
step), capsule and box rest heights with parent and limb lanes, refusals, and bit-identical repeat runs of
a torque-driven jointed body. `Box3DSoaGrabTests`: a strong grab reaching a target 1 m away within 1 s, a weak grab falling at least `g - F/m`, an anchor that does not collide and is not hit by a ray, force lanes not moving an anchor, an off-centre impulse (linear `J/m` and the spin's sign and size), the ray's nearest body with lane, stable id, point and fraction, refusals changing nothing, and bit-identical repeat runs with a moving anchor and an impulse. `Box3DSoaContactTests`: a drop gives one begin and one hit (speed within 5%
of sqrt(2gh), point and normal), lifting gives one end, dropped counters resetting each step, materials on
hits, facts lasting one step, overflow counted, per-step byte determinism of facts and contact lanes, and
no allocation across 100 steps with fact reads. `Box3DSoaImpulseTests`: `contact_impulse` and the hit `impulse` equal the momentum change of the bodies the contact acted on, plus their weight, within 1e-4 of the peak at every step: a 5 kg ball and a 0.3 g BB at 100 m/s onto a jointed forearm (the BB reads 0.030 N s, the ball over 400 times more), the arm falling forearm-first onto a slab, a sphere dropped 1.4 m at 60 and 240 Hz, 1 and 4 sub-steps, restitution 0 and 0.5, two free bodies at restitution 0 and 0.5; a swung arm into a slab, a big sphere with 60 small ones converging (more contacts than graph colours, so the scalar overflow solver runs: the big lane is the sum of the small lanes, each equal to its own momentum change), and at rest `contact_impulse / dt` is the weight (0.1%, a lone sphere) while a body touching nothing reads 0.

`Box3DSoaParticleTests` (particles): the ABI 8 scene's six checkpoint hashes with no particle capacity (pinned from 5104b17); a 48-particle chain hanging from a static point under 1% stretch for 4 s; a 70 kg sphere held by the same chain (from 1 s to 5 s speed under 0.001 m/s, stretch under 0.1%, the impulse per tick within 0.5% of m g dt); an anchor dragging a 5 kg box along the floor by a chain (the box moves, `particle_impulse` reads it); a sphere, capsule, box and static box each pushed into a lying chain's end (no particle over 10% of its radius inside); overload (300 kg) never splits and `split_chain` is the only cut; split keeps lanes and addresses and the tail falls; destroy then create in one step refused, after a step reuses lane and range, stale generations refused for destroy, split and detach; four chains piled on two bodies byte-equal across two runs at six checkpoints, a one-ulp difference diverging, and eight concurrent worlds giving the serial bytes; no allocation across 100 steps; refusals changing nothing. `Box3DSoaParticleSourceTests` reads the C source: no Box3D call in `ymir_particles.*`, no allocation outside create and free, no threads or atomics, `begin` and `end` called once (in the step), `substep` once (in the callback), the callback the only user of the fork's two accessors, no split, destroy or detach in them, and the retired names absent.

`Box3DSoaCouplingTests` (the coupling): the top link, the link at 10 and at 46, and both attachment forces read the load at 1, 5, 20 and 70 kg (links within 1%, the body's pull within 0.5%); a sphere thrown at 5 m/s with a 48 x 20 g chain from rest keeps x momentum within 0.1% and gains no energy, at 5 and 70 kg; thrown up it reaches the apex momentum allows; 5 kg snapping a taut chain at 6 m/s keeps at most 5% of its energy and 70 kg is still within a tick; a 45 degree pendulum for 12 s gains no energy and loses at most 1.25 times a heavy-particle control, and a hung body keeps its spin about the vertical; a detach or a destroyed chain frees a held 70 kg body in the step after the command, with no upward impulse; `particle_impulse` equals M dv minus M g dt every tick, hung and thrown; chain friction 0.6 holds a chain on a 20 degree slope and friction 0 does not; no particle is farther from a tethered point than its rest path; an attached end stays on its body point; a world whose only body is static still steps the particles (a dropped chain falls g t^2 / 2 within 2%) and rests; and a cost line with 16 piled chains x 48 at 4 substeps, one body and forty (printed, not asserted).

## Not done

Body removal with a stable ordering rule, compound bodies
and per-shape material, and motor joints with a velocity target or per-joint force readback (the motor joint here is a position spring only; Lodur drives limbs from torque lanes).
`Ymir.Core` has no session-level surface over the SoA world yet; consumers use `Ymir.Box3D` directly.
Unity consumption is unrun; cross-platform bit-identity is unclaimed.

Particles: a heavy body cannot be held up
by a chain's contacts against a floor (the chain is squeezed between two solids it cannot satisfy: a position solver with a
50:1 mass ratio; the pushing tests drive the body into a chain end instead). A snap on a taut chain is inelastic by the solver's
construction (a 5 kg body keeps 2% of its energy, no rebound), and a swung weight loses a few percent more as a tied body
than as a heavy particle (65.6 against 59.6 J at 70 kg over 12 s); neither gains energy. Determinism under MSVC's `/fp:precise`
is unbuilt. No fluid, emitters, threads or particle write lanes (follow-up `ymir-particle-fluid`).

## Build and lifetime notes

- Native library name: `ymir_box3d.dll` on Windows (MinGW needs `PREFIX ""`, set under `if(MINGW)`; MSVC emits it
  natively), `libymir_box3d.so` on Linux, `libymir_box3d.dylib` on macOS. `Ymir.Box3D.csproj` copies from
  `bin\$(Configuration)\` on every platform: `CMakeLists.txt` writes `bin/$<CONFIG>/` and the csproj passes `-DCMAKE_BUILD_TYPE=$(Configuration)`, so single- and multi-config generators land in the same place.
- Windows run: MinGW gcc with `CMAKE_GENERATOR="Ninja Multi-Config" CC=gcc`; `Ymir.Box3D.Soa.Tests` passes.
  Linux run (Yggdrasil, default generator, gcc): the same tests pass and emit `bin/Debug/libymir_box3d.so`. macOS is unrun.
- Threading: creating and destroying worlds is safe from any number of threads at once (a process-wide lock in the native
  layer serializes `b3CreateWorld`/`b3DestroyWorld`, which otherwise scan and rewrite Box3D's global world pool unsynchronized;
  the retained session and pair-step paths take the same lock). Stepping distinct worlds concurrently is not known to race: Box3D's own docs promise no thread safety, and this rests on a reading of the pinned source and one light test. A single world
  is still not thread-safe. `Box3DSoaConcurrentCreateTests` creates, steps and disposes worlds from eight threads.
- At most 128 live worlds per process (Box3D's `B3_MAX_WORLDS`); creating more trips a Box3D assert, not a managed error.
- `Box3DSoaWorld` is not thread-safe: `Dispose` must not race `Step` or a lane read. A `Box3DSoaView` refuses
  to hand out a lane (ObjectDisposedException) once its world is disposed, and a `default` view refuses with
  InvalidOperationException. The check guards obtaining a span, not a span already held: a span taken from a
  view must not be used after the world is disposed. Known limit: an undisposed world leaks its
  native memory; there is no finalizer or SafeHandle.
