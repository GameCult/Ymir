# Headless SoA world

A headless 3D Box3D world with a structure-of-arrays state view, for Lodur and Bad Day Simulator.
Box3D decides every physical outcome; Ymir creates bodies, steps the world and exposes state.

## Why it is not the retained session

`ymir_box3d_session_*` (`ymir_box3d.c`) is the planar Aetheria session: gravity is zero, Y is locked,
only yaw rotates (`motionLocks.linearY/angularX/angularZ`), and state leaves by `copy_bodies` into an
array of structs. A body cannot fall or tumble there, and every read is a copy. The SoA world is a
separate native object (`ymir_box3d_soa.c`, ABI 5) so the Aetheria session's contract is untouched.

## Contract

- Native (ABI 5): `ymir_box3d_soa_create(capacity, joint_capacity, gravity, fact_capacity, hit_threshold) /
  add_body / add_joint / step / get_view / get_facts / set_anchor_target / apply_impulse / cast_ray_closest / destroy`. `add_body` takes one descriptor (sphere, capsule, box, static box, anchor; position,
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
  (world orientation of frame A, quaternion), `joint_rotation` (frame B relative to frame A, quaternion,
  w >= 0) and `joint_angular_velocity` (B minus A, in frame A). A consumer can compute a joint torque from
  these and write it as equal and opposite torques on the two body lanes.
- Contact lanes (read-only, rewritten by every `step`): `last_substep_force` (N: the final sub-step's
  `normalImpulse` summed over the body's touching manifold points, times sub-steps over dt; 0 for a body
  touching nothing; the weight at rest; NOT an impact measure, it sees one sub-step and misses an impact
  step by 0.02x to 1.9x), `contact_count` (touching contacts) and `material_id` (the body's
  `material_id`, also set as the shape's Box3D user material). Box3D exposes no whole-step impulse:
  `totalNormalImpulse` adds the accumulated impulse once per solve and once per relax pass (so it reads twice
  the weight at rest, and on impacts is no fixed multiple of the step's impulse). For impact severity see the
  hit fact. `impulse_estimate` (N s) is `approach_speed * (1 + e) /
  (1/mA + 1/mB)`, e the larger shape restitution and only above Box3D's restitution threshold, a static
  body's 1/m taken as 0. It is exact for a centre-to-centre hit between free bodies, can over-count when one
  collision yields several hit facts (box corners, tilted capsules: they sum), and undercounts heavily for
  jointed bodies (it takes the limb's own mass, not the joint-coupled effective mass). It is not a bound.
  A mass-aware per-body impulse that holds for jointed bodies is being designed.
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
    at the end of the next step; it is refused for a lane that is not an anchor, for non-finite or non-unit input
    and for a time step <= 0. The force and torque lanes never move an anchor. Its pose reads from the state lanes.
  - Motor joint (kind 3): `add_joint` with the descriptor's `linear_hertz`, `linear_damping`, `max_force`,
    `angular_hertz`, `angular_damping`, `max_torque` (finite, >= 0, ignored by the other kinds; zero hertz turns that
    spring off; no velocity target). The frame is built from the world anchor and axis like every joint, so the
    linear spring pulls body B's anchor point onto A's (A an anchor, B the body to grab). `max_force` and
    `max_torque` bound the pull: with a max force below the weight the body hangs below the target. `joint_angle`
    reads 0.
  - `apply_impulse(lane, impulse, point)`: a one-shot world impulse (N s) at a world point; off the centre of mass it
    also spins the body. Dynamic bodies only (a static body or anchor is refused), non-finite input refused.
  - `cast_ray_closest(origin, translation)` fills a `ymir_box3d_soa_ray_hit` {hit, lane, stable_id, point, normal,
    fraction}: Box3D's closest hit along origin + t * translation, t in [0, 1], default filter; a miss is success with
    `hit` 0 and every field 0; a non-finite or zero translation is refused. Anchors are never hit.
  - Managed: `AddAnchor`, `SetAnchorTarget`, `AddMotorJoint`, `ApplyImpulse`, `CastRayClosest` (returns
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
- Concurrency: see Threading below; world creation and destruction are serialized by Ymir's native lock.

## Tests

`tests/Ymir.Box3D.Soa.Tests`, run serially: gravity and rest read through one held view, address stability
across steps, two runs byte-equal at six checkpoints (25 spheres colliding on a slab), a one-ulp input
difference diverging, argument refusal, dispose; and for joints: hinge angle and the relative-rotation
lanes agreeing, an off-axis hinge, a limit and a weld, a spherical joint leaving the swing plane, force and
torque lanes (exact acceleration, cleared after the step, static bodies ignoring them, NaN refusing the
step), capsule and box rest heights with parent and limb lanes, refusals, and bit-identical repeat runs of
a torque-driven jointed body. `Box3DSoaGrabTests`: a strong grab reaching a target 1 m away within 1 s, a weak grab falling at least `g - F/m`, an anchor that does not collide and is not hit by a ray, force lanes not moving an anchor, an off-centre impulse (linear `J/m` and the spin's sign and size), the ray's nearest body with lane, stable id, point and fraction, refusals changing nothing, and bit-identical repeat runs with a moving anchor and an impulse. `Box3DSoaContactTests`: a drop gives one begin and one hit (speed within 5%
of sqrt(2gh), point and normal), lifting gives one end, resting `last_substep_force` is the weight, the hit `impulse_estimate` (a free sphere on a slab, and two free spheres at mass ratios 1:1, 1:10, 10:1) matches the ball's momentum change
(plus gravity's share) at 60 and 240 Hz, 1 and 4 sub-steps, restitution 0 and 0.5, dropped counters resetting each step, materials on
hits, facts lasting one step, overflow counted, per-step byte determinism of facts and contact lanes, and
no allocation across 100 steps with fact reads.

## Not done

Body removal with a stable ordering rule, compound bodies
and per-shape material, and motor joints with a velocity target or per-joint force readback (the motor joint here is a position spring only; Lodur drives limbs from torque lanes).
`Ymir.Core` has no session-level surface over the SoA world yet; consumers use `Ymir.Box3D` directly.
Unity consumption is unrun; cross-platform bit-identity is unclaimed.

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
