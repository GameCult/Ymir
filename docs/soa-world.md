# Headless SoA world

A headless 3D Box3D world with a structure-of-arrays state view, for Lodur and Bad Day Simulator.
Box3D decides every physical outcome; Ymir creates bodies, steps the world and exposes state.

## Why it is not the retained session

`ymir_box3d_session_*` (`ymir_box3d.c`) is the planar Aetheria session: gravity is zero, Y is locked,
only yaw rotates (`motionLocks.linearY/angularX/angularZ`), and state leaves by `copy_bodies` into an
array of structs. A body cannot fall or tumble there, and every read is a copy. The SoA world is a
separate native object (`ymir_box3d_soa.c`, ABI 16) so the Aetheria session's contract is untouched.

## Contract

- Native (ABI 16): `ymir_box3d_soa_create(capacity, joint_capacity, gravity, fact_capacity, hit_threshold, particle_capacities*) /
  create_particles / destroy_particles / create_distances / create_bends / create_sensors / attach / remove / set_material / free_slots /
  add_body / add_joint / step / get_view / get_facts / set_anchor_target / destroy_joint / set_joint_thresholds / remove_body / free_joint_slots / free_body_slots / apply_impulse / cast_ray_closest / destroy`. `add_body` takes one descriptor (sphere, capsule, box, static box, anchor; position,
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
- Joint stress: `joint_constraint_force_xyz` (N) and `joint_constraint_torque_xyz` (N m) are Box3D's constraint solution for the joint at the end of the last step
  (`b3Joint_GetConstraintForce/Torque`, world frame; a hung mass at rest reads m g within 2%; a removed lane reads 0). The joint desc's `force_threshold` and
  `torque_threshold` (+inf, the default, is never; NaN or a negative value is refused; `set_joint_thresholds(joint, generation, force, torque)` changes a live joint's)
  make Box3D report the joint in a step where its load reaches one: an Overload fact `{joint_lane, joint_generation, force xyz, torque xyz}` (the load read after the step), at most once per
  joint per step. The threshold is a sensor: Ymir never removes a joint on it, the caller decides with `destroy_joint`. Joint user data is lane + 1.
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
- Overload and removal facts (buffers fixed at create like the rest): `overload` (above), `body_removed` `{lane, generation (the dead handle's), stable_id, wet_patches, wet_volume}`
  and `joint_removed` `{lane, generation, body_a, body_b, with_body}` (1 when a removed body took the joint, 0 for `destroy_joint`). `remove_body` and `destroy_joint` write them
  into pending arrays and the next `step` publishes them in removal order (a lane is removed at most once between steps, so they never overflow), with the End touch facts the
  removal caused: a removed body's shape is no longer valid, so its End facts are attributed from the retired shape table `{shape, lane, stable_id}`, cleared once the step has
  collected them. Every removal path (a grab release, a cut, a gib, a level unload) appears in these facts. A step refused for a non-finite write lane publishes nothing and keeps them.
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
  - Release: the cursor's anchor is long-lived (a client creates one per cursor and keeps it, or removes it with `remove_body`).
    `destroy_joint(joint, generation)` releases a joint of any kind (a grab, a cut, a tear) through the one joint-ending path, `ymir_soa_retire_joint`,
    the only `b3DestroyJoint`; the bodies stay. The joint's lane becomes a tombstone: `joint_kind` 4
    (Removed), `joint_body_a/b` -1, every joint lane 0 (the constraint lanes included), generation + 1; no last state is kept, a caller that wants it reads before
    removing. `joint_count` counts slots ever used, tombstones included.
    `add_joint` reuses the lowest tombstone only after a step has completed since the destroy, so lanes read within a
    step stay valid; until then the slot is not free. `free_joint_slots` (managed `FreeJointSlots`) is what
    `add_joint` could create right now: check it, and `FreeBodySlots`, before adding an anchor for a grab, so a full
    world leaves no orphan anchor. Repeated grab and release on one slot do not grow the world.
    A joint's handle is its lane and the slot's generation (`joint_generation` lane, counts destroys of the slot;
    managed `Box3DSoaJointHandle`, returned by `AddJoint` and `AddMotorJoint`, converts to the lane). `destroy_joint`
    refuses a generation that is not the slot's, a removed lane and an out-of-range lane, so a second release click with an old handle never destroys a later
    grab that reused the slot. Anchor creation must be followed by a joint that validates (its spring fields are
    checked there), or the anchor is left behind: with one long-lived anchor per cursor that is accepted, and an atomic
    `AddGrab` is a follow-up only if cursors become dynamic.
  - Body removal: `remove_body(lane, generation)` (managed `RemoveBody(Box3DSoaBodyHandle)`) is the one body-ending path. A body's handle is its lane
    and the lane's generation (`body_generation` lane; managed `Box3DSoaBodyHandle`, returned by every `Add*` body helper, converts to the lane, so lane arrays and `int`
    parameters take it unchanged). It is refused whole, changing nothing, for a lane that is not a live body, a stale generation, and with status REFERENCED (6; managed: it
    throws) while a live body attachment or liquid sensor names the body: the caller removes those first (an anchor is a body and gets no rule of its own: its attachments
    and sensors count the same). Otherwise, in order: the body's wet patches are removed (`ymir_particles_release_body`, no Box3D call), every joint naming the body is
    retired (ascending lane, so a removed anchor ends its grabs and their handles go stale), the body's shape is entered in the retired table, Box3D destroys the body and the lane
    becomes a tombstone: `kind` 5 (Removed), every other lane 0 (stable id 0, parent -1, limb 0), generation + 1. Lane order never changes (no swap-remove; `count` is the
    high-water mark of lanes ever used). `add_body` takes the lowest Removed lane whose step has completed, else appends, else FULL; a lane is never reused before the step
    whose facts name it dead, and the removed stable id may be reused. `free_body_slots` (managed `FreeBodySlots`) counts them. `attach`, `create_sensors`, `add_joint` and a
    new body's `parent` refuse a Removed lane; a live child's `parent` may still name one (the consumer owns its tree). `particle_support_body` is a last-step fact and may name a
    removed lane until the next substep; a drop cannot settle on it (its proxy reads no shape). Box3D's own joint cascade in `b3DestroyBody` never runs: the joints are retired first.
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
  of `ymir_particles_begin`, `_substep` and `_end`). The particle state is a constraint graph: particles are one lane set with a
  phase word and every constraint type is a lane set of its own. There is no object above the lanes: a rope, a sheet, a gut or
  flesh is whatever constraints a caller emitted, and exists only in that caller's builders. A world created with a
  `ymir_particle_capacities{particles, distances, bends, attachments, tethers, materials, fluid, wets}` that is not all zero owns fixed
  lanes that never move; created with none (`NULL` or all zero) it behaves byte-identically to ABI 8. Nothing writes a
  particle's position or velocity but the step.
  - Data model. Particle lanes: `x y z vx vy vz radius inv_mass rest_x rest_y rest_z phase material live generation removed_at
    component`. The phase word: bits 0-19 are the group, bit 20 `SELF_COLLIDE`, bit 21 `SELF_FILTER`, bit 22 `FLUID`, bit 23 `NO_PARTICLE_CONTACT` (refused together with `FLUID`); any other bit is refused.
    Distance lanes: two particles, rest length, compliance (m/N), `tension` (sensor). Bend lanes: two distances that share one
    particle (and so the three particles a, b, c), rest and limit offsets, compliance. Attachment lanes: kind (body, static,
    removed), flags (`TETHER`; `LIMB`, Body only: the particle is flesh of the body's limb tree, so contact skips every body whose limb-tree root, the `parent` lane followed to its end, is the attached body's), particle, body, point, compliance, `force` (sensor). Held lanes (derived, command time): per particle the list of bodies its live Body attachments name, which contact skips, and `limb_body`, the body of its lowest live `LIMB` attachment or -1. Tether lanes (derived): attachment,
    particle, bound, parent tether, edge, other attachment. Material table: friction, XSPH viscosity, wet friction and the caller's material id. Handles are
    (lane, generation).
  - Commands, each refused whole before anything changes: `create_particles(n, desc{position, velocity, rest, mass, radius,
    phase, material})` takes the lowest free lanes; `destroy_particles(n, handles)` removes every constraint that touches them;
    `create_distances(n, desc{a, b, rest (< 0 = rest-pose length), compliance})`; `create_bends(n, desc{e1, e2, compliance
    (< 0 = soft pass off), max_angle (< 0 = no limit)})`, refused unless the two distances share exactly one particle and the
    bend does something; `attach(desc{kind body|static, flags, particle, body, point, compliance})`; `remove(type distance|bend|
    attachment, n, handles)`; `set_material(index, desc{friction, viscosity (>= 0), material_id})`; `free_slots(lane set)`. A distance takes
    the bends over it with it. Ymir never removes a constraint or releases an attachment on a load: tear, cut, gib and release
    are the caller's say-so.
  - Derivation. Every topology command ends in the one `rederive_topology()`, which is the only writer of the tether lanes and
    the component lane. It builds the adjacency of the live distances, labels components by their lowest lane, and then islands
    (Kim, Chentanez, Mueller 2012 sec. 3.4): a particle holding a live `TETHER` attachment is a tether particle, and an island is
    a maximal set of tether particles joined by live distances, named by its lowest `TETHER` attachment. For each island, in lane
    order, one Dijkstra (binary heap, ties by node then edge) over the distances' rest lengths runs from all its tether particles;
    another island's tether particle is settled but not expanded. Each particle that is not a tether particle keeps the two
    nearest islands by rest path (an equal path keeps the lower island) and gets one tether to the nearest point of each, bounded
    by that geodesic path (not the Euclidean rest distance: 0.23% on a flat two-corner sheet, but a coiled rope could never uncoil under
    it, and the graph has no surface to say when it holds), with the tether of the previous particle on the path and the edge used (parents come before children). Ranking
    attachments instead of islands fails when a corner is held by adjacent pins (two corners of two pins stretch 11.6% at 5 s by
    the two nearest attachments, 2.4% by islands); one island per particle (nearest only) stretched 16.4%, and Kim's typical four
    islands buy 1.66% against 1.72% on a 20-pin sheet for twice the lanes, so the count is `YMIR_TETHER_ISLANDS` = 2. Float32
    path sums can break the exact-arithmetic closure of the rankings at a tie (a particle keeps an island its parent dropped), so
    a tether is written only when its parent particle is a source or holds a tether to the same island: a gated particle keeps
    one tether. So N particles carry at most 2N tethers however many points are pinned: a 32 x 32 sheet pinned at 20 points had
    20650 tethers (3.5 ms per tick) and has 2008. Two `TETHER` attachments on different particles of one component, at least one
    on a body, also get a pair tether: the rest path between their particles bounds the distance between their points (what lets
    a heavy body be held at 0.1% stretch from a static point: the full excess moves the body, not only the light end particle).
    Two static points never move, so their pair has no end to pull and is not derived. A pair tether sits behind a route of booking lanes (attachment -1, an edge
    and a parent, no bound of their own) along that path, so the tension sensors read its load on the edges it crossed; pairs
    cost O(A^2) lanes and a derivation runs one more Dijkstra per tether point that has a later partner. Hold a heavy body with
    `TETHER` at both ends: with the tether on one end only a 70 kg body hung from a 48-particle rope sinks 0.25 m below its
    attachment (speed 1.1 m/s at rest), with both it holds to 0.002% stretch. The tether count is known before anything is
    written (the Dijkstras run once to count and once to write); a command that would need more than the tether capacity is undone
    whole and refused (`create_distances` and `attach` are the commands that can).
  - Lanes. Particle, distance, bend and attachment lanes are tombstones (live 0 or kind removed, generation + 1, every other lane 0
    except `removed_at`) reused only once a step has passed, lowest lane first. Stale generations are refused. Counts are high-water
    marks; a view's counts are read when the view is taken, so take it after the last command.
  - Solve (inside the fork's substep callback, `h = dt / substeps`, one iteration of each pass per substep: the small step does
    the converging). Order: predict particles and proxies, hash grid (the non-fluid lanes), pair contacts (Jacobi), the fluid pass (its own grid; fluid-solid contact), attachments, distances in
    lane order, bends in lane order, tethers, body contacts last (with contacts first a box pushed into a rope reads 14% of a
    radius inside). A bend works on the offset of its shared particle b from the centroid of a, b, c (Kelager 2010): a soft XPBD
    pass toward the rest offset, then a hard unilateral pass at the offset `max_angle` allows (computed from the two rest
    lengths), free inside it and rigid at it. It bends on any topology a caller builds, and the limit is a hard constraint at the
    substep the solver runs: how stiff it reads at few substeps is the open question `bend-limit-stiffness`.
  - Contact. Particles of different groups collide; of one group only when both set `SELF_COLLIDE`, and not (when either sets
    `SELF_FILTER`) if their rest-pose distance is under 1.5 (ri + rj). A `NO_PARTICLE_CONTACT` particle meets bodies and fluid
    only: it is left out of the solid grid, so no non-fluid particle touches it, while body contact and the fluid grid still
    hold it. The solid grid is sized by its members (a cell's particles are walked in lane order whatever the bucket count, so
    the sizing moves no byte); the fluid grid keeps the full table, which `fluid_density`'s sum order depends on. Each pair is found once, walking half the 27-cell
    neighbourhood from each particle (its own cell for higher lanes, 13 cells forward), in two walks: the first counts each
    particle's overlaps; the second moves both ends of a pair by one displacement, weighted `1 / max(n_i, n_j)`, so a pair's two
    corrections carry equal and opposite momentum and contact conserves it (to float32 rounding of the positions: 0.01% in the
    thrown scene, 0.6% at a 30:1 particle mass ratio and 6% at 100:1). No pair list is kept: its size is not bounded by the particle
    count. Body contact: a particle against each shaped body in lane order, the correction
    split by inverse mass at the contact point, Coulomb friction from the particle's material; every body a particle is attached to
    is skipped.
  - Fluid. Fluid is a phase bit, not an object: a particle with `FLUID` takes part in a number-density constraint with every
    other fluid particle and with the solids it would touch by contact, and keeps contact (at `ri + rj`) with those solids; two
    fluid particles do not contact, whatever their groups and `SELF_*` flags (all fluid shares one density field, Macklin et al.
    2014 sec. 7). Kernel per pair `h = 2 (ri + rj)` (4 r at equal radii), Poly6 `W = 315 / (64 pi h^9) (h^2 - r^2)^3` for the
    density and Spiky `grad W = -45 / (pi h^6) (h - r)^2 r_hat` for the gradient (Macklin and Mueller 2013); no PhysX code is
    adapted. The rest density `n0(h)` is the Poly6 sum over the simple-cubic lattice of spacing `h / 2` (a constant, computed at
    create, times `h^-3`), so a particle has volume `(2 r)^3` and there is no rest-density field. The constraint is unilateral,
    `C_i = max(n_i / n0 - 1, 0)`, so there is no artificial-pressure term and an under-dense fluid (a gas of far particles) is not
    pulled together; `lambda_i = -C_i / (sum_k w_k |grad_k C_i|^2 + eps w_i)` (0 when that is 0; XPBD inverse-mass weighting, Macklin et al. 2016) with the gradients over `n0` in 1/m^2 and `eps = 1e-4` (a sweep of
    1e-6 to 1e-2 on the column test moved the max density between 1.344 and 1.377 and nothing else: the gradient sum is about 500).
    Per substep, after contact, the fluid pass rebuilds the grid over every live lane at the kernel cell `4 r_fmax` (the same tables, in place; nothing after the
    pass reads the contact grid) and copies position, inverse mass and phase into sorted order, so a bucket reads contiguously (`sorted[q]` is the lane of sorted index `q`).
    `fluid_rings = ceil(2 (r_fmax + r_max) / 4 r_fmax)` (1 at equal radii: 27 cells). Pass A takes each fluid row in sorted order: the buckets of its cells, each bucket once (a
    bucket two offsets hash to is skipped the second time, by a visit stamp; a particle of a cell outside the rings is out of reach, so the distance test needs no cell
    compare), a branchless compaction of the candidates within `2 (r_i + r_max)` into a stack row buffer of 128 (hits past it add to `fluid_neighbour_overflow`), and over the
    buffer `n_i` (self and the solids it would touch by contact counted, `s = 1` as Flex 7.1) and `lambda` (a solid's is 0), and the row's neighbour list: every kernel neighbour
    in walk order, packed (CSR) into an arena of `fluid_capacity * 96` entries (about 33 neighbours at rest at 4 r reach, about 45 at the column's compression); each past the
    arena adds 1 to `fluid_neighbour_overflow`, the ABI 12 view scalar (last step, summed over its substeps), so `n` and `lambda` stay exact until the row buffer overflows and
    only the correction and XSPH lose the pairs past the arena. `fluid_capacity` (`ymir_particle_capacities.fluid`, `Box3DSoaParticleCapacities.Fluid`, 0 by default) is the most
    live fluid particles: `create_particles` refuses a batch past it whole (`FULL`, no lane taken). Pass B reads the lists: each fluid row in index
    order, each listed neighbour in list order, a fluid neighbour below the row skipped (its own row does the pair), a solid always done from the
    fluid row; a pair moves by `d = (lambda_i + lambda_j) grad W / n0(h)`, end `i`
    by `w_i d` and end `j` by `-w_j d`, applied after the pass (equal masses move as before; a static end takes nothing and the fluid its XPBD share, so the energy at 30 : 1 and 100 : 1 does not rise). A pair therefore
    carries equal and opposite momentum at any mass ratio (a solid end is the reaction: `m_i dx_i = -m_j dx_j`), a zero-mass end
    takes nothing, and the one pair formula serves fluid with fluid and with solids. Then the fluid-solid contact (the contact pass never sees a fluid lane: the contact grid holds
    the other lanes only, at `2 r_max`): two passes over the fluid rows' lists, the first counting the overlaps of each listed solid at `ri + rj`, the second moving both ends as the contact
    pass does, weight `1 / max(n_i, n_j)` over these pairs only, applied after (Jacobi); PhysX solves fluid-solid in its fluid pass the same way. The XSPH viscosity of Schechter and Bridson
    runs after the velocity update over the same lists (fluid neighbours above the row), `dv = c (v_j - v_i) W / n0` with `c` the mean of the two
    materials' `viscosity`, split by inverse mass the same way, and only when some material has a viscosity above 0 (one scan of the material lanes a substep). A pair that
    entered reach since pass A is missed by XSPH (smoothing, not a constraint). Lists, copies and the fluid grid are per-substep scratch, carved once at create: 28 bytes a particle lane
    (`nbr_start`, `nbr_count`, `sx sy sz sw sphase`; h1's per-lane lists were 388) and 384 a unit of fluid capacity. A world with no live fluid steps byte-identically to
    one before fluid (no pass runs: `live_fluid` counts the live fluid particles, written by create and destroy only). Cost on the 16 x 16 sheet scene at 4 substeps, increment over the same scene without fluid (Release, three runs; the cost line in `Box3DSoaCouplingTests` now prints the median of nine interleaved with-and-without pairs and its interquartile range):
    512 fluid +0.76 to +0.84 ms a tick at viscosity 0 and +0.86 to +1.01 at 0.01; 2000 fluid +4.1 to +4.4 at both, against a target of 1.0 ms at 512 (h1: +3.0 to +3.4 and +13.1 to +13.9). The probe behind the design
    (a standalone replica of the pass) read 7 ns a cell visit in the 125-cell walk: the cost was per cell, not arithmetic, so the cell is now the kernel reach and the copies and the branchless filter keep the 27 buckets cheap. Two counting sorts a substep run with fluid live.
    sensor lane `particle_density` reads `n_i / n0` (1 at rest, 0 for a solid) in the last substep the particle ran.
    Support lanes `particle_support_body` and `particle_support_particle` (ABI 13; `SupportBody`, `SupportParticle` in the binding) name what a fluid particle touched in the
    last substep it ran. Both are cleared to -1 for each live fluid lane at the start of the substep, after prediction, and written by the substep's contacts: body contact
    sets `support_body` to the lane of each shaped body whose surface comes within the particle's radius, bodies ascending, so the highest lane touched wins; the fluid-solid
    contact pass sets `support_particle` to the first solid listed in the particle's row that touches it (pass 1 of the walk, the solid's lane). A fluid particle resting only
    on fluid reads -1 on both, so the bottom of a deep pool is supported and the interior is not; a solid lane, a free lane and a lane after create or destroy read -1 too.
    The lanes name contact, not rest: the caller judges speed. They are notification only, written by the step and read by no pass; a world with no settled liquid
    steps byte-identically to ABI 12. `destroy_particles` writes -1 to a destroyed lane's support lanes and to the `support_particle` of every live drop that named a destroyed solid, so no read after a destroy
    names a dead lane.
  - Settle (ABI 14). Liquid at rest becomes data: `settle(n, handles)` (`Box3DSoaWorld.Settle`) turns each named fluid particle into volume in a wet patch and destroys the particle (through
    `destroy_particles`, the ISOLATED derivation reused: fluid holds no constraint). A patch is a disc of settled liquid on one support: a body (`wet_body`, its point in the body's frame: the
    particle's nearest surface point from the proxies, which the command refreshes first, so it follows the body) or a solid particle (`wet_particle`, point 0); the other support reads -1. Lanes
    `wet_body wet_particle wet_x wet_y wet_z wet_radius wet_volume wet_material wet_live wet_generation wet_removed_at` (capacity `wets`, `YMIR_LANES_WET` = 4 for `remove` and `free_slots`, lanes
    tombstoned and reused as the others). A drop's support is its last step's `support_body` (else `support_particle`); one with neither is refused. A drop's volume is `(2 r)^3`, its patch's radius
    `sqrt(volume / (pi film))` with `YMIR_WET_FILM` = 1 mm (one constant; a film per liquid is not added). In handle order a drop joins the lowest live patch on the same support and material whose
    disc holds its contact point (volumes add, the centre is the volume-weighted mean, the radius is recomputed); else it takes the lowest free lane; else it joins the nearest patch of the same support
    and material; else the batch is refused `FULL`, whole, and every lane is as it was (an undo log sized by `fluid_capacity`). A dead or non-fluid particle, a stale handle, a particle named twice
    or one with no support is refused whole too. `wet_volume` per body (the view's `WetVolume`, `body_wet_volume` natively) is the sum of the live patches on the body in wet lane order, written by
    `rederive_wet` alone, which `settle`, `remove(WET)` and `destroy_particles` (when it kills patches on the solid particles it destroys) call; the step never writes a wet lane or the volume. `remove(WET)` kills patches and ends in `rederive_wet`,
    never in `rederive_topology`; `destroy_particles` kills the patches on the solid particles it destroys. `settle` and `remove` return the patch handles to read. Material `wet_friction` (>= 0) is stored
    and viewed (`MaterialWetFriction`), and lowers friction where the liquid lies (below). Which particles settle, and when, is the caller's policy
    (speed, age, film depth: `lodur:follow_up:bds-blood-settle-policy`); Ymir holds the data and the command.
  - Sensors (ABI 15). A liquid sensor is a sphere fixed in a body that reads the liquid volume inside it: `create_sensors(n, desc{body, point, radius})` (`Box3DSoaWorld.CreateSensors`; `point` in the body's frame, `radius` m,
    capacity `sensors`, `YMIR_LANES_SENSOR` = 5 for `remove` and `free_slots`, lanes tombstoned and reused as the others) refuses the whole batch, changing nothing, for a body outside the world's body count, a radius that is not
    finite and > 0, a non-finite point, or too few free lanes. Lanes `sensor_body sensor_x sensor_y sensor_z sensor_radius sensor_liquid_volume sensor_wet_volume sensor_live sensor_generation sensor_removed_at`; a free lane reads
    live 0, body -1 and 0 elsewhere. At the end of every step `ymir_particles_end` writes `sensor_liquid_volume` (m^3): for each live sensor in lane order, the sum in lane order of `(2 r)^3` over the live fluid
    particles whose centre lies within `radius` (distance squared <= radius squared) of the sensor's centre, the body's proxy position plus the proxy rotation applied to `point`, in the pose the last substep left the
    proxy in; 0 when no fluid is live. A second lane, `sensor_wet_volume` (`SensorWetVolume`), is the pooled liquid: the sum of the volumes of the live wet patches whose centre lies within `radius` of that centre, on any body or
    solid particle (a body's patch centre is in its body's frame: the sphere is taken into that frame, culled against the body's disc box and walked through the wet index by centre x, bodies ascending; a patch on a solid particle
    sits at the particle's position and is summed after, in wet lane order, by a scan of the wet lanes, which have no index). Two numbers, not a sum, so a consumer weighs pooled and moving liquid apart
    (`lodur:ruling:liquid-sensor-reads-pooled-liquid`). Both are end-of-step readings: a `settle`, `destroy_particles` or `remove` between steps shows at the next step, which is the contract. The pass runs before `ymir_particles_end`'s early return for no live particle, so a sensor never keeps a reading once the liquid is gone. The readings are sensors: the step
    writes them and no pass reads them back, so a world with sensors steps byte for byte as one without, and a world with none steps as ABI 14. `remove(SENSOR)` ends in no topology derivation. Ymir names no organ
    (a head's airway, a lung): the consumer places a sensor and maps the volume (`lodur:follow_up:lodur-airway-liquid`). Gives up: a sphere only, not a shape-exact volume, and no per-liquid split.
  - Wet friction. Settled liquid lowers friction where it lands, per contact, never per shape: `wet_mu(body, point, mu)` is `min(mu, wet_friction)` of the lowest live patch on that body whose
    disc holds the point (the point in the body's frame, as `wet_x wet_y wet_z` are), and `mu` at once when the body's `wet_volume` is 0. It is the one patch test. A particle that touches a body
    uses it at the nearest surface point for a solid particle (also `min` with `particle_wet_mu[i]`, the wet friction of the lowest live patch on that solid particle, +inf when dry, derived by
    `rederive_wet`); a fluid particle keeps its material friction. Body-body contact goes through the GameCult Box3D fork's `b3World_SetContactFrictionCallback`: `b3UpdateConvexContact` calls the
    hook right after the mix (`sqrt(frictionA frictionB)`) with the mean of the manifold points in each body's frame (one point per manifold), both bodies' user data (lane + 1) and the mixed value,
    and the world's hook returns `wet_mu` on B of `wet_mu` on A of it: it only lowers. The read is race-free (patches change only between steps) and a world with no settled liquid steps byte for
    byte as before. Mesh contacts are not hooked (Ymir makes none). The fork runs the hook on a recycled contact too (Box3D skips a recycled contact's update, and with it the friction mix): it mixes again and calls the hook, only when a hook is installed, so friction follows a patch settled or removed under a body at rest. The lookup is culled: `rederive_wet` indexes the live body patches by body then centre x with each body's box and largest radius (`wet_order`, `wet_body_start`, `wet_body_count`, `wet_body_box`), so a point outside the body's box is a constant check and one inside binary-searches the x window; the lowest lane still decides. The fork's own tests: `test.exe WorldTest` (the unfiltered run stops at CompoundTest, which fails at the fork's base as well). No ABI change (14).
    Fluid holds no constraint: `create_distances` and `attach` refuse a fluid particle, so `create_bends` (over distances) can
    never name one; a thing that holds liquid is cloth around free fluid (a balloon), as in Flex sec. 8. Every `create_particles`
    and every `destroy_particles` of particles that held no distance derives only the isolated lanes
    (`rederive_topology` with the ISOLATED mode: `component[i] = i` for created lanes, 0 for destroyed, tethers untouched): emitting
    and removing 64 fluid particles a tick on the 20-pin 32 x 32 sheet costs 0.3 ms a tick of commands where the full derivation
    costs 4.2 ms.
  - Coupling (unchanged): one proxy per body (pose, velocity, shape, real inverse mass and world inverse
    inertia). Before each rigid substep's positions integrate, `b3Body_GetSubStepState` fills the proxies and `b3Body_ApplySubStepImpulse`
    delivers each dynamic body's correction as an impulse; the `particle_impulse` lane is the tick's total. Bodies are never
    teleported; no `b3Body_Apply*Impulse` or `SetTransform` runs from the particle path. The gate is the live particle count.
  - Sensors. Every correction is `lambda / h^2` in newtons, one scale. Distance `tension` is read by momentum balance: the
    constraint's own correction, the bends over it and the tether impulses routed leaves-first down each path tree each book their
    position impulse along the edge, over `h^2`, mean over the substeps; it reads the weight below each edge within 1% from 1 to
    70 kg whichever way the load is routed. Attachment `force` is the sum of the corrections at the point, tether load included. Loop
    caveat: a sensor reads the solver's corrections, and a load that circulates in a closed loop of the graph (a sheet pinned at
    four corners) is read as the corrections it needs, not as a statically determined split.
- Particle determinism: single thread; constraints Gauss-Seidel in lane order; the one Jacobi pass (pair contact) accumulates corrections and applies them after the walk (each pair once, from the
  lower of its two cells, neighbours read in cell then index order from a stable counting-sort grid, a bucket shared
  by two cells is told apart by the cells' coordinates, a pair weighted the same at both ends); the heap and the breadth-first
  labelling break ties by lane; proxy impulses are summed per body in lane order, per substep; only `+ - * /`, `sqrtf` and `cosf`
  (at command time); the file builds with `-ffp-contract=off` (`/fp:precise` on MSVC). Same bytes on the same build, serial or with
  other worlds stepping on other threads. No threads in this cut.
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

`Box3DSoaRemoveTests`: removing a body retires its joints and zeroes its lane (old handles stale, others untouched, facts one step), is refused while an attachment or a liquid sensor names it, each alone and for an anchor too (nothing changed; after the caller removes them it succeeds; attach, sensors, add_joint and a parent then refuse the lane), refuses a current-generation handle to a tombstone (destroy_joint and remove_body) and lets an add reuse stable id 0, skips a removed lane in the particle substeps and the write-lane check (its write lanes are still cleared), names every removed toucher over 200 single-removal steps in a two-lane world (the retired table is cleared each step), survives 12 seeds of random add/remove churn on a floor (run in Debug too: Box3D validates its broad-phase there), takes its wet patches with it (count and summed volume on the fact, another body's patches untouched), ends a grab when the anchor goes, ends a touch by name, leaves a drop unable to settle, keeps the other lanes' state, reuses a lane one step later with a new generation, repeats bit for bit with removals and thresholds over 300 steps and allocates nothing managed. `Box3DSoaJointTests`: the constraint lanes read m g, an overload is reported once its load reaches the threshold and the joint stays, thresholds are refused when NaN or negative. `Box3DSoaGrabTests`: every joint kind destroys through one path. `Box3DSoaParticleTests`: the ABI 8 scene's six checkpoint hashes with no particle capacity (pinned from 5104b17); a 48-particle rope (a builder in the tests: distances and soft bends) hanging from a tethered static point under 1% stretch for 4 s; a 70 kg sphere held by the same rope (from 1 s to 5 s speed under 0.001 m/s, stretch under 0.1%, the impulse per tick within 0.5% of m g dt); an anchor dragging a 5 kg box along the floor (`particle_impulse` reads it); a sphere, capsule, box and static box each pushed into a lying rope's end (no particle over 10% of its radius inside); a 16 x 16 sheet from the same two commands hung by two corners (after 5 s the corners carry the weight within 1% and no edge is past 5%), and by two corners of two pins each (after 8 s, same bounds: it still rings at 5 s); edge tension, the edge at 0, 10 and 46 and both attachment forces read the load at 1, 5, 20 and 70 kg; a cut (remove an edge of a hung 40-rope: the bends over it die, tethers 39 -> 19, the component splits, the head holds and the tail falls); overload (300 kg) removes nothing; destroying a mid-rope particle removes its constraints and releases a held body; a 24-particle cantilever at 32 substeps under a 30 and a 10 degree hard limit and a stiff and a floppy soft pass, and a limit-only bend bit-identical to no bend inside its cone; momentum of a rope, a sheet and a blob thrown into each other with no gravity within 5e-4 and energy never rising; phase rules (groups, `SELF_COLLIDE`, `SELF_FILTER`, a folded rope); an off-centre attachment spinning a free box by I^-1 (r x J) within 2%; lanes reused after a step lowest first, stale generations refused, a derivation that does not fit refused with no lane changed, bad input refused; a pile of four ropes and a sheet on two bodies byte-equal across two runs at six checkpoints (pinned), a one-ulp difference diverging, and eight concurrent worlds giving the serial bytes; no allocation across 100 steps and across a cut, a destroy and a create, and none for a 300-distance batch created and removed; gravity along each axis; a spinning box carrying its attachment point and two ends on one light box settling (the proxy's rotation); one group's `SELF_COLLIDE` needing both particles and `SELF_FILTER` either; a slack pair tether leaving its body alone; a refused `attach` leaving no lane and no tether; each particle on the nearest point of its two nearest islands (a 5-rope held at both ends, a star whose centre keeps the two nearer arms, a 9-rope with two touching points counting as one island) and no pair between two static points; every tether's parent present after each of 20 attaches on a 32 x 32 sheet (float ties at rank 2 gated); a particle held by two bodies touching neither. `Box3DSoaParticleSourceTests` reads the C source: no Box3D call in `ymir_particles.*`, no allocation outside create and free, no threads or atomics, no rope, sheet, cloth or chain identifier in the solver or the binding, the tether and component lanes written only by the derivation, every topology command ending in it, `begin` and `end` called once (in the step), `substep` once (in the callback), the callback the only user of the fork's two accessors, no topology change in the phases, and the retired names absent.

`Box3DSoaCouplingTests`: a sphere thrown at 5 m/s with a 48 x 20 g rope from rest keeps x momentum within 0.1% and gains no energy, at 5 and 70 kg; thrown up it reaches the apex momentum allows; 5 kg snapping a taut rope at 6 m/s keeps at most 5% of its energy and 70 kg is still within a tick; a 45 degree pendulum for 12 s gains no energy and loses at most 1.25 times a heavy-particle control, and a hung body keeps its spin about the vertical; removing the body attachment or destroying the rope's particles frees a held 70 kg body in the step after, with no upward impulse; `particle_impulse` equals M dv minus M g dt every tick, hung and thrown; material friction 0.6 holds a rope on a 20 degree slope and friction 0 does not; no particle is farther from a tethered point than its rest path; an attached end stays on its body point; a world whose only body is static still steps the particles (a dropped rope falls g t^2 / 2 within 2%) and rests; and a cost line with 16 ropes x 48 and one 16 x 16 sheet at 4 substeps, one body and forty (printed, not asserted).

## Not done

`Box3DSoaFluidTests`: the refusals (a distance or attachment naming fluid, a viscosity below 0 or not finite; a batch of fluid past the fluid capacity, refused whole); 64 fluid particles created and destroyed a tick on the 20-pin sheet leaving every tether lane byte-equal; an 8 x 8 x 16 column in a box well (max density, mean density, nothing outside); fluid thrown into a free sheet and rope at fluid : solid mass 64 : 20, 1 : 30, 30 : 1 and 100 : 1 (momentum drift, energy on every row); a 343-particle fluid block dropped 15 cm onto a floor of immovable solid particles at 4 substeps (penetration, nothing through the floor's interior bar the one particle over a lattice hole); probe scene 3, the 343-particle pour on a tilted sheet and a sloped rope at 4 substeps, printed as readings with the same block flagged solid as the control, asserting nothing; fluid pushing a free box with the impulse lane equal to its M dv; a 3 r lattice not clumping; XSPH damping a shear field; the kernel reaching a larger solid beyond two contact radii; the pour's six checkpoint hashes (two runs, a one-ulp nudge, eight concurrent worlds). Each bound states its readings and the mutant it separates from. `Fluid_cost_line` prints 512 and 2000 fluid particles on the sheet scene at viscosity 0 and 0.01. Overflow is asserted 0 in the column, the floor and the pour.

Compound bodies
and per-shape material, and motor joints with a velocity target (the motor joint here is a position spring only; Lodur drives limbs from torque lanes). A one-call cascading `remove_body` that also removes the attachments and sensors naming the body is not built: the caller removes them first.
`Ymir.Core` has no session-level surface over the SoA world yet; consumers use `Ymir.Box3D` directly.
Unity consumption is unrun; cross-platform bit-identity is unclaimed.

Particles: a sheet pinned at few points sieves at 4 substeps (the pour stretches a corner-pinned 16 x 16 sheet 38% and a same-size particle passes it, a same-mass solid block worse; per-substep travel is about 1 cm against a 4 cm contact, so it is the sheet and the contact solver, not the fluid: a loose-cloth limit, ruling ymir-sheet-sieve-at-budget-substeps); a sheet attached along its length to a body holds (`Skin_on_a_bone_holds_under_pour_and_drop`: 30 skin particles on a capsule under a 343-particle pour or 64 dropped solids, 3 s at 4 substeps and 60 or 120 Hz, worst edge stretch 0.021, no load particle inside the bone, skin place held to float noise), so BDS skin needs no more substeps, no boundary density for rigid bodies (a Box3D body adds nothing to a fluid particle's density estimate, so fluid
beside a wall or floor under-reads: a 8 x 8 x 16 column in a box well reads 0.88 mean at zero gravity), no cohesion, adhesion or
vorticity, no settle of fluid to rest (the column keeps a velocity noise, 1.25 m/s at its worst at 3 s with the one pass a substep, 0.82 with viscosity 0.1), a column compresses under its own weight (max density 1.34 on 16 layers), no pinning command beyond zero mass and attachments, no
sleep, no threads, no particle write lanes. A heavy body cannot be held up by contacts against a floor (the particles are squeezed
between two solids a position solver cannot satisfy at a 50:1 mass ratio; the pushing tests drive the body into a rope end
instead). A snap on a taut rope is inelastic by the solver's construction, and a swung weight loses a few percent more as a tied
body than as a heavy particle; neither gains energy.
A hard bend limit is one iteration per substep and is not rigid at the substep counts the tests run; whether to change the solver is
the open question `bend-limit-stiffness`, and the solver is as it was. Measured on a 24-particle cantilever held at two particles, 600 ticks at 60 Hz, the
largest deflection of one joint: a 10 degree limit allows 21.7 degrees at 4 substeps and 22.3 at 8, 13.6 at 32 under a 1 kg tip and
32.0 at 32 under a 5 kg tip; a 30 degree limit allows 31.5 at 4, 33.8 at 8 and 29.9 at 32 under a 5 kg tip. A soft bend is no stiffer
than a floppy one at 4 or 8 substeps (tip drop with no load, stiff 1e-7 against floppy 1e-4: 0.764 m against 0.779 at 4, 0.835
against 0.822 at 8, 0.053 against 0.798 at 32 where it reads as pinned). Read a bend limit as a guide below about 32 substeps and
not at all under a heavy tip. A 16 x 16 sheet hung by two corners rings for about 9 s before the corners carry its weight within 1%
(by two corners of two pins each, about 6 s). Pair contact
conserves momentum to float32 rounding of the positions, which grows with the particle mass ratio (0.6% at 30:1, 6% at 100:1);
the momentum test's 5e-4 is that floor, not a contact property: translating the whole scene moves its drift between 0.56e-4 and
4.0e-4 under either contact walk, and the per-particle-average mutant reads 5.6e-2.
Determinism under MSVC's
`/fp:precise` is unbuilt (no `cl` on the workstation).

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
