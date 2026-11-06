# Headless SoA world

A headless 3D Box3D world with a structure-of-arrays state view, for Lodur and Bad Day Simulator.
Box3D decides every physical outcome; Ymir creates bodies, steps the world and exposes state.

## Why it is not the retained session

`ymir_box3d_session_*` (`ymir_box3d.c`) is the planar Aetheria session: gravity is zero, Y is locked,
only yaw rotates (`motionLocks.linearY/angularX/angularZ`), and state leaves by `copy_bodies` into an
array of structs. A body cannot fall or tumble there, and every read is a copy. The SoA world is a
separate native object (`ymir_box3d_soa.c`, ABI 1) so the Aetheria session's contract is untouched.

## Contract

- Native: `ymir_box3d_soa_create(capacity, gravity) / add_sphere / add_static_box / step / get_view / destroy`.
  Capacity is fixed at creation, so lane pointers never move.
- Lanes (`ymir_box3d_soa_view`): `stable_id`, `kind`, position xyz, rotation quaternion xyzw, linear
  velocity xyz, angular velocity xyz. One contiguous array per component, indexed by body.
- Lane contents change only inside `step`. Between steps a caller may read any lane in place.
- Managed (`src/Ymir.Box3D/Box3DSoaWorld.cs`, internal): `Box3DSoaView` exposes each lane as a
  `ReadOnlySpan<T>` over native memory. Spans are invalid after `Dispose`. `Count` is as of when the
  view was taken, so take it after the last add.
- Determinism: one Box3D worker, bodies stepped in insertion order, same inputs give identical lane
  bytes on the same build. It is not claimed across platforms or CPUs.

## Tests

`tests/Ymir.Box3D.Soa.Tests`: gravity and rest read through one held view, address stability across
steps, two runs byte-equal at six checkpoints (25 spheres colliding on a slab), a one-ulp input
difference diverging (so the equality check can fail), argument refusal, dispose.

## Needed next for Lodur

Recorded as a mind follow-up; summary: joints (Box3D has revolute, spherical, weld, motor), a body
layout beyond spheres and static boxes (capsules, boxes, compound, per-shape density), a torque/force
write path (per-body lane written before the step, as the planar session's `apply_torque`), joint
state lanes, contact facts for the SoA world, and a public surface in `Ymir.Core` rather than an
internal wrapper.
