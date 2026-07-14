# Box3D Parity Harness

## Authority

Box3D v0.1.0 is Ymir's physics implementation and executable semantics oracle.
The production native session ABI and snapshot lowering are live; explicit
retained-world ownership is still being connected. The exact upstream source is pinned as the
`extern/box3d` submodule at commit
`8441b4a06d6d09dcfb0b0f704df4d847d1437b92`.

Box3D does not own GameCult entity ids, gameplay policy, persistence, or result
ordering. Ymir projects Box3D facts into those contracts. Box3D owns
shape overlap, shape cast, solver, broadphase, contact lifecycle, force and
torque lifetime, and numeric tolerances.

## Body

`native/Ymir.Box3D` statically links the pinned C17 library into one small
shared-library facade. The facade exports retained sphere-world sessions plus
the overlap, cast, pair-step, torque-lifetime, version, and ABI-layout
operations used by the harness. This keeps Box3D's alpha C ABI out of Ymir's
public managed contracts.

Production ABI version 2 uses caller-owned blittable buffers, explicit cdecl,
fixed-width status/count values, compile-time C layout assertions, and a
managed/native layout sentinel before opening a session. Planar direction maps
to Box3D rotation about Y; angular velocity and transient torque remain Box3D
state and commands rather than managed integration.

`tests/Ymir.Box3D.Parity` builds that facade through CMake and invokes it with
source-generated .NET interop. Run it with:

```powershell
dotnet test tests\Ymir.Box3D.Parity\Ymir.Box3D.Parity.csproj
```

The build requires CMake and a C17 compiler. Box3D is MIT licensed; its license
is retained in the pinned submodule.

## Released v0.1.0 Law

- Sphere overlap accepts separation strictly below the `0.0005 m` overlap
  slop. Exact tangency therefore overlaps.
- Sphere shape casts use Box3D's `0.005 m` linear slop. The hit fraction is not
  the zero-tolerance analytic sphere intersection.
- The low-level v0.1.0 sphere shape cast reports initially touching and
  initially overlapping shapes as hits. The harness records the released
  binary's behavior rather than relying on prose or recollection.
- A two-point shape proxy plus radius is the tractor capsule. Membership
  includes rounded start and end caps and the same overlap slop.
- Box3D query callback order is not a contract. Ymir sorts projected results by
  distance and stable entity id.
- Contact begin facts are emitted after the world step.
- Restitution uses the maximum of the two shape materials.
- Applied torque is transient and cleared by each world step; a continuous
  torque must be submitted again for the next step.

## Cut Line

The managed integrator and circle collision solver are no longer runtime
authorities. The remaining parity slice covers explicit retained-world
ownership, end-contact lifecycle, continuous collision, and replay. Custom
query geometry and spatial indexes are not fallback authorities; Box3D owns
those edge cases.
