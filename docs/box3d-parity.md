# Box3D Parity Harness

## Authority

Box3D v0.1.0 is Ymir's physics implementation and executable semantics oracle.
The production session cutover is still under construction. The exact upstream source is pinned as the
`extern/box3d` submodule at commit
`8441b4a06d6d09dcfb0b0f704df4d847d1437b92`.

Box3D does not own GameCult entity ids, gameplay policy, persistence, or result
ordering. Ymir projects Box3D facts into those contracts. Box3D owns
shape overlap, shape cast, solver, broadphase, contact lifecycle, force and
torque lifetime, and numeric tolerances.

## Body

`native/Ymir.Box3D` statically links the pinned C17 library into one small
shared-library facade. The facade exports only blittable sphere overlap,
sphere cast, capsule-versus-sphere membership, planar pair stepping, torque
lifetime, and version operations. This keeps Box3D's alpha C ABI out of Ymir's
public managed contracts.

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

The current managed collision formulas are transitional. They may exist only
while differential tests prove compatibility. The remaining parity slice
covers retained world sessions, end-contact lifecycle, continuous collision,
and replay. Once those facts are projected through a stable Ymir native port,
the managed solver, query geometry, and spatial indexes are deleted rather
than retained as fallback authority.
