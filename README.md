# Ymir

Ymir is the GameCult Box3D wrapper and authoritative physics daemon for
Aetheria and future world-bearing projects. The name means **Not Invented
Here**. We are taking the hint.

Box3D owns physics algorithms. Ymir owns the service boundary that makes those
algorithms usable as durable, typed GameCult world truth. Unity, renderers, and
game products consume Ymir state and contact facts; they do not become physics
authorities.

## Ownership

- Box3D owns rigid-body integration, collision geometry, broadphase, solver
  behavior, continuous collision, overlap and cast semantics, contact
  lifecycle, force and torque lifetime, tolerances, and deterministic step
  behavior for identical recorded inputs.
- Ymir owns the pinned Box3D version, stable GameCult entity ids, native ABI
  isolation, long-lived world sessions, typed commands and facts,
  deterministic result ordering, checkpoint reconstruction, CultCache state,
  CultMesh publication, and daemon lifecycle.
- Aetheria owns gameplay policy and supplies typed physical intent: bodies,
  fields, tractor targets, forces, projectiles, filters, and the gameplay
  consequences of Ymir contact facts.
- CultMath remains available for GameCult-side numeric projections and force
  preparation. It is not a second collision or integration engine.
- Unity and Eve lowerers render world state and submit commands. They never
  repair, override, or independently simulate authoritative physics.

Box3D opaque handles are process-local derived state. They are never durable
entity ids and never cross the public contract boundary.

## Current Cutover State

Box3D v0.1.0 is pinned at commit
`8441b4a06d6d09dcfb0b0f704df4d847d1437b92`. A Ymir-owned native shim and
parity suite already witness released Box3D behavior for:

- sphere overlap and shape casts
- tractor capsule membership and rounded caps
- numeric overlap and cast slop
- planar sphere-world stepping
- contact-begin facts
- restitution mixing
- transient torque lifetime

The legacy managed Ymir solver remains temporarily live while retained Box3D
world sessions, end-contact lifecycle, continuous collision, replay, and
checkpoint reconstruction are connected. It is compatibility scaffolding, not
the target architecture. It will be deleted, along with Ymir's custom geometry
and spatial indexes, once the Box3D-backed path proves the public contract.

See [the architecture map](docs/architecture.md) and
[the executable parity contract](docs/box3d-parity.md).

## Build And Test

Initialize the pinned dependency and run the solution:

```powershell
git submodule update --init --recursive
dotnet test Ymir.slnx
```

The parity project builds the native shim from source and requires CMake plus a
C17 compiler:

```powershell
dotnet test tests\Ymir.Box3D.Parity\Ymir.Box3D.Parity.csproj
```

Box3D is MIT licensed. Its license is retained in `extern/box3d`.

## Cutover Rule

A Ymir cutover is complete only when all runtime stepping, collision queries,
and contact facts come from a retained Box3D world through one Ymir session
primitive. Manual Unity callbacks, Aetheria geometry tests, and the managed
Ymir solver must be structurally unable to decide the result.
