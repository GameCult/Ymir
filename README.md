# Ymir

Ymir is the GameCult Box3D wrapper and physics-daemon boundary for Aetheria and
future world-bearing projects. The name means **Not Invented Here**. We are
taking the hint.

Box3D owns physics algorithms. Ymir owns the service boundary that makes those
algorithms usable as durable, typed GameCult world truth. Unity, renderers, and
game products consume Ymir state and contact facts; they do not become physics
authorities.

## Ownership

- Box3D owns rigid-body integration, collision geometry, broadphase, solver
  behavior, continuous collision, overlap and cast semantics, contact
  lifecycle, force and torque lifetime, tolerances, and deterministic step
  behavior for identical recorded inputs.
- Ymir currently owns the pinned Box3D version, stable GameCult entity ids,
  native ABI isolation, public in-process retained sessions, explicit mutation
  receipts, typed contact facts, snapshot lowering, deterministic result
  ordering, versioned CultCache snapshots, and a diagnostic Eve publication.
- The Ymir daemon will own named session routing, durable command receipts,
  checkpoint reconstruction, CultMesh command/fact publication, and process
  supervision. Those are the daemon cutover contract, not a claim that the
  current CLI already exposes them.
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
- category/mask/group collision filtering
- bullet CCD against kinematic bodies, including Box3D's next-step contact-event timing

The production `Ymir.Box3D` boundary owns the native session ABI. The legacy
snapshot `YmirSimulator.Step` contract now creates an isolated Box3D session,
projects the result, and disposes it. It contains no managed integrator,
collision loop, or impulse solver.

That snapshot facade deliberately does not pretend to own long-lived world
identity. The public in-process `YmirSession` now exposes explicit body
mutations, force and torque commands, revisioned receipts, retained stepping,
and typed Begin/Hit/End facts. It never treats omission as removal. The daemon
still lacks a named session registry and CultMesh command lowering. Aetheria
will name one session per run and zone through that daemon contract; its old
process-wide simulator objects are not session identifiers. Retained-world
queries, checkpoint reconstruction, and non-Windows RID artifacts remain
cutover work.

See [the architecture map](docs/architecture.md) and
[the executable parity contract](docs/box3d-parity.md). The
[retained-session contract](docs/retained-session-contract.md) records the
current ownership cut and the public daemon boundary being built.

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

`GameCult.Ymir.Box3D` packages the managed boundary, Box3D license, and native
facade under `runtimes/<rid>/native`. A local Windows x64 package has loaded in
an isolated empty consumer without CMake or the submodule. A committed release
smoke is still required before that is a repeatable packaging guarantee. Linux
and macOS packages must be produced and smoked on their own release workers
before those RIDs are advertised.

The current package version is `0.3.0`. ABI v4 exposes Box3D body type,
bullet, field-participation, and collision-filter configuration without
exposing Box3D handles. These are Ymir contract fields, but their behavior is
Box3D behavior.

## Cutover Rule

A Ymir cutover is complete only when all runtime stepping, collision queries,
and contact facts come from Box3D through an explicitly owned Ymir session.
Manual Unity callbacks and Aetheria geometry tests must be structurally unable
to decide the result. Snapshot compatibility calls may use isolated sessions;
they are not allowed to infer or share retained-world identity.
