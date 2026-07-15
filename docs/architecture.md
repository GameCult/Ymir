# Ymir Architecture

## Objective

Expose Box3D as authoritative, typed, persistent GameCult physics without
leaking an alpha native API into products, clients, or durable state.

Ymir's target is a stable embedded wrapper around retained Box3D worlds, not an
independent physics-engine research project. Box3D supplies the physics mind.
Ymir supplies the memory, nerves, names, and session boundary. The same
boundary may optionally be hosted as a standalone daemon.

## Target Owner

A long-lived Box3D `b3World` owns the physical decision: given the current
world plus commands for a fixed step, what transforms, velocities, overlaps,
casts, and contact transitions are true.

Ymir owns the enclosing session and contract:

- pin and build the accepted Box3D release;
- map stable GameCult ids to transient Box3D handles;
- accept typed body, shape, filter, force, torque, field, query, and step
  commands;
- project Box3D outputs into typed, deterministically ordered Ymir facts;
- reconstruct sessions from CultCache checkpoints;
- return typed state, contacts, queries, and receipts to an embedding product;
- when running standalone, publish Ymir's own physics provider and operator
  surfaces through CultMesh;
- supervise world lifetime, failure, replay, and observability.

## Deployment Boundary

For Aetheria, Ymir is an embedded library. Aetheria owns the outer daemon,
gameplay session, fixed tick, product commands and receipts, asset catalog, and
advertised Eve/CultMesh surface. Generic Eve clients connect directly to the
Aetheria daemon. They never import Ymir internals and do not route gameplay
through a standalone Ymir or discovery service.

A standalone Ymir daemon is an optional hosting mode for physics tooling and
other products. It hosts the same Ymir session primitive; it is not an
Aetheria relay and cannot own a shadow copy of Aetheria's world.

## Current Body

The runtime solver cut is complete: Box3D is the only integrator, collision
engine, broadphase, query-geometry implementation, and contact source in the
committed Ymir path. The public `YmirSimulator.Step` compatibility API creates
an isolated internal session for one complete snapshot, projects its output,
and disposes it. Public circle overlap and cast APIs invoke Box3D geometry over
caller-supplied bodies.

The retained native session now has a public in-process `YmirSession` port with
explicit spawn, remove, teleport, velocity, configuration, force, torque,
step, and disposal operations. It emits revisioned receipts and typed
Begin/Hit/End facts with stable Ymir identities. Ordinary retained steps do not
accept complete body snapshots.

The current CLI still has no named session registry, CultMesh command
lowerings, durable receipt ledger, or reconstruction
checkpoint. Its CultCache and Eve output is a regenerated diagnostic
projection. The sections marked as target contract describe that daemon cut;
they are not advertisements of current CLI capability.

## Target Inputs

- stable body and shape definitions
- fixed time step and explicit substep count
- transient forces, torques, impulses, and target transforms
- collision filters and material definitions
- overlap, ray, and shape-cast queries
- product-owned selections such as which pickup ids a tractor may affect

Aetheria chooses physical intent and gameplay eligibility. Box3D decides
geometry membership and contact. Ymir carries the typed handoff.

## Target Outputs

- body transforms and linear/angular velocities
- begin, end, and hit contact facts with stable Ymir ids
- overlap and cast results sorted by distance and stable id
- step receipts and replay/checkpoint provenance
- provider-owned operator and debug projections

Box3D callback order is not a public contract. Ymir may sort facts, but it may
not alter their membership or invent missing contacts.

## Derived State

- Box3D world, body, shape, contact, and joint handles are transient session
  state derived from stable Ymir definitions.
- CultCache checkpoints are reconstruction inputs and publication records; they
  are not a second live solver.
- SoA body documents are transport and inspection projections over the live
  session.
- Eve surfaces are presentation projections over Ymir state and receipts.
- Unity GameObjects and Electron objects are render state only.
- Aetheria damage, loot, capacity, faction, and scoring consequences are
  derived from Ymir facts plus Aetheria policy.

## Dimensional Compatibility

The fossil Aetheria simulation uses a two-dimensional gameplay plane. During
cutover, Ymir maps that plane into Box3D X/Z with translation on Y locked and
rotation constrained to the plane normal. This is an explicit compatibility
projection. It does not make Ymir a second 2D physics engine.

Sphere friction and inertia are not identical to planar disk behavior. Any
mechanic that depends on that difference must be witnessed against the fossil
and then expressed through Box3D configuration, not repaired by a parallel
solver.

## Forbidden Writers

- managed integration, impulse resolution, collision geometry, or broadphase
  code in Ymir;
- persisted force or torque replayed as if it were continuing authority;
- Aetheria rectangle, capsule, overlap, or contact tests deciding gameplay
  physics;
- Unity collision callbacks deciding hits, destruction, or pickup collection;
- renderer transforms pushing hidden corrections into the world;
- transport endpoints or dashboards owning world state;
- Box3D opaque ids stored as durable GameCult identity.

## Target Shared Paths

Direct player commands, agent commands, programmatic spawns, server ticks,
imports, reconnects, replay, and checkpoint recovery will enter the same Ymir
session primitive for their named world. Today, DTO compatibility methods
lower into an isolated Box3D session; they may not run an alternate step or
infer retained identity from a body list.

Session identity must be explicit. A process-wide simulator object, zone index,
or repeated stable body id is not enough to join two calls to the same world.
Aetheria owns run and zone lifetime and creates, retains, and disposes the
corresponding embedded Ymir session. Projectile sweep
and stationary mine proximity integration can use Ymir's revision-checked
retained-session circle-cast and overlap queries. They observe the current
stable-id body projection; they do not invent a contact episode or mutate the
Box3D world.

Radial gravity and tractor attraction are typed force-generation inputs. They
are applied to Box3D before stepping. Pickup collection remains an Aetheria
transaction triggered exactly once by a Ymir/Box3D contact fact.

## Native Boundary

`extern/box3d` pins the accepted upstream source. `native/Ymir.Box3D` owns a
small stable C ABI facade. Managed Ymir code binds only to that facade; it does
not scatter hundreds of direct P/Invokes across the daemon.

The native facade must remain boring:

- caller-owned blittable buffers;
- explicit version and precision facts;
- no GameCult gameplay policy;
- no JSON state boundary;
- no public exposure of Box3D internal handles;
- first-class RID packaging before production consumers depend on it.

## Target Persistence Contract

Canonical deterministic replay starts from the same initial definitions and
creation order, then applies the complete command stream under the pinned
Box3D build and accepted runtime configuration.

A checkpoint restart is a different promise. Its schema must contain enough
typed definitions and state to reconstruct an equivalent Box3D world: stable
ids, 3D transforms, linear and angular velocity,
body/shape/material/filter definitions, session tuning, and provenance. Box3D
does not expose every internal solver cache, so arbitrary mid-world checkpoint
restart is tested against named numeric state tolerances rather than claimed as
bit-identical continuation. Contact membership, ordering, and exactly-once
lifecycle identity are binary contracts, not tolerance checks. Authoritative
contact facts resume only after canonical history has been replayed through the
checkpoint tick; a checkpoint alone cannot emit gameplay contact facts.

`gamecult.ymir.world_state.v2` appends Box3D body type, bullet,
field-participation, and collision filter arrays to the v1 layout. The v1 and
v0 readers are explicit migration paths; keys are not reassigned. Torque is
transient command state and is intentionally absent. Version 2 is a
compatibility world snapshot, not a complete retained-session
checkpoint: it does not preserve full shape, material, filter, session tuning,
provenance, or warm contact state.

Transient forces and torques are commands, not durable body state. Derived
mass and inertia are published only when useful for inspection. Old checkpoint
versions must migrate explicitly or fail closed; they may not silently decode
under a reassigned MessagePack layout.

## Cut Line

The managed integrator and circle impulse solver have been deleted from the
runtime path. Any uncommitted `YmirQueryPrimitives` or `YmirSpatialIndex` work
is obsolete and must not be annexed. The retained-session cut continues until
Box3D-backed sessions prove:

1. fixed-step body state projection;
2. begin/end/hit contact lifecycle;
3. overlap and shape-cast lowering;
4. continuous collision event timing and same-step Box3D cast witnesses;
5. transient force and torque application;
6. checkpoint reconstruction and replay;
7. Aetheria gravity, tractor, pickup, and payload smokes.

There is no managed fallback after that cut. A native load or compatibility
failure fails closed and reports the missing capability.

## Required Verification

- Box3D parity tests record released upstream law at the native boundary.
- Ymir differential tests compare public projections with the same Box3D
  session facts.
- Negative source checks prove custom solver and spatial-index authorities are
  absent after cutover.
- Aetheria smokes must prove gravity, contact-gated destruction loot, tractor
  attraction, pickup collection, and fast payload hits through the retained
  contract. Current gravity, pickup-contact, cast, and overlap witnesses cover
  only the snapshot/query compatibility path.
- Released-client witnesses prove generic Eve clients only observe Ymir and
  Aetheria provider contracts, never native implementation details.
