# Ymir Architecture

## Objective

Expose Box3D as authoritative, typed, persistent GameCult physics without
leaking an alpha native API into products, clients, or durable state.

Ymir is a wrapper and daemon, not an independent physics-engine research
project. Box3D supplies the physics mind. Ymir supplies the memory, nerves,
names, and service boundary.

## Owner

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
- publish provider, operator, command, receipt, and Eve surfaces through
  CultMesh;
- supervise world lifetime, failure, replay, and observability.

## Inputs

- stable body and shape definitions
- fixed time step and explicit substep count
- transient forces, torques, impulses, and target transforms
- collision filters and material definitions
- overlap, ray, and shape-cast queries
- product-owned selections such as which pickup ids a tractor may affect

Aetheria chooses physical intent and gameplay eligibility. Box3D decides
geometry membership and contact. Ymir carries the typed handoff.

## Outputs

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

## Shared Paths

Direct player commands, agent commands, programmatic spawns, server ticks,
imports, reconnects, replay, and checkpoint recovery all enter one Ymir world
session primitive. DTO compatibility methods may lower into that primitive;
they may not run an alternate step.

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

## Persistence

The canonical checkpoint schema must contain enough typed definitions and
state to reconstruct the accepted Box3D world: stable ids, 3D transforms,
linear and angular velocity, body/shape/material/filter definitions, session
tuning, and provenance.

Transient forces and torques are commands, not durable body state. Derived
mass and inertia are published only when useful for inspection. Old checkpoint
versions must migrate explicitly or fail closed; they may not silently decode
under a reassigned MessagePack layout.

## Cut Line

The managed Ymir solver, `YmirQueryPrimitives`, and `YmirSpatialIndex` are
deleted when retained Box3D sessions prove:

1. fixed-step body state projection;
2. begin/end/hit contact lifecycle;
3. overlap and shape-cast lowering;
4. continuous collision and projectile witnesses;
5. transient force and torque application;
6. checkpoint reconstruction and replay;
7. Aetheria gravity, tractor, pickup, and payload smokes.

There is no managed fallback after that cut. A native load or compatibility
failure fails closed and reports the missing capability.

## Verification

- Box3D parity tests record released upstream law at the native boundary.
- Ymir differential tests compare public projections with the same Box3D
  session facts.
- Negative source checks prove custom solver and spatial-index authorities are
  absent after cutover.
- Aetheria smokes prove gravity, contact-gated destruction loot, tractor
  attraction, pickup collection, and fast payload hits.
- Released-client witnesses prove generic Eve clients only observe Ymir and
  Aetheria provider contracts, never native implementation details.
