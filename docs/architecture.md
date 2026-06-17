# Ymir Architecture

## Objective

Own physics for Aetheria outside Unity so projectiles, fields, bodies, and
contacts share one simulation truth across game runtime, server logic, tools,
and future Verse surfaces.

## Current Mechanism

The MVP has one pure core and one daemon:

- `Ymir.Core` receives a `YmirWorld` and a fixed `deltaTime`.
- The core computes radial field acceleration, integrates dynamic bodies, tests
  circle overlaps, resolves simple collision response, and emits contact events.
- `ymir-daemon` publishes provider/operator state and exposes the core through a
  small HTTP compatibility surface.

## Owner

Ymir owns the decision: given world state plus a time step, what is the next
physics state and which contacts occurred.

## Inputs

- body id, position, velocity, radius, mass, restitution, static flag
- radial field id, position, strength, radius
- fixed `deltaTime`

## Outputs

- next world time
- next body positions and velocities
- contact events with body ids, point, normal, penetration, and relative speed

## Derived State

- HTTP health, provider advertisement, operator state, and Eve surface are
  derived compatibility views.
- Unity GameObjects are derived render/adapter state.
- Damage events in Aetheria should become derived from Ymir contacts plus
  Aetheria gameplay policy.

## Forbidden Writers

- Unity `OnCollisionEnter` and collider callbacks must stop deciding projectile
  hit truth once a cutover path exists.
- Renderer transforms must not push hidden physics corrections back into Ymir.
- HTTP compatibility witnesses must not become durable state owners.

## Shared Paths

Direct user actions, weapon firing, programmatic spawns, reloads, server ticks,
and replay/import paths should all create or modify Ymir world state through the
same command/step primitive. Manual Unity interaction may author inputs; it does
not become a second simulation.

## Cut Line

The old Aetheria projectile GameObject/collider path is demoted first to a
renderer and diagnostic witness. New damage or hit policy must consume Ymir
contact events. Only after that path is proven should Unity collider callbacks
be deleted.

## Verification Layer

- Unit tests prove core integration, gravity field acceleration, deterministic
  ordering, and contact generation.
- CLI smoke proves the daemon can execute a step without booting Unity.
- Aetheria cutover needs a Unity-side timeline probe that compares visible
  projectile positions and hit events against Ymir snapshots during travel, not
  only after impact.
