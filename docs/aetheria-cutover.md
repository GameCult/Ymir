# Aetheria Cutover Notes

## First Slice

Cut over projectile travel and impact discovery before trying to replace every
Unity physics use.

1. Spawn a Ymir body for each projectile and relevant target hull/contact shape.
2. Step Ymir on the same fixed tick used by Aetheria gameplay.
3. Render Unity projectile GameObjects from Ymir snapshots.
4. Convert Ymir contacts into Aetheria hit/splash policy.
5. Keep Unity colliders as visual/debug witnesses only during the transition.
6. Delete or neuter collider callbacks once the Ymir path owns hit truth.

## Negative Checks

- A projectile with a Unity collider disabled still travels and hits through
  Ymir.
- A Unity `OnCollisionEnter` callback cannot apply damage independently.
- Reloading or replaying the same Ymir world produces the same contact order.
- Mid-flight visible projectile position matches the Ymir snapshot within a
  named tolerance.

## Adapter Boundary

Unity adapter code may:

- submit spawn/despawn/field/body commands
- receive snapshots and contacts
- render GameObjects from snapshots
- publish diagnostics

Unity adapter code may not:

- alter positions as a hidden repair loop
- apply gameplay damage from Unity collisions
- maintain a parallel physics cache that can override Ymir
- treat scene hierarchy as the source of simulation truth
