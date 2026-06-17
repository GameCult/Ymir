# Ymir

Ymir is the GameCult physics/world substrate engine for Aetheria and future
world-bearing projects.

Unity made the first playable body move. It does not get to keep physics truth.
Ymir owns simulation state: bodies, fields, projectiles, collision, contact
events, integration, and deterministic queries. Renderers and engines lower
Ymir state into visible worlds; they do not become the world.

## Authority

- Ymir owns physics simulation truth, step semantics, contact generation,
  collision queries, and physics debug/operator state.
- CultMath owns the numeric substrate Ymir uses for vector math and shared
  shader-shaped primitives.
- Aetheria owns gameplay policy: damage, faction meaning, weapon selection,
  ship fitting, and player-visible consequence.
- Unity owns rendering, authoring affordances, GameObject presentation, VFX,
  audio, and temporary adapter code during cutover.
- Brokkr may expose Unity editor state to the Verse, but it does not own Ymir
  simulation truth.
- Odin discovers Ymir as a Verse provider; Odin does not own Ymir state.
- CultCache is the target durable state substrate. Current JSON surfaces are
  compatibility witnesses for Aetheria cutover and HTTP inspection.

## First Body

`Ymir.Core` targets `net8.0` to stay compatible with CultMath and likely
Aetheria integration paths. The daemon and tests target `net10.0` because this
machine currently has the .NET 10 runtime installed.

The MVP is deliberately small:

- circle bodies in a 2D physics plane
- radial fields for Aetheria-style gravity wells
- semi-implicit Euler integration
- circle collision detection and simple impulse response
- deterministic contact events
- CLI smoke commands
- local HTTP daemon with `/simulate/step`

This is not pretending to be a finished physics engine. It is the first clean
owner, and therefore already better than collider callbacks wearing a crown.

## Smoke

```powershell
dotnet test
dotnet run --project src\ymir-daemon -- step-sample
dotnet run --project src\ymir-daemon -- provider
dotnet run --project src\ymir-daemon -- serve --port 8877
```

HTTP endpoints:

- `GET /health`
- `GET /provider-advertisement`
- `GET /operator-state`
- `GET /eve/operator`
- `POST /simulate/step`

`POST /simulate/step` accepts the same shape emitted by `step-sample`.

## Cutover Rule

Aetheria should first route projectile travel and hit discovery through Ymir
while keeping Unity as renderer/presenter. The old Unity collider path becomes a
visual witness and adapter path only. If manual collision callbacks can still
decide gameplay damage after the cutover, the cutover is not done.
