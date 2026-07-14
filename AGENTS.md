# Ymir Agent Instructions

Ymir owns authoritative physics sessions and their GameCult contracts. Box3D
owns the physics algorithms inside those sessions. Treat Unity, products,
renderers, editor tools, and dashboards as clients or adapters unless a
document explicitly transfers authority.

Before substantial implementation, state:

- Objective: what physical invariant or cutover outcome the work serves.
- Current mechanism: how bodies, fields, contacts, and commands flow now.
- Invariants: what must remain true during direct load, step, query, adapter
  playback, and reload.
- Intended change: which owner or data path becomes simpler.
- Cut line: which old callback, cache, adapter, or compatibility path may be
  deleted or demoted.

Architecture constraints:

- Do not reimplement Box3D integration, collision geometry, broadphase,
  queries, solver behavior, contact lifecycle, or numeric tolerances.
  Physics edge cases follow the pinned Box3D release. Executable parity tests
  witness selected released behaviors; they are not a second specification.
- `Ymir.Core` owns typed contracts, stable-id projection, session commands,
  deterministic ordering, persistence reconstruction, and publication. The
  native Ymir wrapper owns Box3D ABI isolation. Daemon, Unity, and future Rust
  or CultMesh surfaces consume those boundaries without importing Box3D
  internals.
- Use CultMath for GameCult-side numeric projections and force preparation. It
  must not become a second integration or collision engine. Do not copy
  Unity.Mathematics helpers into Ymir.
- JSON is allowed at HTTP/CLI compatibility boundaries. It is not durable truth.
  Durable state should move toward CultCache `.cc` documents and CultMesh
  publication.
- Keep adapters boring. A Unity adapter may submit commands, receive snapshots,
  and render them; it must not silently repair or override Ymir state.
- During cutover, provider advertisements and operator state must describe the
  running implementation. Target Box3D/CultMesh capabilities are not live
  capabilities until their production paths exist.
- Aetheria embeds Ymir. Eve clients connect directly to Aetheria's advertised
  CultMesh provider; standalone Ymir and rendezvous services are not in the
  Aetheria gameplay path.
- Add tests at the layer where the invariant lives. State invariants belong in
  `Ymir.Core`; daemon route shape belongs in daemon smokes; Unity visual timing
  bugs need Unity-side probes.

Persona and persistent memory:

- Ymir's repo Face identity lives in `.voidbot/voice/identity.json`.
- Ymir's Persona state path is `.voidbot/state/ymir.cc`.
- Read state with `tools/persona-state-read.ps1`.
- Write durable repo memories with `tools/persona-remember.ps1`; do not hand-edit
  `.cc` state.
- Preserve lessons that steer future ownership, cutover, invariants, and
  failure modes. Do not commemorate every small patch.
