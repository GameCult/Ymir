# Ymir Agent Instructions

Ymir owns physics truth. Treat Unity, renderers, editor tools, and dashboards as
clients or adapters unless a document explicitly transfers authority.

Before substantial implementation, state:

- Objective: what physical invariant or cutover outcome the work serves.
- Current mechanism: how bodies, fields, contacts, and commands flow now.
- Invariants: what must remain true during direct load, step, query, adapter
  playback, and reload.
- Intended change: which owner or data path becomes simpler.
- Cut line: which old callback, cache, adapter, or compatibility path may be
  deleted or demoted.

Architecture constraints:

- Build physics primitives in `Ymir.Core` first. Daemon, Unity, and future Rust
  or CultMesh surfaces are presentation/transport clients.
- Use CultMath for numeric work. Do not copy Unity.Mathematics helpers into
  Ymir.
- JSON is allowed at HTTP/CLI compatibility boundaries. It is not durable truth.
  Durable state should move toward CultCache `.cc` documents and CultMesh
  publication.
- Keep adapters boring. A Unity adapter may submit commands, receive snapshots,
  and render them; it must not silently repair or override Ymir state.
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
