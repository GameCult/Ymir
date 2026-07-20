# Retained Session Contract

## Objective

One named Ymir session owns one retained Box3D world. A product may restore the
world once, issue explicit mutations, step it, query it, and consume typed
Box3D contact facts. A body snapshot is output and checkpoint material; it is
not an implicit command language.

## Current Mechanism

`YmirSimulator` is an isolated snapshot compatibility API. It creates a native
session, synchronizes a complete body list, steps once, projects the result,
and disposes the session. Its private compatibility helper uses that complete
list, so omission can delete a body and stale input can teleport or overwrite
solver velocity inside the isolated call. Its legacy contact output merges
current manifolds, begin events, and hit events into one pair-deduplicated list
and omits End.

The public `YmirSession` does not use that mechanism for ordinary retained
steps. The compatibility path is not the retained session contract.

## Owner

- One public Ymir session serializes commands for one named world.
- Its native Box3D `b3World` exclusively owns integration, collision,
  broadphase, queries, solver state, and contact lifecycle.
- Ymir owns stable GameCult ids, body generations, session revision, step
  sequence, deterministic projection, fact identity, and command receipts.
- The product owns session lifetime and gameplay policy. Aetheria maps one
  `(RunId, ZoneIndex)` lifetime to one Ymir session.

## Target Inputs

The retained port accepts narrow operations with explicit meaning:

- spawn a body, rejecting an existing id;
- remove a body, rejecting an unknown id;
- teleport a body;
- set linear and angular velocity;
- configure body shape, mass, static/kinematic/dynamic type, and restitution;
- apply one-shot force or torque;
- step with a fixed delta, substep count, and typed fields;
- dispose the session.

The current public slice implements those mutation and step operations plus
revision-checked circle overlap and cast queries. A query observes the current
retained body projection under the session lock and accepts an explicit set of
stable candidate body ids. The candidate set is product-owned eligibility;
Box3D still owns the overlap/cast geometry and numeric result.
Bullet, collision-filter, and field-participation profiles are currently
spawn/restore definitions. Retained `Configure` changes radius, mass,
static/kinematic/dynamic type, and restitution only. Changing the profile
requires explicit remove-and-spawn until a typed profile command exists.

Restore/import is the only path allowed to accept a complete world snapshot.
There is no `Upsert`, generic patch bag, or omission-based removal.

## Outputs

Each accepted mutation produces a receipt that names the session generation,
command id, before revision, and after revision. A step additionally returns:

- authoritative Box3D body state;
- the new step index;
- typed `Begin`, `Hit`, and `End` facts;
- stable fact ids and contact-episode ids;
- the accepted step configuration through its typed command.

The current library validates the native Box3D version and ABI before session
creation but does not repeat those facts in every Step result.

Facts canonicalize bodies by stable id and flip normals when canonicalization
swaps endpoints. Their public order is body pair, contact episode, kind, then
within-kind ordinal. This is stable publication order, not invented substep
chronology.

## Derived State

- Box3D handles are process-local and never cross the contract.
- DTO and SoA body arrays are projections.
- `gamecult.ymir.world_state.v2` is an inspection/migration snapshot, not a
  retained-session checkpoint.
- Receipt and contact-episode ledgers are session metadata, not physics law.
- Aetheria cargo, damage, loot, feedback, and scoring remain gameplay state
  derived from Ymir facts.

The replay checkpoint preserves the logical session generation, complete typed
command journal, receipt ledger, body-generation counter, world projection, and
active contact episodes. Restore creates a fresh native Box3D world and replays
that journal through the checkpoint boundary before facts may leave the
session. The world and episode records verify replay; they do not replace it or
infer contact continuity from body proximity.

`gamecult.ymir.session_checkpoint.replay.v1` is encoded by the bounded binary
`YmirSessionCheckpointCodec`. The checkpoint carries the pinned Box3D/ABI/
precision fingerprint and fails closed on incompatible provenance, corrupt or
trailing data, journal divergence, world divergence, or contact-lineage
divergence. A SHA-256 checksum binds session identity, provenance, journal,
world verifier, and active episodes. ABI v5 independently validates the exact
pinned native build id before opening a session. It contains no native Box3D
handle or contact key.

For incremental host persistence,
`gamecult.ymir.session_journal_chunk.v1` carries a contiguous command-journal
suffix and `gamecult.ymir.session_resume.v1` carries the bounded reconstruction
boundary. `CapturePersistence(persistedJournalEntryCount)` never repeats an
already acknowledged prefix. Restore orders chunks by their first entry,
requires exact coverage from zero through the descriptor count, and then uses
the same full replay and verifier path as a complete checkpoint. Both binary
forms are independently checksummed.

The cursor is an index within one session generation. Bounded persistence uses
an explicit generation rollover, never an inferred acknowledgement side effect.
`TryCreateCompactedPersistenceBaseline` may create a replacement session only
when the current journal is empty or ends in an accepted Step and Box3D reports
no active contact episodes. The replacement starts from the current body
projection and time with a fresh session generation, revision zero, an empty
receipt ledger, and an empty journal. The host atomically swaps ownership and
then disposes the prior session.

Active contacts, pending forces, or other unstepped mutations refuse rollover.
Those sessions retain replay reconstruction until a later quiescent boundary.
Ymir does not synthesize contact closure, serialize private Box3D handles, or
pretend a body snapshot can preserve solver contact state. Old generation
journals become audit history after the host commits the replacement boundary;
they are not inputs to the new generation's restore.

## Forbidden Writers

- recurring complete-body synchronization after restore;
- snapshot omission as body removal;
- transform or velocity setters hidden inside ordinary Step;
- managed contact lifecycle inference that replaces Box3D events;
- random fact ids or ids derived from manifold floats;
- Box3D callback order exposed as a public ordering guarantee;
- Unity callbacks, renderer state, or gameplay proximity checks deciding
  contact truth.

## Shared Paths

Direct commands, agent commands, spawns, and checkpoint replay use the same
explicit session operations. Restore establishes a fresh native world through
those operations; it is not a parallel state writer.
Retained queries will observe a named revision without mutating it. Snapshot
reads cannot be fed back without an explicit restore, teleport, velocity, or
configuration operation.

## Contact Identity

A contact episode is identified by the canonical pair of body id plus body
generation and a monotonically assigned episode number. Begin and End share
that identity. A later re-contact creates a new episode. Fact identity derives
from session generation, step index, contact episode, kind, and deterministic
ordinal. Box3D contact and shape handles are never durable identity.

Body removal must close any active episode exactly once. Because Box3D End
events may reference destroyed shapes, the native boundary retains stable ids
until the event buffer has been drained.

Box3D publishes Begin, Hit, and End in separate arrays rather than one causal
sequence. The private ABI carries Box3D's stored transient contact key so Ymir
can correlate those arrays before assigning public episode identity. That key
is derived process state: it never appears in a public fact or checkpoint.

## Cut Line

1. Typed native mutation operations and step-owned lifecycle buffers: done.
2. Public managed retained session with no ordinary full synchronization:
   done.
3. Revision-checked retained-session overlap and cast queries: done.
4. One embedded retained world and physical-payload session per Aetheria run
   and zone: done.
5. Incremental journal chunks and bounded resume descriptors: done. Aetheria
   stores them in a daemon-private CultCache; complete history does not ride in
   public product frames. Quiescent generation rollover provides the compaction
   primitive; host adoption and old-generation retention policy are separate.
6. For optional standalone hosting, add a session registry, generation-bearing
   routed commands, idempotent Create, durable receipts, and CultMesh lowering.

The isolated `YmirSimulator` remains a compatibility API. It is never the
retained-world authority.

Mutation-generated End evidence is currently retained in the native pending
buffer and published by the next accepted Step. The host transaction cut
must make mutation plus Step one publication boundary, or return mutation facts
directly, before Ymir promises that removal followed by disposal publishes an
End fact.

## Verification

- A spawned body advances across steps without being resubmitted.
- Omission from unrelated commands cannot remove or reset a body.
- Force and torque affect exactly one accepted step.
- stale revisions and conflicting command ids cannot mutate the world.
- persistent touching emits one Begin, separation emits one End, and re-contact
  creates a new episode.
- duplicate fact delivery cannot commit Aetheria cargo twice.
- different insertion orders produce the same semantic fact order.
- queries observe the requested retained revision and cannot inject bodies.
- disposed or stale session generations fail closed.
- restored continuation reproduces command receipts, bodies, active contact
  identity, and subsequent Begin/End facts before authoritative publication.
