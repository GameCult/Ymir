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
steps. The compatibility path is not the retained daemon contract.

## Owner

- One public Ymir session serializes commands for one named world.
- Its native Box3D `b3World` exclusively owns integration, collision,
  broadphase, queries, solver state, and contact lifecycle.
- Ymir owns stable GameCult ids, body generations, session revision, step
  sequence, deterministic projection, fact identity, and command receipts.
- The product owns session lifetime and gameplay policy. Aetheria will map one
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

The current in-process receipt ledger is not the durable daemon ledger. It is
kept for the lifetime of the session and is intentionally not advertised as a
reconnect or checkpoint guarantee.

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

Direct commands, agent commands, and spawns use the same explicit in-process
session operations. Imports, reconnects, and replay must lower through those
operations after the daemon registry and reconstruction contract exist.
Restore will establish initial state; it will not remain a parallel writer.
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
4. Add the daemon session registry, generation-bearing routed commands,
   idempotent Create, bounded live dedupe backed by durable receipts, and
   CultMesh command lowering.
5. Move Aetheria to one session per run and zone, then delete its process-wide
   simulator identity, manual tractor geometry, per-projectile mini-worlds,
   and tuple contact dedup.

The isolated `YmirSimulator` remains a compatibility API. It is never the
daemon authority.

Mutation-generated End evidence is currently retained in the native pending
buffer and published by the next accepted Step. The daemon transaction cut
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
- restored continuation must reproduce bodies and fact identity before Ymir
  advertises retained-session checkpoint support.
