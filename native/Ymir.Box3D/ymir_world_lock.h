// Process-wide lock around Box3D world creation and destruction. b3CreateWorld and b3DestroyWorld scan
// and rewrite the global world pool (b3_worlds) with no synchronization, so two threads creating worlds
// at once can claim the same slot and corrupt it. Stepping and every other call touch only the caller's
// own world and stay unlocked. A single world is still not thread-safe. One lock, defined in
// ymir_box3d.c, serves every translation unit.
#pragma once

#include <box3d/box3d.h>
#include <box3d/types.h>

b3WorldId ymir_create_world_locked(const b3WorldDef* def);
void ymir_destroy_world_locked(b3WorldId world_id);
