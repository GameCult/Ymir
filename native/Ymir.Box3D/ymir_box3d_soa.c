// Headless 3D Box3D world with a structure-of-arrays state view. Box3D owns every physical
// decision (integration, collision, contact); this file only creates bodies, steps the world and
// copies each body's state into contiguous lanes once per step. A caller reads the lanes in place
// through `ymir_box3d_soa_view`: the pointers are stable for the world's whole life (capacity is
// fixed at creation) and the lane contents change only inside `ymir_box3d_soa_step`.
//
// This world is not the retained planar session in ymir_box3d.c: that one locks Y, zeroes gravity
// and hands out an array of structs by copy. See docs/soa-world.md.

#include <box3d/base.h>
#include <box3d/box3d.h>
#include <box3d/collision.h>
#include <box3d/math_functions.h>
#include <box3d/types.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#define YMIR_EXPORT __declspec(dllexport)
#define YMIR_CALL __cdecl
#else
#define YMIR_EXPORT __attribute__((visibility("default")))
#define YMIR_CALL
#endif

enum
{
    YMIR_SOA_OK = 0,
    YMIR_SOA_INVALID_ARGUMENT = 1,
    YMIR_SOA_DUPLICATE_ID = 2,
    YMIR_SOA_FULL = 3,
    YMIR_SOA_OUT_OF_MEMORY = 4,
    YMIR_SOA_INTERNAL_ERROR = 5
};

enum
{
    YMIR_SOA_KIND_SPHERE = 0,
    YMIR_SOA_KIND_STATIC_BOX = 1
};

#define YMIR_SOA_ABI_VERSION 1u

// Pointers first and in lane order, so the layout is the same on every 64-bit target.
typedef struct ymir_box3d_soa_view
{
    uint32_t abi_version;
    int32_t count;
    int32_t capacity;
    uint32_t reserved;
    uint64_t step_count;
    const uint64_t* stable_id;
    const uint32_t* kind;
    const float* position_x;
    const float* position_y;
    const float* position_z;
    const float* rotation_x;
    const float* rotation_y;
    const float* rotation_z;
    const float* rotation_w;
    const float* velocity_x;
    const float* velocity_y;
    const float* velocity_z;
    const float* angular_velocity_x;
    const float* angular_velocity_y;
    const float* angular_velocity_z;
} ymir_box3d_soa_view;

#define YMIR_SOA_FLOAT_LANES 13

typedef struct ymir_box3d_soa_world
{
    b3WorldId world_id;
    int32_t count;
    int32_t capacity;
    uint64_t step_count;
    b3BodyId* bodies;
    uint64_t* stable_id;
    uint32_t* kind;
    float* lanes; // YMIR_SOA_FLOAT_LANES arrays of `capacity` floats, lane-major
} ymir_box3d_soa_world;

static float* ymir_soa_lane(const ymir_box3d_soa_world* world, int lane)
{
    return world->lanes + (size_t)lane * (size_t)world->capacity;
}

static int ymir_soa_finite(float v)
{
    return isfinite(v);
}

static void ymir_soa_refresh(ymir_box3d_soa_world* world, int index)
{
    b3BodyId body = world->bodies[index];
    b3Pos p = b3Body_GetPosition(body);
    b3Quat q = b3Body_GetRotation(body);
    b3Vec3 v = b3Body_GetLinearVelocity(body);
    b3Vec3 w = b3Body_GetAngularVelocity(body);
    const float values[YMIR_SOA_FLOAT_LANES] = {
        p.x, p.y, p.z, q.v.x, q.v.y, q.v.z, q.s, v.x, v.y, v.z, w.x, w.y, w.z};
    for (int lane = 0; lane < YMIR_SOA_FLOAT_LANES; ++lane)
    {
        ymir_soa_lane(world, lane)[index] = values[lane];
    }
}

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_abi_version(void)
{
    return YMIR_SOA_ABI_VERSION;
}

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_view_size(void)
{
    return (uint32_t)sizeof(ymir_box3d_soa_view);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_create(
    int32_t capacity, float gravity_x, float gravity_y, float gravity_z, ymir_box3d_soa_world** output)
{
    if (output == NULL)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    *output = NULL;
    if (capacity <= 0 || !ymir_soa_finite(gravity_x) || !ymir_soa_finite(gravity_y) || !ymir_soa_finite(gravity_z))
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }

    ymir_box3d_soa_world* world = (ymir_box3d_soa_world*)calloc(1, sizeof(ymir_box3d_soa_world));
    if (world == NULL)
    {
        return YMIR_SOA_OUT_OF_MEMORY;
    }
    world->capacity = capacity;
    world->bodies = (b3BodyId*)calloc((size_t)capacity, sizeof(b3BodyId));
    world->stable_id = (uint64_t*)calloc((size_t)capacity, sizeof(uint64_t));
    world->kind = (uint32_t*)calloc((size_t)capacity, sizeof(uint32_t));
    world->lanes = (float*)calloc((size_t)capacity * YMIR_SOA_FLOAT_LANES, sizeof(float));
    if (world->bodies == NULL || world->stable_id == NULL || world->kind == NULL || world->lanes == NULL)
    {
        free(world->bodies);
        free(world->stable_id);
        free(world->kind);
        free(world->lanes);
        free(world);
        return YMIR_SOA_OUT_OF_MEMORY;
    }

    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = (b3Vec3){gravity_x, gravity_y, gravity_z};
    def.enableSleep = false;
    def.workerCount = 1; // one worker: the step order is fixed, which determinism rests on
    world->world_id = b3CreateWorld(&def);
    if (B3_IS_NULL(world->world_id))
    {
        free(world->bodies);
        free(world->stable_id);
        free(world->kind);
        free(world->lanes);
        free(world);
        return YMIR_SOA_INTERNAL_ERROR;
    }

    *output = world;
    return YMIR_SOA_OK;
}

YMIR_EXPORT void YMIR_CALL ymir_box3d_soa_destroy(ymir_box3d_soa_world* world)
{
    if (world == NULL)
    {
        return;
    }
    if (B3_IS_NON_NULL(world->world_id) && b3World_IsValid(world->world_id))
    {
        b3DestroyWorld(world->world_id);
    }
    free(world->bodies);
    free(world->stable_id);
    free(world->kind);
    free(world->lanes);
    free(world);
}

static int32_t ymir_soa_reserve(ymir_box3d_soa_world* world, uint64_t stable_id)
{
    if (world->count >= world->capacity)
    {
        return YMIR_SOA_FULL;
    }
    for (int i = 0; i < world->count; ++i)
    {
        if (world->stable_id[i] == stable_id)
        {
            return YMIR_SOA_DUPLICATE_ID;
        }
    }
    return YMIR_SOA_OK;
}

static int32_t ymir_soa_commit(
    ymir_box3d_soa_world* world, uint64_t stable_id, uint32_t kind, b3BodyId body, int32_t* index)
{
    int i = world->count++;
    world->bodies[i] = body;
    world->stable_id[i] = stable_id;
    world->kind[i] = kind;
    ymir_soa_refresh(world, i);
    if (index != NULL)
    {
        *index = i;
    }
    return YMIR_SOA_OK;
}

// A dynamic sphere. `density` is Box3D's shape density; mass follows from it.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_add_sphere(
    ymir_box3d_soa_world* world, uint64_t stable_id, float position_x, float position_y, float position_z,
    float velocity_x, float velocity_y, float velocity_z, float radius, float density, float friction,
    float restitution, int32_t* index)
{
    if (world == NULL || radius <= 0.0f || density <= 0.0f || !ymir_soa_finite(position_x) ||
        !ymir_soa_finite(position_y) || !ymir_soa_finite(position_z) || !ymir_soa_finite(velocity_x) ||
        !ymir_soa_finite(velocity_y) || !ymir_soa_finite(velocity_z) || !ymir_soa_finite(radius) ||
        !ymir_soa_finite(density) || !ymir_soa_finite(friction) || !ymir_soa_finite(restitution))
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    int32_t status = ymir_soa_reserve(world, stable_id);
    if (status != YMIR_SOA_OK)
    {
        return status;
    }

    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = b3_dynamicBody;
    body_def.position = (b3Vec3){position_x, position_y, position_z};
    body_def.linearVelocity = (b3Vec3){velocity_x, velocity_y, velocity_z};
    body_def.enableSleep = false;
    b3BodyId body = b3CreateBody(world->world_id, &body_def);
    if (B3_IS_NULL(body))
    {
        return YMIR_SOA_INTERNAL_ERROR;
    }
    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.userData = (void*)(uintptr_t)stable_id;
    shape_def.density = density;
    shape_def.baseMaterial.friction = friction;
    shape_def.baseMaterial.restitution = restitution;
    b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, radius};
    if (B3_IS_NULL(b3CreateSphereShape(body, &shape_def, &sphere)))
    {
        b3DestroyBody(body);
        return YMIR_SOA_INTERNAL_ERROR;
    }
    return ymir_soa_commit(world, stable_id, YMIR_SOA_KIND_SPHERE, body, index);
}

// A static box (ground, walls) centred at the position, half extents along the world axes.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_add_static_box(
    ymir_box3d_soa_world* world, uint64_t stable_id, float position_x, float position_y, float position_z,
    float half_x, float half_y, float half_z, float friction, float restitution, int32_t* index)
{
    if (world == NULL || half_x <= 0.0f || half_y <= 0.0f || half_z <= 0.0f || !ymir_soa_finite(position_x) ||
        !ymir_soa_finite(position_y) || !ymir_soa_finite(position_z) || !ymir_soa_finite(half_x) ||
        !ymir_soa_finite(half_y) || !ymir_soa_finite(half_z) || !ymir_soa_finite(friction) ||
        !ymir_soa_finite(restitution))
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    int32_t status = ymir_soa_reserve(world, stable_id);
    if (status != YMIR_SOA_OK)
    {
        return status;
    }

    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = b3_staticBody;
    body_def.position = (b3Vec3){position_x, position_y, position_z};
    b3BodyId body = b3CreateBody(world->world_id, &body_def);
    if (B3_IS_NULL(body))
    {
        return YMIR_SOA_INTERNAL_ERROR;
    }
    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.userData = (void*)(uintptr_t)stable_id;
    shape_def.baseMaterial.friction = friction;
    shape_def.baseMaterial.restitution = restitution;
    b3BoxHull box = b3MakeBoxHull(half_x, half_y, half_z);
    if (B3_IS_NULL(b3CreateHullShape(body, &shape_def, &box.base)))
    {
        b3DestroyBody(body);
        return YMIR_SOA_INTERNAL_ERROR;
    }
    return ymir_soa_commit(world, stable_id, YMIR_SOA_KIND_STATIC_BOX, body, index);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_step(ymir_box3d_soa_world* world, float time_step, int32_t sub_step_count)
{
    if (world == NULL || !ymir_soa_finite(time_step) || time_step <= 0.0f || sub_step_count <= 0)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    b3World_Step(world->world_id, time_step, sub_step_count);
    for (int i = 0; i < world->count; ++i)
    {
        ymir_soa_refresh(world, i);
    }
    world->step_count += 1;
    return YMIR_SOA_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_get_view(const ymir_box3d_soa_world* world, ymir_box3d_soa_view* view)
{
    if (world == NULL || view == NULL)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    const float* base = world->lanes;
    size_t n = (size_t)world->capacity;
    *view = (ymir_box3d_soa_view){
        .abi_version = YMIR_SOA_ABI_VERSION,
        .count = world->count,
        .capacity = world->capacity,
        .step_count = world->step_count,
        .stable_id = world->stable_id,
        .kind = world->kind,
        .position_x = base + 0 * n,
        .position_y = base + 1 * n,
        .position_z = base + 2 * n,
        .rotation_x = base + 3 * n,
        .rotation_y = base + 4 * n,
        .rotation_z = base + 5 * n,
        .rotation_w = base + 6 * n,
        .velocity_x = base + 7 * n,
        .velocity_y = base + 8 * n,
        .velocity_z = base + 9 * n,
        .angular_velocity_x = base + 10 * n,
        .angular_velocity_y = base + 11 * n,
        .angular_velocity_z = base + 12 * n,
    };
    return YMIR_SOA_OK;
}
