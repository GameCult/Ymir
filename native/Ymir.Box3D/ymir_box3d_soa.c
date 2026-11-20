// Headless 3D Box3D world with a structure-of-arrays state view. Box3D owns every physical
// decision (integration, collision, contact, joints); this file creates bodies and joints, applies
// the caller's force and torque lanes, steps the world and copies each body's and joint's state into
// contiguous lanes once per step. A caller reads the state lanes and writes the force/torque lanes in
// place through `ymir_box3d_soa_get_view`: the pointers are stable for the world's whole life
// (capacities are fixed at creation) and the state lanes change only inside `ymir_box3d_soa_step`.
//
// This world is not the retained planar session in ymir_box3d.c: that one locks Y, zeroes gravity
// and hands out an array of structs by copy. See docs/soa-world.md.

#include <box3d/base.h>
#include <box3d/box3d.h>
#include <box3d/collision.h>
#include <box3d/math_functions.h>
#include <box3d/types.h>

#include "ymir_world_lock.h"

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
    YMIR_SOA_KIND_STATIC_BOX = 1,
    YMIR_SOA_KIND_CAPSULE = 2,
    YMIR_SOA_KIND_BOX = 3,
    YMIR_SOA_KIND_ANCHOR = 4
};

enum
{
    YMIR_SOA_JOINT_REVOLUTE = 0,
    YMIR_SOA_JOINT_SPHERICAL = 1,
    YMIR_SOA_JOINT_WELD = 2,
    YMIR_SOA_JOINT_MOTOR = 3,
    YMIR_SOA_JOINT_REMOVED = 4 // a destroyed joint's lane: never accepted by add_joint
};

#define YMIR_SOA_ABI_VERSION 8u

// A body to create. `size` by kind: sphere {radius}, capsule {radius, half_length} (segment along
// local Y), box and static box {half_x, half_y, half_z}. `parent` is the lane index of an earlier body
// or -1; `limb` is the caller's label. Neither is used by Box3D: they are lanes for the consumer.
// An anchor (kind 4) is a kinematic body with no shape: it is never hit and never collides, and size,
// density, friction, restitution, rolling resistance and material are ignored. Its pose is set with
// `ymir_box3d_soa_set_anchor_target`; the force and torque lanes never move it.
typedef struct ymir_box3d_soa_body_desc
{
    uint64_t stable_id;
    uint32_t kind;
    int32_t parent;
    int32_t limb;
    uint32_t reserved;
    float position[3];
    float rotation[4]; // x y z w, unit
    float velocity[3];
    float size[3];
    float density;
    float friction;
    float restitution;
    float rolling_resistance; // Box3D's surface rolling resistance, >= 0; spheres and capsules roll without it
    uint64_t material_id; // the caller's surface material id: set as the shape's user material, echoed on hit facts
} ymir_box3d_soa_body_desc;

// A joint between two bodies, given in world space at creation: the frame has its origin at `anchor`
// and its z axis along `axis`; the creation pose is the joint's zero. Revolute turns about that z axis.
// `enable_limit` applies `lower`/`upper` (revolute angle, spherical twist), and `cone_angle` (spherical
// swing) when it is positive. Weld ignores limits.
// A motor joint (kind 3) is a pair of position springs, no velocity target: the linear spring pulls body B's
// frame origin onto body A's (so B's anchor point onto A's), the angular spring pulls B's frame orientation onto
// A's. `*_hertz` 0 turns that spring off; `max_force` (N) and `max_torque` (N m) bound what it may apply, so a
// weak grab hangs below its target. All six are finite and >= 0 and are ignored by the other kinds.
// `ymir_box3d_soa_destroy_joint` releases a motor joint: its lane becomes a tombstone (kind REMOVED = 4, bodies -1,
// every lane 0) that add_joint may reuse only once a step has completed since.
typedef struct ymir_box3d_soa_joint_desc
{
    uint32_t kind;
    int32_t body_a;
    int32_t body_b;
    uint32_t enable_limit;
    float anchor[3];
    float axis[3];
    float lower;
    float upper;
    float cone_angle;
    float linear_hertz;
    float linear_damping;
    float max_force;
    float angular_hertz;
    float angular_damping;
    float max_torque;
} ymir_box3d_soa_joint_desc;

// Pointers first and in lane order, so the layout is the same on every 64-bit target. Lanes the caller
// writes (force and torque per body) are mutable; every other lane is read-only to the caller.
typedef struct ymir_box3d_soa_view
{
    uint32_t abi_version;
    int32_t count;
    int32_t capacity;
    int32_t joint_count;
    int32_t joint_capacity;
    uint32_t reserved;
    uint64_t step_count;
    // bodies
    const uint64_t* stable_id;
    const uint32_t* kind;
    const int32_t* parent;
    const int32_t* limb;
    // The body's shape size, read-only, same per-kind meaning as the body desc size: sphere {radius},
    // capsule {radius, half_length}, box and static box {half_x, half_y, half_z}; unused components are 0.
    const float* size_0;
    const float* size_1;
    const float* size_2;
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
    float* force_x;
    float* force_y;
    float* force_z;
    float* torque_x;
    float* torque_y;
    float* torque_z;
    // joints
    const uint32_t* joint_kind;
    const int32_t* joint_body_a;
    const int32_t* joint_body_b;
    const float* joint_angle;
    const float* joint_frame_x;
    const float* joint_frame_y;
    const float* joint_frame_z;
    const float* joint_frame_w;
    const float* joint_rotation_x;
    const float* joint_rotation_y;
    const float* joint_rotation_z;
    const float* joint_rotation_w;
    const float* joint_angular_velocity_x;
    const float* joint_angular_velocity_y;
    const float* joint_angular_velocity_z;
    // World position of the joint frame origin on body A (the joint anchor), refreshed every step like joint_frame.
    const float* joint_origin_x;
    const float* joint_origin_y;
    const float* joint_origin_z;
    // Contact lanes, read-only, refreshed every step. contact_impulse is N s: the sum over the body's touching
    // contact manifold points of stepNormalImpulse, the normal impulse the contacts delivered to the body over
    // the whole step (0 for a body touching nothing). Exact, normal channel only: Box3D exposes no tangent
    // impulse. A resting load is contact_impulse / dt.
    // contact_count is the number of touching contacts. material_id is the body's desc material_id.
    const float* contact_impulse;
    const int32_t* contact_count;
    const uint64_t* material_id;
    // Per joint slot: counts the destroys of that slot. A grab's handle is (lane, generation); destroy_joint refuses
    // a handle whose generation is not the slot's, so a stale release cannot destroy a later grab in a reused slot.
    const uint32_t* joint_generation;
} ymir_box3d_soa_view;

// A begin or end of touch between two bodies: their lane indices and the shapes' stable ids.
typedef struct ymir_box3d_soa_touch_fact
{
    int32_t lane_a;
    int32_t lane_b;
    uint64_t stable_id_a;
    uint64_t stable_id_b;
} ymir_box3d_soa_touch_fact;

// A hit: a touch approaching faster than the world's hit threshold. `normal` points from A to B.
typedef struct ymir_box3d_soa_hit_fact
{
    int32_t lane_a;
    int32_t lane_b;
    uint64_t stable_id_a;
    uint64_t stable_id_b;
    float point[3];
    float normal[3];
    float approach_speed;
    // The normal impulse (N s) the pair's contact delivered over the whole step: the sum of stepNormalImpulse
    // over the manifold of the two shapes, read after the step. Exact, normal channel only.
    float impulse;
    uint64_t material_a;
    uint64_t material_b;
} ymir_box3d_soa_hit_fact;

// What Box3D reported for the step just taken, in Box3D's event order. The arrays are fixed for the
// world's life and rewritten by every step: facts last one step. `dropped` counts facts past
// `fact_capacity` that this step could not keep.
typedef struct ymir_box3d_soa_facts
{
    uint32_t begin_count;
    uint32_t end_count;
    uint32_t hit_count;
    uint32_t fact_capacity;
    uint32_t begin_dropped;
    uint32_t end_dropped;
    uint32_t hit_dropped;
    uint32_t reserved;
    const ymir_box3d_soa_touch_fact* begin;
    const ymir_box3d_soa_touch_fact* end;
    const ymir_box3d_soa_hit_fact* hit;
} ymir_box3d_soa_facts;

// The closest hit of a ray. `hit` is 0 for a miss and every other field is then 0. `lane` and `stable_id` name
// the body and shape hit; `fraction` is of the ray's translation.
typedef struct ymir_box3d_soa_ray_hit
{
    int32_t hit;
    int32_t lane;
    uint64_t stable_id;
    float point[3];
    float normal[3];
    float fraction;
    uint32_t reserved;
} ymir_box3d_soa_ray_hit;

#define YMIR_SOA_STATE_LANES 13
#define YMIR_SOA_WRITE_LANES 6
#define YMIR_SOA_CONTACT_IMPULSE_LANE (YMIR_SOA_STATE_LANES + YMIR_SOA_WRITE_LANES)
#define YMIR_SOA_BODY_FLOAT_LANES (YMIR_SOA_CONTACT_IMPULSE_LANE + 1)
#define YMIR_SOA_JOINT_FLOAT_LANES 15

typedef struct ymir_box3d_soa_world
{
    b3WorldId world_id;
    int32_t count;
    int32_t capacity;
    int32_t joint_count;
    int32_t joint_capacity;
    uint64_t step_count;
    b3BodyId* bodies;
    uint64_t* stable_id;
    uint32_t* kind;
    int32_t* parent;
    int32_t* limb;
    int32_t* contact_count;
    uint64_t* material_id;
    float* size; // 3 arrays of `capacity` floats, lane-major; written once in add_body
    float* lanes; // YMIR_SOA_BODY_FLOAT_LANES arrays of `capacity` floats, lane-major
    b3JointId* joints;
    uint32_t* joint_kind;
    uint32_t* joint_generation;
    uint64_t* joint_removed_at; // step_count when the slot was destroyed; reusable once a step has passed
    int32_t* joint_body_a;
    int32_t* joint_body_b;
    b3Quat* joint_local_a; // joint frame in body A, kept so the state is read without asking Box3D
    b3Vec3* joint_local_origin_a; // the frame origin in body A, kept for the same reason
    b3Quat* joint_local_b;
    float* joint_lanes; // YMIR_SOA_JOINT_FLOAT_LANES arrays of `joint_capacity` floats, lane-major
    uint32_t fact_capacity;
    ymir_box3d_soa_facts facts; // counts and dropped counters live here; the arrays below back its pointers
    ymir_box3d_soa_touch_fact* begin_facts;
    ymir_box3d_soa_touch_fact* end_facts;
    ymir_box3d_soa_hit_fact* hit_facts;
    b3ContactData* contact_scratch; // `capacity` entries: a body has one shape, so at most capacity - 1 contacts
} ymir_box3d_soa_world;

static float* ymir_soa_lane(const ymir_box3d_soa_world* world, int lane)
{
    return world->lanes + (size_t)lane * (size_t)world->capacity;
}

static float* ymir_soa_joint_lane(const ymir_box3d_soa_world* world, int lane)
{
    return world->joint_lanes + (size_t)lane * (size_t)world->joint_capacity;
}

static int ymir_soa_finite(float v)
{
    return isfinite(v);
}

static int ymir_soa_finite3(const float* v)
{
    return ymir_soa_finite(v[0]) && ymir_soa_finite(v[1]) && ymir_soa_finite(v[2]);
}

static int ymir_soa_is_anchor(uint32_t kind)
{
    return kind == YMIR_SOA_KIND_ANCHOR;
}

static int ymir_soa_is_dynamic(uint32_t kind)
{
    return kind != YMIR_SOA_KIND_STATIC_BOX && !ymir_soa_is_anchor(kind);
}

static void ymir_soa_refresh(ymir_box3d_soa_world* world, int index)
{
    b3BodyId body = world->bodies[index];
    b3Pos p = b3Body_GetPosition(body);
    b3Quat q = b3Body_GetRotation(body);
    b3Vec3 v = b3Body_GetLinearVelocity(body);
    b3Vec3 w = b3Body_GetAngularVelocity(body);
    const float values[YMIR_SOA_STATE_LANES] = {
        p.x, p.y, p.z, q.v.x, q.v.y, q.v.z, q.s, v.x, v.y, v.z, w.x, w.y, w.z};
    for (int lane = 0; lane < YMIR_SOA_STATE_LANES; ++lane)
    {
        ymir_soa_lane(world, lane)[index] = values[lane];
    }
}

static b3Transform ymir_soa_body_transform(b3BodyId body)
{
    b3Pos p = b3Body_GetPosition(body);
    b3Transform t = {{p.x, p.y, p.z}, b3Body_GetRotation(body)};
    return t;
}

// Joint state from the bodies: the world orientation of frame A, frame B relative to frame A (as a
// quaternion with w >= 0, the shorter arc) and the relative angular velocity in frame A. The angle
// lane is Box3D's own: the revolute angle or the spherical twist angle, zero for a weld.
static void ymir_soa_refresh_joint(ymir_box3d_soa_world* world, int j)
{
    int a = world->joint_body_a[j];
    int b = world->joint_body_b[j];
    b3Quat qa = b3MulQuat(b3Body_GetRotation(world->bodies[a]), world->joint_local_a[j]);
    b3Vec3 origin = b3TransformPoint(ymir_soa_body_transform(world->bodies[a]), world->joint_local_origin_a[j]);
    b3Quat qb = b3MulQuat(b3Body_GetRotation(world->bodies[b]), world->joint_local_b[j]);
    b3Quat rel = b3InvMulQuat(qa, qb);
    if (rel.s < 0.0f)
    {
        rel = b3NegateQuat(rel);
    }
    b3Vec3 w = b3InvRotateVector(
        qa, b3Sub(b3Body_GetAngularVelocity(world->bodies[b]), b3Body_GetAngularVelocity(world->bodies[a])));
    float angle = 0.0f;
    if (world->joint_kind[j] == YMIR_SOA_JOINT_REVOLUTE)
    {
        angle = b3RevoluteJoint_GetAngle(world->joints[j]);
    }
    else if (world->joint_kind[j] == YMIR_SOA_JOINT_SPHERICAL)
    {
        angle = b3SphericalJoint_GetTwistAngle(world->joints[j]);
    }
    const float values[YMIR_SOA_JOINT_FLOAT_LANES] = {
        angle, qa.v.x, qa.v.y, qa.v.z, qa.s, rel.v.x, rel.v.y, rel.v.z, rel.s, w.x, w.y, w.z, origin.x, origin.y, origin.z};
    for (int lane = 0; lane < YMIR_SOA_JOINT_FLOAT_LANES; ++lane)
    {
        ymir_soa_joint_lane(world, lane)[j] = values[lane];
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

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_body_desc_size(void)
{
    return (uint32_t)sizeof(ymir_box3d_soa_body_desc);
}

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_joint_desc_size(void)
{
    return (uint32_t)sizeof(ymir_box3d_soa_joint_desc);
}

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_ray_hit_size(void)
{
    return (uint32_t)sizeof(ymir_box3d_soa_ray_hit);
}

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_touch_fact_size(void)
{
    return (uint32_t)sizeof(ymir_box3d_soa_touch_fact);
}

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_hit_fact_size(void)
{
    return (uint32_t)sizeof(ymir_box3d_soa_hit_fact);
}

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_soa_get_facts_size(void)
{
    return (uint32_t)sizeof(ymir_box3d_soa_facts);
}

static void ymir_soa_free(ymir_box3d_soa_world* world)
{
    free(world->bodies);
    free(world->stable_id);
    free(world->kind);
    free(world->parent);
    free(world->limb);
    free(world->contact_count);
    free(world->material_id);
    free(world->size);
    free(world->lanes);
    free(world->joints);
    free(world->joint_kind);
    free(world->joint_removed_at);
    free(world->joint_generation);
    free(world->joint_body_a);
    free(world->joint_body_b);
    free(world->joint_local_a);
    free(world->joint_local_origin_a);
    free(world->joint_local_b);
    free(world->joint_lanes);
    free(world->begin_facts);
    free(world->end_facts);
    free(world->hit_facts);
    free(world->contact_scratch);
    free(world);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_create(
    int32_t capacity, int32_t joint_capacity, float gravity_x, float gravity_y, float gravity_z,
    int32_t fact_capacity, float hit_threshold, ymir_box3d_soa_world** output)
{
    if (output == NULL)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    *output = NULL;
    if (capacity <= 0 || joint_capacity < 0 || fact_capacity <= 0 || !ymir_soa_finite(gravity_x) ||
        !ymir_soa_finite(gravity_y) || !ymir_soa_finite(gravity_z) || !ymir_soa_finite(hit_threshold) ||
        hit_threshold < 0.0f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }

    ymir_box3d_soa_world* world = (ymir_box3d_soa_world*)calloc(1, sizeof(ymir_box3d_soa_world));
    if (world == NULL)
    {
        return YMIR_SOA_OUT_OF_MEMORY;
    }
    world->capacity = capacity;
    world->joint_capacity = joint_capacity;
    size_t n = (size_t)capacity;
    size_t m = (size_t)joint_capacity > 0 ? (size_t)joint_capacity : 1; // calloc(0) may return NULL
    world->bodies = (b3BodyId*)calloc(n, sizeof(b3BodyId));
    world->stable_id = (uint64_t*)calloc(n, sizeof(uint64_t));
    world->kind = (uint32_t*)calloc(n, sizeof(uint32_t));
    world->parent = (int32_t*)calloc(n, sizeof(int32_t));
    world->limb = (int32_t*)calloc(n, sizeof(int32_t));
    world->contact_count = (int32_t*)calloc(n, sizeof(int32_t));
    world->material_id = (uint64_t*)calloc(n, sizeof(uint64_t));
    world->size = (float*)calloc(n * 3, sizeof(float));
    world->lanes = (float*)calloc(n * YMIR_SOA_BODY_FLOAT_LANES, sizeof(float));
    world->joints = (b3JointId*)calloc(m, sizeof(b3JointId));
    world->joint_kind = (uint32_t*)calloc(m, sizeof(uint32_t));
    world->joint_removed_at = (uint64_t*)calloc(m, sizeof(uint64_t));
    world->joint_generation = (uint32_t*)calloc(m, sizeof(uint32_t));
    world->joint_body_a = (int32_t*)calloc(m, sizeof(int32_t));
    world->joint_body_b = (int32_t*)calloc(m, sizeof(int32_t));
    world->joint_local_a = (b3Quat*)calloc(m, sizeof(b3Quat));
    world->joint_local_origin_a = (b3Vec3*)calloc(m, sizeof(b3Vec3));
    world->joint_local_b = (b3Quat*)calloc(m, sizeof(b3Quat));
    world->joint_lanes = (float*)calloc(m * YMIR_SOA_JOINT_FLOAT_LANES, sizeof(float));
    world->fact_capacity = (uint32_t)fact_capacity;
    world->begin_facts = (ymir_box3d_soa_touch_fact*)calloc((size_t)fact_capacity, sizeof(ymir_box3d_soa_touch_fact));
    world->end_facts = (ymir_box3d_soa_touch_fact*)calloc((size_t)fact_capacity, sizeof(ymir_box3d_soa_touch_fact));
    world->hit_facts = (ymir_box3d_soa_hit_fact*)calloc((size_t)fact_capacity, sizeof(ymir_box3d_soa_hit_fact));
    world->contact_scratch = (b3ContactData*)calloc(n, sizeof(b3ContactData));
    if (world->contact_count == NULL || world->material_id == NULL || world->begin_facts == NULL ||
        world->end_facts == NULL || world->hit_facts == NULL || world->contact_scratch == NULL ||
        world->bodies == NULL || world->stable_id == NULL || world->kind == NULL || world->parent == NULL ||
        world->limb == NULL || world->size == NULL || world->lanes == NULL || world->joints == NULL || world->joint_kind == NULL ||
        world->joint_body_a == NULL || world->joint_body_b == NULL || world->joint_local_a == NULL || world->joint_local_origin_a == NULL ||
        world->joint_local_b == NULL || world->joint_lanes == NULL || world->joint_removed_at == NULL ||
        world->joint_generation == NULL)
    {
        ymir_soa_free(world);
        return YMIR_SOA_OUT_OF_MEMORY;
    }

    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = (b3Vec3){gravity_x, gravity_y, gravity_z};
    def.enableSleep = false;
    def.hitEventThreshold = hit_threshold;
    def.workerCount = 1; // one worker: the step order is fixed, which determinism rests on
    world->world_id = ymir_create_world_locked(&def);
    if (B3_IS_NULL(world->world_id))
    {
        ymir_soa_free(world);
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
        ymir_destroy_world_locked(world->world_id);
    }
    ymir_soa_free(world);
}

static int32_t ymir_soa_check_body(const ymir_box3d_soa_world* world, const ymir_box3d_soa_body_desc* d)
{
    if (d->kind > YMIR_SOA_KIND_ANCHOR || !ymir_soa_finite3(d->position) || !ymir_soa_finite3(d->velocity) ||
        !ymir_soa_finite3(d->size) || !ymir_soa_finite(d->friction) || !ymir_soa_finite(d->restitution) ||
        !ymir_soa_finite(d->density) || !ymir_soa_finite(d->rolling_resistance) || d->friction < 0.0f ||
        d->restitution < 0.0f || d->rolling_resistance < 0.0f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    float q2 = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        if (!ymir_soa_finite(d->rotation[i]))
        {
            return YMIR_SOA_INVALID_ARGUMENT;
        }
        q2 += d->rotation[i] * d->rotation[i];
    }
    if (fabsf(q2 - 1.0f) > 1e-3f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    if (d->parent < -1 || d->parent >= world->count)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    if (ymir_soa_is_dynamic(d->kind) && d->density <= 0.0f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    switch (d->kind)
    {
        case YMIR_SOA_KIND_ANCHOR:
            break; // no shape: nothing to size
        case YMIR_SOA_KIND_SPHERE:
            if (d->size[0] <= 0.0f)
            {
                return YMIR_SOA_INVALID_ARGUMENT;
            }
            break;
        case YMIR_SOA_KIND_CAPSULE:
            if (d->size[0] <= 0.0f || d->size[1] <= 0.0f)
            {
                return YMIR_SOA_INVALID_ARGUMENT;
            }
            break;
        default:
            if (d->size[0] <= 0.0f || d->size[1] <= 0.0f || d->size[2] <= 0.0f)
            {
                return YMIR_SOA_INVALID_ARGUMENT;
            }
            break;
    }
    if (world->count >= world->capacity)
    {
        return YMIR_SOA_FULL;
    }
    for (int i = 0; i < world->count; ++i)
    {
        if (world->stable_id[i] == d->stable_id)
        {
            return YMIR_SOA_DUPLICATE_ID;
        }
    }
    return YMIR_SOA_OK;
}

// Creates one body from a descriptor. Box3D owns mass: it follows from `density` and the shape.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_add_body(
    ymir_box3d_soa_world* world, const ymir_box3d_soa_body_desc* d, int32_t* index)
{
    if (world == NULL || d == NULL)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    int32_t status = ymir_soa_check_body(world, d);
    if (status != YMIR_SOA_OK)
    {
        return status;
    }

    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = ymir_soa_is_anchor(d->kind) ? b3_kinematicBody
                    : ymir_soa_is_dynamic(d->kind) ? b3_dynamicBody
                                                   : b3_staticBody;
    body_def.position = (b3Vec3){d->position[0], d->position[1], d->position[2]};
    body_def.rotation = (b3Quat){{d->rotation[0], d->rotation[1], d->rotation[2]}, d->rotation[3]};
    body_def.linearVelocity = (b3Vec3){d->velocity[0], d->velocity[1], d->velocity[2]};
    body_def.enableSleep = false;
    body_def.userData = (void*)((uintptr_t)world->count + 1u); // lane + 1: a shape resolves to its lane
    b3BodyId body = b3CreateBody(world->world_id, &body_def);
    if (B3_IS_NULL(body))
    {
        return YMIR_SOA_INTERNAL_ERROR;
    }
    if (!ymir_soa_is_anchor(d->kind)) // an anchor has no shape
    {
        b3ShapeDef shape_def = b3DefaultShapeDef();
        shape_def.userData = (void*)(uintptr_t)d->stable_id;
        shape_def.enableContactEvents = true; // off by default in Box3D
        shape_def.enableHitEvents = true;
        shape_def.baseMaterial.userMaterialId = d->material_id;
        if (ymir_soa_is_dynamic(d->kind))
        {
            shape_def.density = d->density;
        }
        shape_def.baseMaterial.friction = d->friction;
        shape_def.baseMaterial.restitution = d->restitution;
        shape_def.baseMaterial.rollingResistance = d->rolling_resistance;

        b3ShapeId shape;
        if (d->kind == YMIR_SOA_KIND_SPHERE)
        {
            b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, d->size[0]};
            shape = b3CreateSphereShape(body, &shape_def, &sphere);
        }
        else if (d->kind == YMIR_SOA_KIND_CAPSULE)
        {
            b3Capsule capsule = {{0.0f, -d->size[1], 0.0f}, {0.0f, d->size[1], 0.0f}, d->size[0]};
            shape = b3CreateCapsuleShape(body, &shape_def, &capsule);
        }
        else
        {
            b3BoxHull box = b3MakeBoxHull(d->size[0], d->size[1], d->size[2]);
            shape = b3CreateHullShape(body, &shape_def, &box.base);
        }
        if (B3_IS_NULL(shape))
        {
            b3DestroyBody(body);
            return YMIR_SOA_INTERNAL_ERROR;
        }
    }

    int i = world->count++;
    world->bodies[i] = body;
    world->stable_id[i] = d->stable_id;
    world->kind[i] = d->kind;
    world->parent[i] = d->parent;
    world->limb[i] = d->limb;
    world->material_id[i] = ymir_soa_is_anchor(d->kind) ? 0 : d->material_id;
    /* Publish only the components the kind uses; unused ones read 0 whatever the caller passed. */
    int used = ymir_soa_is_anchor(d->kind) ? 0 : d->kind == YMIR_SOA_KIND_SPHERE ? 1 : d->kind == YMIR_SOA_KIND_CAPSULE ? 2 : 3;
    for (int k = 0; k < 3; k++)
    {
        world->size[(size_t)k * (size_t)world->capacity + (size_t)i] = k < used ? d->size[k] : 0.0f;
    }
    ymir_soa_refresh(world, i);
    if (index != NULL)
    {
        *index = i;
    }
    return YMIR_SOA_OK;
}

// The lane a new joint takes: the lowest tombstone whose destroying step has completed, else the next fresh
// slot, else -1. A destroyed lane is not reused within the step that destroyed it, so lanes a caller read
// that step stay meaningful.
static int ymir_soa_joint_slot(const ymir_box3d_soa_world* world)
{
    for (int j = 0; j < world->joint_count; ++j)
    {
        if (world->joint_kind[j] == YMIR_SOA_JOINT_REMOVED && world->step_count > world->joint_removed_at[j])
        {
            return j;
        }
    }
    return world->joint_count < world->joint_capacity ? world->joint_count : -1;
}

// Joints that add_joint could create right now.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_free_joint_slots(const ymir_box3d_soa_world* world)
{
    if (world == NULL)
    {
        return 0;
    }
    int32_t free_slots = world->joint_capacity - world->joint_count;
    for (int j = 0; j < world->joint_count; ++j)
    {
        free_slots += world->joint_kind[j] == YMIR_SOA_JOINT_REMOVED && world->step_count > world->joint_removed_at[j];
    }
    return free_slots;
}

// Creates a joint between two existing bodies. The joint frame is built in world space (origin at the
// anchor, z along the axis) and expressed in each body's local frame, so the creation pose is zero.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_add_joint(
    ymir_box3d_soa_world* world, const ymir_box3d_soa_joint_desc* d, int32_t* index)
{
    if (world == NULL || d == NULL || d->kind > YMIR_SOA_JOINT_MOTOR || d->body_a < 0 || d->body_a >= world->count ||
        d->body_b < 0 || d->body_b >= world->count || d->body_a == d->body_b || !ymir_soa_finite3(d->anchor) ||
        !ymir_soa_finite3(d->axis) || !ymir_soa_finite(d->lower) || !ymir_soa_finite(d->upper) ||
        !ymir_soa_finite(d->cone_angle) || d->cone_angle < 0.0f || d->cone_angle > B3_PI ||
        !ymir_soa_finite(d->linear_hertz) || !ymir_soa_finite(d->linear_damping) || !ymir_soa_finite(d->max_force) ||
        !ymir_soa_finite(d->angular_hertz) || !ymir_soa_finite(d->angular_damping) || !ymir_soa_finite(d->max_torque) ||
        d->linear_hertz < 0.0f || d->linear_damping < 0.0f || d->max_force < 0.0f || d->angular_hertz < 0.0f ||
        d->angular_damping < 0.0f || d->max_torque < 0.0f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    b3Vec3 axis = {d->axis[0], d->axis[1], d->axis[2]};
    float length = sqrtf(b3Dot(axis, axis));
    if (length < 1e-6f || (d->enable_limit != 0 && d->kind != YMIR_SOA_JOINT_WELD && d->lower > d->upper))
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    int slot = ymir_soa_joint_slot(world);
    if (slot < 0)
    {
        return YMIR_SOA_FULL;
    }
    axis = b3MulSV(1.0f / length, axis);

    b3Transform frame = {{d->anchor[0], d->anchor[1], d->anchor[2]}, b3ComputeQuatBetweenUnitVectors((b3Vec3){0.0f, 0.0f, 1.0f}, axis)};
    b3Transform local_a = b3InvMulTransforms(ymir_soa_body_transform(world->bodies[d->body_a]), frame);
    b3Transform local_b = b3InvMulTransforms(ymir_soa_body_transform(world->bodies[d->body_b]), frame);

    b3JointId joint;
    if (d->kind == YMIR_SOA_JOINT_REVOLUTE)
    {
        b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
        def.base.bodyIdA = world->bodies[d->body_a];
        def.base.bodyIdB = world->bodies[d->body_b];
        def.base.localFrameA = local_a;
        def.base.localFrameB = local_b;
        def.enableLimit = d->enable_limit != 0;
        def.lowerAngle = d->lower;
        def.upperAngle = d->upper;
        joint = b3CreateRevoluteJoint(world->world_id, &def);
    }
    else if (d->kind == YMIR_SOA_JOINT_SPHERICAL)
    {
        b3SphericalJointDef def = b3DefaultSphericalJointDef();
        def.base.bodyIdA = world->bodies[d->body_a];
        def.base.bodyIdB = world->bodies[d->body_b];
        def.base.localFrameA = local_a;
        def.base.localFrameB = local_b;
        def.enableConeLimit = d->enable_limit != 0 && d->cone_angle > 0.0f;
        def.coneAngle = d->cone_angle;
        def.enableTwistLimit = d->enable_limit != 0;
        def.lowerTwistAngle = d->lower;
        def.upperTwistAngle = d->upper;
        joint = b3CreateSphericalJoint(world->world_id, &def);
    }
    else if (d->kind == YMIR_SOA_JOINT_MOTOR)
    {
        b3MotorJointDef def = b3DefaultMotorJointDef();
        def.base.bodyIdA = world->bodies[d->body_a];
        def.base.bodyIdB = world->bodies[d->body_b];
        def.base.localFrameA = local_a;
        def.base.localFrameB = local_b;
        def.linearHertz = d->linear_hertz;
        def.linearDampingRatio = d->linear_damping;
        def.maxSpringForce = d->max_force;
        def.angularHertz = d->angular_hertz;
        def.angularDampingRatio = d->angular_damping;
        def.maxSpringTorque = d->max_torque;
        joint = b3CreateMotorJoint(world->world_id, &def);
    }
    else
    {
        b3WeldJointDef def = b3DefaultWeldJointDef();
        def.base.bodyIdA = world->bodies[d->body_a];
        def.base.bodyIdB = world->bodies[d->body_b];
        def.base.localFrameA = local_a;
        def.base.localFrameB = local_b;
        joint = b3CreateWeldJoint(world->world_id, &def);
    }
    if (B3_IS_NULL(joint))
    {
        return YMIR_SOA_INTERNAL_ERROR;
    }

    int j = slot;
    if (j == world->joint_count)
    {
        world->joint_count++;
    }
    world->joints[j] = joint;
    world->joint_kind[j] = d->kind;
    world->joint_body_a[j] = d->body_a;
    world->joint_body_b[j] = d->body_b;
    world->joint_local_a[j] = local_a.q;
    world->joint_local_origin_a[j] = local_a.p;
    world->joint_local_b[j] = local_b.q;
    ymir_soa_refresh_joint(world, j);
    if (index != NULL)
    {
        *index = j;
    }
    return YMIR_SOA_OK;
}

// Releases a motor joint (a grab) named by its lane and the slot's generation read when it was added. Only motor
// joints: general joint removal is another cut's. A generation that is not the slot's current one is refused, so
// a stale handle never destroys a later joint in a reused slot. The lane becomes a tombstone, kind REMOVED,
// bodies -1 and every lane 0, until a later step lets add_joint reuse it.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_destroy_joint(ymir_box3d_soa_world* world, int32_t joint, uint32_t generation)
{
    if (world == NULL || joint < 0 || joint >= world->joint_count || world->joint_kind[joint] != YMIR_SOA_JOINT_MOTOR ||
        world->joint_generation[joint] != generation)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    b3DestroyJoint(world->joints[joint], true);
    world->joints[joint] = (b3JointId){0};
    world->joint_kind[joint] = YMIR_SOA_JOINT_REMOVED;
    world->joint_body_a[joint] = -1;
    world->joint_body_b[joint] = -1;
    world->joint_removed_at[joint] = world->step_count;
    world->joint_generation[joint] += 1;
    for (int lane = 0; lane < YMIR_SOA_JOINT_FLOAT_LANES; ++lane)
    {
        ymir_soa_joint_lane(world, lane)[joint] = 0.0f;
    }
    return YMIR_SOA_OK;
}

// Applies the caller's force and torque lanes to the dynamic bodies, then clears them: a lane holds one
// step's intent, so a stale force is never applied twice. A non-finite value refuses the whole step
// before any body is touched.
static int32_t ymir_soa_apply_write_lanes(ymir_box3d_soa_world* world)
{
    float* lane[YMIR_SOA_WRITE_LANES];
    for (int k = 0; k < YMIR_SOA_WRITE_LANES; ++k)
    {
        lane[k] = ymir_soa_lane(world, YMIR_SOA_STATE_LANES + k);
        for (int i = 0; i < world->count; ++i)
        {
            if (!ymir_soa_finite(lane[k][i]))
            {
                return YMIR_SOA_INVALID_ARGUMENT;
            }
        }
    }
    for (int i = 0; i < world->count; ++i)
    {
        b3Vec3 force = {lane[0][i], lane[1][i], lane[2][i]};
        b3Vec3 torque = {lane[3][i], lane[4][i], lane[5][i]};
        if (ymir_soa_is_dynamic(world->kind[i]))
        {
            if (force.x != 0.0f || force.y != 0.0f || force.z != 0.0f)
            {
                b3Body_ApplyForceToCenter(world->bodies[i], force, true);
            }
            if (torque.x != 0.0f || torque.y != 0.0f || torque.z != 0.0f)
            {
                b3Body_ApplyTorque(world->bodies[i], torque, true);
            }
        }
        for (int k = 0; k < YMIR_SOA_WRITE_LANES; ++k)
        {
            lane[k][i] = 0.0f;
        }
    }
    return YMIR_SOA_OK;
}

// The lane of a shape's body, from the body user data set at add_body (lane + 1).
static int32_t ymir_soa_shape_lane(b3ShapeId shape)
{
    return (int32_t)((uintptr_t)b3Body_GetUserData(b3Shape_GetBody(shape))) - 1;
}

static void ymir_soa_touch_fact(
    ymir_box3d_soa_world* world, ymir_box3d_soa_touch_fact* out, uint32_t* count, uint32_t* dropped, b3ShapeId a,
    b3ShapeId b)
{
    if (*count >= world->fact_capacity)
    {
        *dropped += 1;
        return;
    }
    out[(*count)++] = (ymir_box3d_soa_touch_fact){
        ymir_soa_shape_lane(a), ymir_soa_shape_lane(b), (uint64_t)(uintptr_t)b3Shape_GetUserData(a),
        (uint64_t)(uintptr_t)b3Shape_GetUserData(b)};
}

// The normal impulse (N s) a contact's manifold points delivered over the step.
static float ymir_soa_manifold_impulse(const b3ContactData* data)
{
    float impulse = 0.0f;
    for (int m = 0; m < data->manifoldCount; ++m)
    {
        const b3Manifold* manifold = data->manifolds + m;
        for (int p = 0; p < manifold->pointCount; ++p)
        {
            impulse += manifold->points[p].stepNormalImpulse;
        }
    }
    return impulse;
}

// A hit's impulse: the contact between its two shapes. A body has one shape, so a shape has at most
// capacity - 1 contacts and the scratch holds them all.
static float ymir_soa_pair_impulse(ymir_box3d_soa_world* world, const b3ContactHitEvent* e)
{
    int n = b3Shape_GetContactData(e->shapeIdA, world->contact_scratch, world->capacity);
    for (int c = 0; c < n; ++c)
    {
        const b3ContactData* data = world->contact_scratch + c;
        b3ShapeId other = B3_ID_EQUALS(data->shapeIdA, e->shapeIdA) ? data->shapeIdB : data->shapeIdA;
        if (B3_ID_EQUALS(other, e->shapeIdB))
        {
            return ymir_soa_manifold_impulse(data);
        }
    }
    return 0.0f;
}

// Rewrites the fact arrays from Box3D's contact events for the step just taken, in Box3D's event order.
// Writes only into the buffers allocated at create: nothing is allocated here.
static void ymir_soa_collect_facts(ymir_box3d_soa_world* world)
{
    ymir_box3d_soa_facts* f = &world->facts;
    f->begin_count = f->end_count = f->hit_count = 0;
    f->begin_dropped = f->end_dropped = f->hit_dropped = 0;
    b3ContactEvents events = b3World_GetContactEvents(world->world_id);
    for (int i = 0; i < events.beginCount; ++i)
    {
        const b3ContactBeginTouchEvent* e = events.beginEvents + i;
        ymir_soa_touch_fact(world, world->begin_facts, &f->begin_count, &f->begin_dropped, e->shapeIdA, e->shapeIdB);
    }
    for (int i = 0; i < events.endCount; ++i)
    {
        const b3ContactEndTouchEvent* e = events.endEvents + i;
        ymir_soa_touch_fact(world, world->end_facts, &f->end_count, &f->end_dropped, e->shapeIdA, e->shapeIdB);
    }
    for (int i = 0; i < events.hitCount; ++i)
    {
        const b3ContactHitEvent* e = events.hitEvents + i;
        if (f->hit_count >= world->fact_capacity)
        {
            f->hit_dropped += 1;
            continue;
        }
        world->hit_facts[f->hit_count++] = (ymir_box3d_soa_hit_fact){
            .lane_a = ymir_soa_shape_lane(e->shapeIdA),
            .lane_b = ymir_soa_shape_lane(e->shapeIdB),
            .stable_id_a = (uint64_t)(uintptr_t)b3Shape_GetUserData(e->shapeIdA),
            .stable_id_b = (uint64_t)(uintptr_t)b3Shape_GetUserData(e->shapeIdB),
            .point = {(float)e->point.x, (float)e->point.y, (float)e->point.z},
            .normal = {e->normal.x, e->normal.y, e->normal.z},
            .approach_speed = e->approachSpeed,
            .impulse = ymir_soa_pair_impulse(world, e),
            .material_a = e->userMaterialIdA,
            .material_b = e->userMaterialIdB,
        };
    }
}

// Per body: the touching contacts' count and the normal impulse they delivered over the whole step.
static void ymir_soa_collect_contact_lanes(ymir_box3d_soa_world* world)
{
    float* lane = ymir_soa_lane(world, YMIR_SOA_CONTACT_IMPULSE_LANE);
    for (int i = 0; i < world->count; ++i)
    {
        int n = b3Body_GetContactData(world->bodies[i], world->contact_scratch, world->capacity);
        float impulse = 0.0f;
        for (int c = 0; c < n; ++c)
        {
            impulse += ymir_soa_manifold_impulse(world->contact_scratch + c);
        }
        world->contact_count[i] = n;
        lane[i] = impulse;
    }
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_step(ymir_box3d_soa_world* world, float time_step, int32_t sub_step_count)
{
    if (world == NULL || !ymir_soa_finite(time_step) || time_step <= 0.0f || sub_step_count <= 0)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    int32_t status = ymir_soa_apply_write_lanes(world);
    if (status != YMIR_SOA_OK)
    {
        return status;
    }
    b3World_Step(world->world_id, time_step, sub_step_count);
    // An anchor moves only on a target: the one just applied has been reached, so it holds still until the
    // next target. Otherwise its velocity would carry it on past the pose (and fling what it holds).
    for (int i = 0; i < world->count; ++i)
    {
        if (ymir_soa_is_anchor(world->kind[i]))
        {
            b3Body_SetLinearVelocity(world->bodies[i], b3Vec3_zero);
            b3Body_SetAngularVelocity(world->bodies[i], b3Vec3_zero);
        }
    }
    ymir_soa_collect_facts(world);
    ymir_soa_collect_contact_lanes(world);
    for (int i = 0; i < world->count; ++i)
    {
        ymir_soa_refresh(world, i);
    }
    for (int j = 0; j < world->joint_count; ++j)
    {
        if (world->joint_kind[j] != YMIR_SOA_JOINT_REMOVED)
        {
            ymir_soa_refresh_joint(world, j);
        }
    }
    world->step_count += 1;
    return YMIR_SOA_OK;
}

// Grab and poke input, each an immediate command on the caller's say-so. Ymir decides nothing about grabbing.

// Sets an anchor's pose for the coming step: Box3D gives it the velocity that reaches the pose after
// `time_step` seconds, so pass the step's own dt and the anchor arrives at the end of that step, where `step`
// stops it (velocity 0) until the next target.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_set_anchor_target(
    ymir_box3d_soa_world* world, int32_t lane, const float* position, const float* rotation, float time_step)
{
    if (world == NULL || position == NULL || rotation == NULL || lane < 0 || lane >= world->count ||
        !ymir_soa_is_anchor(world->kind[lane]) || !ymir_soa_finite3(position) || !ymir_soa_finite(time_step) ||
        time_step <= 0.0f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    float q2 = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        if (!ymir_soa_finite(rotation[i]))
        {
            return YMIR_SOA_INVALID_ARGUMENT;
        }
        q2 += rotation[i] * rotation[i];
    }
    if (fabsf(q2 - 1.0f) > 1e-3f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    b3WorldTransform target = {
        {position[0], position[1], position[2]}, {{rotation[0], rotation[1], rotation[2]}, rotation[3]}};
    b3Body_SetTargetTransform(world->bodies[lane], target, time_step, true);
    return YMIR_SOA_OK;
}

// A one-shot world impulse (N s) applied at a world point, so it turns the body when the point is off the
// centre of mass. Dynamic bodies only.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_apply_impulse(
    ymir_box3d_soa_world* world, int32_t lane, const float* impulse, const float* point)
{
    if (world == NULL || impulse == NULL || point == NULL || lane < 0 || lane >= world->count ||
        !ymir_soa_is_dynamic(world->kind[lane]) || !ymir_soa_finite3(impulse) || !ymir_soa_finite3(point))
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    b3Body_ApplyLinearImpulse(
        world->bodies[lane], (b3Vec3){impulse[0], impulse[1], impulse[2]}, (b3Pos){point[0], point[1], point[2]}, true);
    return YMIR_SOA_OK;
}

// The closest shape along origin + t * translation, t in [0, 1], with Box3D's default filter. A miss is a
// success with `hit` 0. Refuses a non-finite or zero-length translation.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_cast_ray_closest(
    const ymir_box3d_soa_world* world, const float* origin, const float* translation, ymir_box3d_soa_ray_hit* out)
{
    if (world == NULL || origin == NULL || translation == NULL || out == NULL || !ymir_soa_finite3(origin) ||
        !ymir_soa_finite3(translation) ||
        translation[0] * translation[0] + translation[1] * translation[1] + translation[2] * translation[2] <= 0.0f)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    memset(out, 0, sizeof(*out));
    b3RayResult r = b3World_CastRayClosest(
        world->world_id, (b3Pos){origin[0], origin[1], origin[2]},
        (b3Vec3){translation[0], translation[1], translation[2]}, b3DefaultQueryFilter());
    if (!r.hit)
    {
        return YMIR_SOA_OK;
    }
    out->hit = 1;
    out->lane = ymir_soa_shape_lane(r.shapeId);
    out->stable_id = (uint64_t)(uintptr_t)b3Shape_GetUserData(r.shapeId);
    out->point[0] = (float)r.point.x;
    out->point[1] = (float)r.point.y;
    out->point[2] = (float)r.point.z;
    out->normal[0] = r.normal.x;
    out->normal[1] = r.normal.y;
    out->normal[2] = r.normal.z;
    out->fraction = r.fraction;
    return YMIR_SOA_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_get_view(const ymir_box3d_soa_world* world, ymir_box3d_soa_view* view)
{
    if (world == NULL || view == NULL)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    float* base = world->lanes;
    size_t n = (size_t)world->capacity;
    float* jb = world->joint_lanes;
    size_t m = (size_t)world->joint_capacity;
    *view = (ymir_box3d_soa_view){
        .abi_version = YMIR_SOA_ABI_VERSION,
        .count = world->count,
        .capacity = world->capacity,
        .joint_count = world->joint_count,
        .joint_capacity = world->joint_capacity,
        .step_count = world->step_count,
        .stable_id = world->stable_id,
        .kind = world->kind,
        .parent = world->parent,
        .limb = world->limb,
        .size_0 = world->size + 0 * n,
        .size_1 = world->size + 1 * n,
        .size_2 = world->size + 2 * n,
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
        .force_x = base + 13 * n,
        .force_y = base + 14 * n,
        .force_z = base + 15 * n,
        .torque_x = base + 16 * n,
        .torque_y = base + 17 * n,
        .torque_z = base + 18 * n,
        .joint_kind = world->joint_kind,
        .joint_body_a = world->joint_body_a,
        .joint_body_b = world->joint_body_b,
        .joint_angle = jb + 0 * m,
        .joint_frame_x = jb + 1 * m,
        .joint_frame_y = jb + 2 * m,
        .joint_frame_z = jb + 3 * m,
        .joint_frame_w = jb + 4 * m,
        .joint_rotation_x = jb + 5 * m,
        .joint_rotation_y = jb + 6 * m,
        .joint_rotation_z = jb + 7 * m,
        .joint_rotation_w = jb + 8 * m,
        .joint_angular_velocity_x = jb + 9 * m,
        .joint_angular_velocity_y = jb + 10 * m,
        .joint_angular_velocity_z = jb + 11 * m,
        .joint_origin_x = jb + 12 * m,
        .joint_origin_y = jb + 13 * m,
        .joint_origin_z = jb + 14 * m,
        .contact_impulse = base + YMIR_SOA_CONTACT_IMPULSE_LANE * n,
        .contact_count = world->contact_count,
        .material_id = world->material_id,
        .joint_generation = world->joint_generation,
    };
    return YMIR_SOA_OK;
}

// The facts of the last step: pointers into buffers fixed for the world's life, valid until the next step.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_get_facts(const ymir_box3d_soa_world* world, ymir_box3d_soa_facts* facts)
{
    if (world == NULL || facts == NULL)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    *facts = world->facts;
    facts->fact_capacity = world->fact_capacity;
    facts->begin = world->begin_facts;
    facts->end = world->end_facts;
    facts->hit = world->hit_facts;
    return YMIR_SOA_OK;
}
