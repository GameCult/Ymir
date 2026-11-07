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
    YMIR_SOA_KIND_BOX = 3
};

enum
{
    YMIR_SOA_JOINT_REVOLUTE = 0,
    YMIR_SOA_JOINT_SPHERICAL = 1,
    YMIR_SOA_JOINT_WELD = 2
};

#define YMIR_SOA_ABI_VERSION 2u

// A body to create. `size` by kind: sphere {radius}, capsule {radius, half_length} (segment along
// local Y), box and static box {half_x, half_y, half_z}. `parent` is the lane index of an earlier body
// or -1; `limb` is the caller's label. Neither is used by Box3D: they are lanes for the consumer.
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
    float reserved_float;
} ymir_box3d_soa_body_desc;

// A joint between two bodies, given in world space at creation: the frame has its origin at `anchor`
// and its z axis along `axis`; the creation pose is the joint's zero. Revolute turns about that z axis.
// `enable_limit` applies `lower`/`upper` (revolute angle, spherical twist), and `cone_angle` (spherical
// swing) when it is positive. Weld ignores limits.
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
    float reserved_float;
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
} ymir_box3d_soa_view;

#define YMIR_SOA_STATE_LANES 13
#define YMIR_SOA_WRITE_LANES 6
#define YMIR_SOA_BODY_FLOAT_LANES (YMIR_SOA_STATE_LANES + YMIR_SOA_WRITE_LANES)
#define YMIR_SOA_JOINT_FLOAT_LANES 12

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
    float* lanes; // YMIR_SOA_BODY_FLOAT_LANES arrays of `capacity` floats, lane-major
    b3JointId* joints;
    uint32_t* joint_kind;
    int32_t* joint_body_a;
    int32_t* joint_body_b;
    b3Quat* joint_local_a; // joint frame in body A, kept so the state is read without asking Box3D
    b3Quat* joint_local_b;
    float* joint_lanes; // YMIR_SOA_JOINT_FLOAT_LANES arrays of `joint_capacity` floats, lane-major
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

static int ymir_soa_is_dynamic(uint32_t kind)
{
    return kind != YMIR_SOA_KIND_STATIC_BOX;
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

// Joint state from the bodies: the world orientation of frame A, frame B relative to frame A (as a
// quaternion with w >= 0, the shorter arc) and the relative angular velocity in frame A. The angle
// lane is Box3D's own: the revolute angle or the spherical twist angle, zero for a weld.
static void ymir_soa_refresh_joint(ymir_box3d_soa_world* world, int j)
{
    int a = world->joint_body_a[j];
    int b = world->joint_body_b[j];
    b3Quat qa = b3MulQuat(b3Body_GetRotation(world->bodies[a]), world->joint_local_a[j]);
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
        angle, qa.v.x, qa.v.y, qa.v.z, qa.s, rel.v.x, rel.v.y, rel.v.z, rel.s, w.x, w.y, w.z};
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

static void ymir_soa_free(ymir_box3d_soa_world* world)
{
    free(world->bodies);
    free(world->stable_id);
    free(world->kind);
    free(world->parent);
    free(world->limb);
    free(world->lanes);
    free(world->joints);
    free(world->joint_kind);
    free(world->joint_body_a);
    free(world->joint_body_b);
    free(world->joint_local_a);
    free(world->joint_local_b);
    free(world->joint_lanes);
    free(world);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_create(
    int32_t capacity, int32_t joint_capacity, float gravity_x, float gravity_y, float gravity_z,
    ymir_box3d_soa_world** output)
{
    if (output == NULL)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    *output = NULL;
    if (capacity <= 0 || joint_capacity < 0 || !ymir_soa_finite(gravity_x) || !ymir_soa_finite(gravity_y) ||
        !ymir_soa_finite(gravity_z))
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
    world->lanes = (float*)calloc(n * YMIR_SOA_BODY_FLOAT_LANES, sizeof(float));
    world->joints = (b3JointId*)calloc(m, sizeof(b3JointId));
    world->joint_kind = (uint32_t*)calloc(m, sizeof(uint32_t));
    world->joint_body_a = (int32_t*)calloc(m, sizeof(int32_t));
    world->joint_body_b = (int32_t*)calloc(m, sizeof(int32_t));
    world->joint_local_a = (b3Quat*)calloc(m, sizeof(b3Quat));
    world->joint_local_b = (b3Quat*)calloc(m, sizeof(b3Quat));
    world->joint_lanes = (float*)calloc(m * YMIR_SOA_JOINT_FLOAT_LANES, sizeof(float));
    if (world->bodies == NULL || world->stable_id == NULL || world->kind == NULL || world->parent == NULL ||
        world->limb == NULL || world->lanes == NULL || world->joints == NULL || world->joint_kind == NULL ||
        world->joint_body_a == NULL || world->joint_body_b == NULL || world->joint_local_a == NULL ||
        world->joint_local_b == NULL || world->joint_lanes == NULL)
    {
        ymir_soa_free(world);
        return YMIR_SOA_OUT_OF_MEMORY;
    }

    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = (b3Vec3){gravity_x, gravity_y, gravity_z};
    def.enableSleep = false;
    def.workerCount = 1; // one worker: the step order is fixed, which determinism rests on
    world->world_id = b3CreateWorld(&def);
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
        b3DestroyWorld(world->world_id);
    }
    ymir_soa_free(world);
}

static int32_t ymir_soa_check_body(const ymir_box3d_soa_world* world, const ymir_box3d_soa_body_desc* d)
{
    if (d->kind > YMIR_SOA_KIND_BOX || !ymir_soa_finite3(d->position) || !ymir_soa_finite3(d->velocity) ||
        !ymir_soa_finite3(d->size) || !ymir_soa_finite(d->friction) || !ymir_soa_finite(d->restitution) ||
        !ymir_soa_finite(d->density) || d->friction < 0.0f || d->restitution < 0.0f)
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
    body_def.type = ymir_soa_is_dynamic(d->kind) ? b3_dynamicBody : b3_staticBody;
    body_def.position = (b3Vec3){d->position[0], d->position[1], d->position[2]};
    body_def.rotation = (b3Quat){{d->rotation[0], d->rotation[1], d->rotation[2]}, d->rotation[3]};
    body_def.linearVelocity = (b3Vec3){d->velocity[0], d->velocity[1], d->velocity[2]};
    body_def.enableSleep = false;
    b3BodyId body = b3CreateBody(world->world_id, &body_def);
    if (B3_IS_NULL(body))
    {
        return YMIR_SOA_INTERNAL_ERROR;
    }
    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.userData = (void*)(uintptr_t)d->stable_id;
    if (ymir_soa_is_dynamic(d->kind))
    {
        shape_def.density = d->density;
    }
    shape_def.baseMaterial.friction = d->friction;
    shape_def.baseMaterial.restitution = d->restitution;

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

    int i = world->count++;
    world->bodies[i] = body;
    world->stable_id[i] = d->stable_id;
    world->kind[i] = d->kind;
    world->parent[i] = d->parent;
    world->limb[i] = d->limb;
    ymir_soa_refresh(world, i);
    if (index != NULL)
    {
        *index = i;
    }
    return YMIR_SOA_OK;
}

static b3Transform ymir_soa_body_transform(b3BodyId body)
{
    b3Pos p = b3Body_GetPosition(body);
    b3Transform t = {{p.x, p.y, p.z}, b3Body_GetRotation(body)};
    return t;
}

// Creates a joint between two existing bodies. The joint frame is built in world space (origin at the
// anchor, z along the axis) and expressed in each body's local frame, so the creation pose is zero.
YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_add_joint(
    ymir_box3d_soa_world* world, const ymir_box3d_soa_joint_desc* d, int32_t* index)
{
    if (world == NULL || d == NULL || d->kind > YMIR_SOA_JOINT_WELD || d->body_a < 0 || d->body_a >= world->count ||
        d->body_b < 0 || d->body_b >= world->count || d->body_a == d->body_b || !ymir_soa_finite3(d->anchor) ||
        !ymir_soa_finite3(d->axis) || !ymir_soa_finite(d->lower) || !ymir_soa_finite(d->upper) ||
        !ymir_soa_finite(d->cone_angle) || d->cone_angle < 0.0f || d->cone_angle > B3_PI)
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    b3Vec3 axis = {d->axis[0], d->axis[1], d->axis[2]};
    float length = sqrtf(b3Dot(axis, axis));
    if (length < 1e-6f || (d->enable_limit != 0 && d->kind != YMIR_SOA_JOINT_WELD && d->lower > d->upper))
    {
        return YMIR_SOA_INVALID_ARGUMENT;
    }
    if (world->joint_count >= world->joint_capacity)
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

    int j = world->joint_count++;
    world->joints[j] = joint;
    world->joint_kind[j] = d->kind;
    world->joint_body_a[j] = d->body_a;
    world->joint_body_b[j] = d->body_b;
    world->joint_local_a[j] = local_a.q;
    world->joint_local_b[j] = local_b.q;
    ymir_soa_refresh_joint(world, j);
    if (index != NULL)
    {
        *index = j;
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
    for (int i = 0; i < world->count; ++i)
    {
        ymir_soa_refresh(world, i);
    }
    for (int j = 0; j < world->joint_count; ++j)
    {
        ymir_soa_refresh_joint(world, j);
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
    };
    return YMIR_SOA_OK;
}
