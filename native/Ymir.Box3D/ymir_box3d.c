#include <box3d/base.h>
#include <box3d/box3d.h>
#include <box3d/collision.h>
#include <box3d/constants.h>
#include <box3d/math_functions.h>
#include <box3d/types.h>

#if defined(_WIN32)
#define YMIR_EXPORT __declspec(dllexport)
#else
#define YMIR_EXPORT __attribute__((visibility("default")))
#endif

typedef struct ymir_box3d_overlap_result
{
    int hit;
    float distance;
    float normal_x;
    float normal_y;
} ymir_box3d_overlap_result;

typedef struct ymir_box3d_cast_result
{
    int hit;
    float fraction;
    float point_x;
    float point_y;
    float normal_x;
    float normal_y;
} ymir_box3d_cast_result;

typedef struct ymir_box3d_body_input
{
    float position_x;
    float position_y;
    float velocity_x;
    float velocity_y;
    float radius;
    float mass;
    float restitution;
    int is_static;
} ymir_box3d_body_input;

typedef struct ymir_box3d_body_output
{
    float position_x;
    float position_y;
    float velocity_x;
    float velocity_y;
    float angular_velocity;
} ymir_box3d_body_output;

typedef struct ymir_box3d_pair_step_result
{
    ymir_box3d_body_output body_a;
    ymir_box3d_body_output body_b;
    int begin_contact_count;
    int end_contact_count;
    int hit_contact_count;
} ymir_box3d_pair_step_result;

typedef struct ymir_box3d_torque_result
{
    float angular_velocity_after_applied_step;
    float angular_velocity_after_unforced_step;
} ymir_box3d_torque_result;

static b3Vec3 ymir_vec(float x, float y)
{
    return (b3Vec3){x, 0.0f, y};
}

static b3BodyId ymir_create_body(b3WorldId world_id, const ymir_box3d_body_input* input)
{
    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = input->is_static != 0 ? b3_staticBody : b3_dynamicBody;
    body_def.position = ymir_vec(input->position_x, input->position_y);
    body_def.linearVelocity = ymir_vec(input->velocity_x, input->velocity_y);
    body_def.motionLocks.linearY = true;
    body_def.motionLocks.angularX = true;
    body_def.motionLocks.angularZ = true;
    body_def.enableSleep = false;
    b3BodyId body_id = b3CreateBody(world_id, &body_def);

    b3ShapeDef shape_def = b3DefaultShapeDef();
    if (input->is_static == 0)
    {
        float volume = 4.0f / 3.0f * B3_PI * input->radius * input->radius * input->radius;
        shape_def.density = input->mass / volume;
    }
    shape_def.baseMaterial.friction = 0.0f;
    shape_def.baseMaterial.restitution = input->restitution;
    shape_def.enableContactEvents = true;
    shape_def.enableHitEvents = true;
    b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, input->radius};
    b3CreateSphereShape(body_id, &shape_def, &sphere);
    return body_id;
}

static ymir_box3d_body_output ymir_read_body(b3BodyId body_id)
{
    b3Pos position = b3Body_GetPosition(body_id);
    b3Vec3 velocity = b3Body_GetLinearVelocity(body_id);
    b3Vec3 angular_velocity = b3Body_GetAngularVelocity(body_id);
    return (ymir_box3d_body_output){
        .position_x = position.x,
        .position_y = position.z,
        .velocity_x = velocity.x,
        .velocity_y = velocity.z,
        .angular_velocity = angular_velocity.y,
    };
}

YMIR_EXPORT int ymir_box3d_get_version(int* major, int* minor, int* revision)
{
    b3Version version = b3GetVersion();
    *major = version.major;
    *minor = version.minor;
    *revision = version.revision;
    return b3IsDoublePrecision() ? 1 : 0;
}

YMIR_EXPORT ymir_box3d_overlap_result ymir_box3d_overlap_spheres(
    float query_x,
    float query_y,
    float query_radius,
    float body_x,
    float body_y,
    float body_radius)
{
    b3Sphere body = {ymir_vec(body_x, body_y), body_radius};
    b3Vec3 query_point = ymir_vec(query_x, query_y);
    b3ShapeProxy query = {&query_point, 1, query_radius};

    b3DistanceInput input = {
        .proxyA = (b3ShapeProxy){&body.center, 1, body.radius},
        .proxyB = query,
        .transform = b3Transform_identity,
        .useRadii = true,
    };
    b3SimplexCache cache = {0};
    b3DistanceOutput distance = b3ShapeDistance(&input, &cache, NULL, 0);

    return (ymir_box3d_overlap_result){
        .hit = b3OverlapSphere(&body, b3Transform_identity, &query) ? 1 : 0,
        .distance = distance.distance,
        .normal_x = distance.normal.x,
        .normal_y = distance.normal.z,
    };
}

YMIR_EXPORT ymir_box3d_overlap_result ymir_box3d_overlap_capsule_sphere(
    float start_x,
    float start_y,
    float end_x,
    float end_y,
    float capsule_radius,
    float body_x,
    float body_y,
    float body_radius)
{
    b3Sphere body = {ymir_vec(body_x, body_y), body_radius};
    b3Vec3 capsule_points[2] = {ymir_vec(start_x, start_y), ymir_vec(end_x, end_y)};
    b3ShapeProxy capsule = {capsule_points, 2, capsule_radius};

    b3DistanceInput input = {
        .proxyA = (b3ShapeProxy){&body.center, 1, body.radius},
        .proxyB = capsule,
        .transform = b3Transform_identity,
        .useRadii = true,
    };
    b3SimplexCache cache = {0};
    b3DistanceOutput distance = b3ShapeDistance(&input, &cache, NULL, 0);

    return (ymir_box3d_overlap_result){
        .hit = b3OverlapSphere(&body, b3Transform_identity, &capsule) ? 1 : 0,
        .distance = distance.distance,
        .normal_x = distance.normal.x,
        .normal_y = distance.normal.z,
    };
}

YMIR_EXPORT ymir_box3d_cast_result ymir_box3d_cast_sphere(
    float origin_x,
    float origin_y,
    float translation_x,
    float translation_y,
    float query_radius,
    float body_x,
    float body_y,
    float body_radius,
    int can_encroach)
{
    b3Sphere body = {ymir_vec(body_x, body_y), body_radius};
    b3Vec3 query_point = ymir_vec(origin_x, origin_y);
    b3ShapeCastInput input = {
        .proxy = (b3ShapeProxy){&query_point, 1, query_radius},
        .translation = ymir_vec(translation_x, translation_y),
        .maxFraction = 1.0f,
        .canEncroach = can_encroach != 0,
    };
    b3CastOutput output = b3ShapeCastSphere(&body, &input);

    return (ymir_box3d_cast_result){
        .hit = output.hit ? 1 : 0,
        .fraction = output.fraction,
        .point_x = output.point.x,
        .point_y = output.point.z,
        .normal_x = output.normal.x,
        .normal_y = output.normal.z,
    };
}

YMIR_EXPORT void ymir_box3d_step_pair(
    const ymir_box3d_body_input* body_a_input,
    const ymir_box3d_body_input* body_b_input,
    float time_step,
    int sub_step_count,
    float restitution_threshold,
    ymir_box3d_pair_step_result* result)
{
    b3WorldDef world_def = b3DefaultWorldDef();
    world_def.gravity = b3Vec3_zero;
    world_def.enableSleep = false;
    world_def.restitutionThreshold = restitution_threshold;
    b3WorldId world_id = b3CreateWorld(&world_def);

    b3BodyId body_a = ymir_create_body(world_id, body_a_input);
    b3BodyId body_b = ymir_create_body(world_id, body_b_input);
    b3World_Step(world_id, time_step, sub_step_count);

    b3ContactEvents events = b3World_GetContactEvents(world_id);
    result->body_a = ymir_read_body(body_a);
    result->body_b = ymir_read_body(body_b);
    result->begin_contact_count = events.beginCount;
    result->end_contact_count = events.endCount;
    result->hit_contact_count = events.hitCount;

    b3DestroyWorld(world_id);
}

YMIR_EXPORT ymir_box3d_torque_result ymir_box3d_torque_lifetime(float torque, float time_step, int sub_step_count)
{
    b3WorldDef world_def = b3DefaultWorldDef();
    world_def.gravity = b3Vec3_zero;
    world_def.enableSleep = false;
    b3WorldId world_id = b3CreateWorld(&world_def);

    ymir_box3d_body_input input = {
        .radius = 1.0f,
        .mass = 1.0f,
        .is_static = 0,
    };
    b3BodyId body_id = ymir_create_body(world_id, &input);
    b3Body_ApplyTorque(body_id, (b3Vec3){0.0f, torque, 0.0f}, true);
    b3World_Step(world_id, time_step, sub_step_count);
    float after_applied_step = b3Body_GetAngularVelocity(body_id).y;

    b3World_Step(world_id, time_step, sub_step_count);
    float after_unforced_step = b3Body_GetAngularVelocity(body_id).y;
    b3DestroyWorld(world_id);

    return (ymir_box3d_torque_result){after_applied_step, after_unforced_step};
}
