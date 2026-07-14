#include <box3d/base.h>
#include <box3d/box3d.h>
#include <box3d/collision.h>
#include <box3d/constants.h>
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

typedef enum ymir_box3d_status
{
    YMIR_BOX3D_OK = 0,
    YMIR_BOX3D_INVALID_ARGUMENT = 1,
    YMIR_BOX3D_DUPLICATE_ID = 2,
    YMIR_BOX3D_BUFFER_TOO_SMALL = 3,
    YMIR_BOX3D_OUT_OF_MEMORY = 4,
    YMIR_BOX3D_INTERNAL_ERROR = 5,
} ymir_box3d_status;

typedef struct ymir_box3d_session_body_input
{
    uint64_t stable_id;
    uint64_t collision_category_bits;
    uint64_t collision_mask_bits;
    float position_x;
    float position_y;
    float velocity_x;
    float velocity_y;
    float direction_x;
    float direction_y;
    float angular_velocity;
    float torque;
    float radius;
    float mass;
    float restitution;
    uint32_t is_static;
    uint32_t is_kinematic;
    uint32_t is_bullet;
    uint32_t participates_in_fields;
    int32_t collision_group_index;
} ymir_box3d_session_body_input;

typedef struct ymir_box3d_radial_field_input
{
    float position_x;
    float position_y;
    float strength;
    float radius;
} ymir_box3d_radial_field_input;

typedef struct ymir_box3d_session_body_output
{
    uint64_t stable_id;
    float position_x;
    float position_y;
    float velocity_x;
    float velocity_y;
    float direction_x;
    float direction_y;
    float angular_velocity;
} ymir_box3d_session_body_output;

typedef struct ymir_box3d_session_contact_output
{
    uint64_t stable_id_a;
    uint64_t stable_id_b;
    float point_x;
    float point_y;
    float normal_x;
    float normal_y;
    float penetration;
    float relative_speed;
} ymir_box3d_session_contact_output;

typedef enum ymir_box3d_contact_kind
{
    YMIR_BOX3D_CONTACT_BEGIN = 1,
    YMIR_BOX3D_CONTACT_HIT = 2,
    YMIR_BOX3D_CONTACT_END = 3,
} ymir_box3d_contact_kind;

typedef struct ymir_box3d_session_contact_event_output
{
    int32_t kind;
    uint32_t has_details;
    uint32_t contact_id[3];
    uint64_t stable_id_a;
    uint64_t stable_id_b;
    float point_x;
    float point_y;
    float normal_x;
    float normal_y;
    float penetration;
    float relative_speed;
} ymir_box3d_session_contact_event_output;

typedef struct ymir_box3d_retired_shape
{
    uint64_t shape_id;
    uint64_t stable_id;
} ymir_box3d_retired_shape;

typedef struct ymir_box3d_drained_event
{
    int32_t kind;
    uint32_t contact_id[3];
} ymir_box3d_drained_event;

typedef struct ymir_box3d_session_body
{
    uint64_t stable_id;
    b3BodyId body_id;
    b3ShapeId shape_id;
    float last_position_x;
    float last_position_y;
    float last_velocity_x;
    float last_velocity_y;
    float last_direction_x;
    float last_direction_y;
    float last_angular_velocity;
    float pending_torque;
    float radius;
    float mass;
    float restitution;
    uint32_t is_static;
    uint32_t is_kinematic;
    uint32_t is_bullet;
    uint32_t participates_in_fields;
    uint64_t collision_category_bits;
    uint64_t collision_mask_bits;
    int32_t collision_group_index;
    int seen;
} ymir_box3d_session_body;

typedef struct ymir_box3d_session
{
    b3WorldId world_id;
    ymir_box3d_session_body* bodies;
    int body_count;
    int body_capacity;
    ymir_box3d_session_contact_event_output* contact_events;
    int contact_event_count;
    int contact_event_capacity;
    ymir_box3d_session_contact_event_output* pending_events;
    int pending_event_count;
    int pending_event_capacity;
    ymir_box3d_retired_shape* retired_shapes;
    int retired_shape_count;
    int retired_shape_capacity;
    ymir_box3d_drained_event* drained_events;
    int drained_event_count;
    int drained_event_capacity;
} ymir_box3d_session;

_Static_assert(sizeof(ymir_box3d_status) == sizeof(int32_t), "Ymir Box3D status ABI must be 32-bit");
_Static_assert(sizeof(ymir_box3d_session_body_input) == 88, "Unexpected Ymir Box3D body input layout");
_Static_assert(sizeof(ymir_box3d_radial_field_input) == 16, "Unexpected Ymir Box3D field layout");
_Static_assert(sizeof(ymir_box3d_session_body_output) == 40, "Unexpected Ymir Box3D body output layout");
_Static_assert(sizeof(ymir_box3d_session_contact_output) == 40, "Unexpected Ymir Box3D contact layout");
_Static_assert(sizeof(ymir_box3d_session_contact_event_output) == 64, "Unexpected Ymir Box3D contact event layout");

static b3Vec3 ymir_vec(float x, float y)
{
    return (b3Vec3){x, 0.0f, y};
}

static int ymir_is_finite(float value)
{
    return isfinite(value) != 0;
}

static int ymir_session_body_input_is_valid(const ymir_box3d_session_body_input* input)
{
    float direction_length_squared = input != NULL
        ? input->direction_x * input->direction_x + input->direction_y * input->direction_y
        : 0.0f;
    return input != NULL && input->stable_id != 0 && input->stable_id <= (uint64_t)UINTPTR_MAX &&
           ymir_is_finite(input->position_x) && ymir_is_finite(input->position_y) &&
           ymir_is_finite(input->velocity_x) && ymir_is_finite(input->velocity_y) &&
           ymir_is_finite(input->direction_x) && ymir_is_finite(input->direction_y) &&
           direction_length_squared > 1.0e-8f && ymir_is_finite(input->angular_velocity) &&
           ymir_is_finite(input->torque) &&
           ymir_is_finite(input->radius) && input->radius > 0.0f && ymir_is_finite(input->mass) &&
           (input->is_static != 0 || input->mass > 0.0f) && ymir_is_finite(input->restitution) &&
           input->restitution >= 0.0f && !(input->is_static != 0 && input->is_kinematic != 0) &&
           input->collision_category_bits != 0;
}

static b3BodyType ymir_session_body_type(const ymir_box3d_session_body_input* input)
{
    if (input->is_static != 0)
    {
        return b3_staticBody;
    }
    return input->is_kinematic != 0 ? b3_kinematicBody : b3_dynamicBody;
}

static b3Quat ymir_session_rotation(float direction_x, float direction_y)
{
    float angle = atan2f(direction_x, direction_y);
    return b3MakeQuatFromAxisAngle((b3Vec3){0.0f, 1.0f, 0.0f}, angle);
}

static b3Vec3 ymir_session_direction(b3BodyId body_id)
{
    return b3RotateVector(b3Body_GetRotation(body_id), (b3Vec3){0.0f, 0.0f, 1.0f});
}

static int ymir_session_find_body(const ymir_box3d_session* session, uint64_t stable_id)
{
    for (int i = 0; i < session->body_count; ++i)
    {
        if (session->bodies[i].stable_id == stable_id)
        {
            return i;
        }
    }

    return -1;
}

static ymir_box3d_status ymir_session_reserve_bodies(ymir_box3d_session* session, int capacity)
{
    if (capacity <= session->body_capacity)
    {
        return YMIR_BOX3D_OK;
    }

    int new_capacity = session->body_capacity > 0 ? session->body_capacity : 16;
    while (new_capacity < capacity)
    {
        if (new_capacity > INT32_MAX / 2)
        {
            new_capacity = capacity;
            break;
        }
        new_capacity *= 2;
    }

    ymir_box3d_session_body* bodies =
        (ymir_box3d_session_body*)realloc(session->bodies, (size_t)new_capacity * sizeof(ymir_box3d_session_body));
    if (bodies == NULL)
    {
        return YMIR_BOX3D_OUT_OF_MEMORY;
    }

    session->bodies = bodies;
    session->body_capacity = new_capacity;
    return YMIR_BOX3D_OK;
}

static ymir_box3d_status ymir_session_reserve_contact_events(
    ymir_box3d_session_contact_event_output** events,
    int* current_capacity,
    int capacity)
{
    if (capacity <= *current_capacity)
    {
        return YMIR_BOX3D_OK;
    }

    int new_capacity = *current_capacity > 0 ? *current_capacity : 16;
    while (new_capacity < capacity)
    {
        new_capacity *= 2;
    }
    ymir_box3d_session_contact_event_output* resized = (ymir_box3d_session_contact_event_output*)realloc(
        *events, (size_t)new_capacity * sizeof(ymir_box3d_session_contact_event_output));
    if (resized == NULL)
    {
        return YMIR_BOX3D_OUT_OF_MEMORY;
    }
    *events = resized;
    *current_capacity = new_capacity;
    return YMIR_BOX3D_OK;
}

static ymir_box3d_status ymir_session_reserve_retired_shapes(ymir_box3d_session* session, int capacity)
{
    if (capacity <= session->retired_shape_capacity)
    {
        return YMIR_BOX3D_OK;
    }
    int new_capacity = session->retired_shape_capacity > 0 ? session->retired_shape_capacity * 2 : 16;
    while (new_capacity < capacity)
    {
        new_capacity *= 2;
    }
    ymir_box3d_retired_shape* resized = (ymir_box3d_retired_shape*)realloc(
        session->retired_shapes, (size_t)new_capacity * sizeof(ymir_box3d_retired_shape));
    if (resized == NULL)
    {
        return YMIR_BOX3D_OUT_OF_MEMORY;
    }
    session->retired_shapes = resized;
    session->retired_shape_capacity = new_capacity;
    return YMIR_BOX3D_OK;
}

static ymir_box3d_status ymir_session_reserve_drained_events(ymir_box3d_session* session, int capacity)
{
    if (capacity <= session->drained_event_capacity)
    {
        return YMIR_BOX3D_OK;
    }
    int new_capacity = session->drained_event_capacity > 0 ? session->drained_event_capacity * 2 : 16;
    while (new_capacity < capacity)
    {
        new_capacity *= 2;
    }
    ymir_box3d_drained_event* resized = (ymir_box3d_drained_event*)realloc(
        session->drained_events, (size_t)new_capacity * sizeof(ymir_box3d_drained_event));
    if (resized == NULL)
    {
        return YMIR_BOX3D_OUT_OF_MEMORY;
    }
    session->drained_events = resized;
    session->drained_event_capacity = new_capacity;
    return YMIR_BOX3D_OK;
}

static float ymir_session_density(float mass, float radius)
{
    float volume = 4.0f / 3.0f * B3_PI * radius * radius * radius;
    return mass / volume;
}

static ymir_box3d_status ymir_session_create_body(
    ymir_box3d_session* session,
    const ymir_box3d_session_body_input* input,
    ymir_box3d_session_body* output)
{
    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = ymir_session_body_type(input);
    body_def.position = ymir_vec(input->position_x, input->position_y);
    body_def.rotation = ymir_session_rotation(input->direction_x, input->direction_y);
    body_def.linearVelocity = ymir_vec(input->velocity_x, input->velocity_y);
    body_def.angularVelocity = (b3Vec3){0.0f, input->angular_velocity, 0.0f};
    body_def.motionLocks.linearY = true;
    body_def.motionLocks.angularX = true;
    body_def.motionLocks.angularZ = true;
    body_def.enableSleep = false;
    body_def.isBullet = input->is_bullet != 0;
    b3BodyId body_id = b3CreateBody(session->world_id, &body_def);
    if (B3_IS_NULL(body_id))
    {
        return YMIR_BOX3D_INTERNAL_ERROR;
    }

    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.userData = (void*)(uintptr_t)input->stable_id;
    shape_def.density = input->is_static != 0 ? 0.0f : ymir_session_density(input->mass, input->radius);
    shape_def.baseMaterial.friction = 0.0f;
    shape_def.baseMaterial.restitution = input->restitution;
    shape_def.filter.categoryBits = input->collision_category_bits;
    shape_def.filter.maskBits = input->collision_mask_bits;
    shape_def.filter.groupIndex = input->collision_group_index;
    shape_def.enableContactEvents = true;
    shape_def.enableHitEvents = true;
    b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, input->radius};
    b3ShapeId shape_id = b3CreateSphereShape(body_id, &shape_def, &sphere);
    if (B3_IS_NULL(shape_id))
    {
        b3DestroyBody(body_id);
        return YMIR_BOX3D_INTERNAL_ERROR;
    }

    *output = (ymir_box3d_session_body){
        .stable_id = input->stable_id,
        .body_id = body_id,
        .shape_id = shape_id,
        .last_position_x = input->position_x,
        .last_position_y = input->position_y,
        .last_velocity_x = input->velocity_x,
        .last_velocity_y = input->velocity_y,
        .last_direction_x = input->direction_x,
        .last_direction_y = input->direction_y,
        .last_angular_velocity = input->angular_velocity,
        .pending_torque = input->torque,
        .radius = input->radius,
        .mass = input->mass,
        .restitution = input->restitution,
        .is_static = input->is_static != 0,
        .is_kinematic = input->is_kinematic != 0,
        .is_bullet = input->is_bullet != 0,
        .participates_in_fields = input->participates_in_fields != 0,
        .collision_category_bits = input->collision_category_bits,
        .collision_mask_bits = input->collision_mask_bits,
        .collision_group_index = input->collision_group_index,
        .seen = 1,
    };
    return YMIR_BOX3D_OK;
}

static void ymir_session_update_body(
    ymir_box3d_session_body* body,
    const ymir_box3d_session_body_input* input)
{
    int type_changed = body->is_static != (input->is_static != 0) ||
        body->is_kinematic != (input->is_kinematic != 0);
    int radius_changed = body->radius != input->radius;
    int mass_changed = body->mass != input->mass;

    if (type_changed)
    {
        b3Body_SetType(body->body_id, ymir_session_body_type(input));
    }

    if (body->is_bullet != (input->is_bullet != 0))
    {
        b3Body_SetBullet(body->body_id, input->is_bullet != 0);
    }

    if (body->collision_category_bits != input->collision_category_bits ||
        body->collision_mask_bits != input->collision_mask_bits ||
        body->collision_group_index != input->collision_group_index)
    {
        b3Filter filter = {
            .categoryBits = input->collision_category_bits,
            .maskBits = input->collision_mask_bits,
            .groupIndex = input->collision_group_index,
        };
        b3Shape_SetFilter(body->shape_id, filter, true);
    }

    if (radius_changed)
    {
        b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, input->radius};
        b3Shape_SetSphere(body->shape_id, &sphere);
    }

    if (type_changed || radius_changed || mass_changed)
    {
        float density = input->is_static != 0 ? 0.0f : ymir_session_density(input->mass, input->radius);
        b3Shape_SetDensity(body->shape_id, density, false);
        b3Body_ApplyMassFromShapes(body->body_id);
    }

    if (body->restitution != input->restitution)
    {
        b3Shape_SetRestitution(body->shape_id, input->restitution);
    }

    if (body->last_position_x != input->position_x || body->last_position_y != input->position_y ||
        body->last_direction_x != input->direction_x || body->last_direction_y != input->direction_y)
    {
        b3Body_SetTransform(
            body->body_id,
            ymir_vec(input->position_x, input->position_y),
            ymir_session_rotation(input->direction_x, input->direction_y));
    }

    if (type_changed || body->last_velocity_x != input->velocity_x || body->last_velocity_y != input->velocity_y)
    {
        b3Body_SetLinearVelocity(body->body_id, ymir_vec(input->velocity_x, input->velocity_y));
    }

    if (type_changed || body->last_angular_velocity != input->angular_velocity)
    {
        b3Body_SetAngularVelocity(body->body_id, (b3Vec3){0.0f, input->angular_velocity, 0.0f});
    }

    body->last_position_x = input->position_x;
    body->last_position_y = input->position_y;
    body->last_velocity_x = input->velocity_x;
    body->last_velocity_y = input->velocity_y;
    body->last_direction_x = input->direction_x;
    body->last_direction_y = input->direction_y;
    body->last_angular_velocity = input->angular_velocity;
    body->pending_torque = input->torque;
    body->radius = input->radius;
    body->mass = input->mass;
    body->restitution = input->restitution;
    body->is_static = input->is_static != 0;
    body->is_kinematic = input->is_kinematic != 0;
    body->is_bullet = input->is_bullet != 0;
    body->participates_in_fields = input->participates_in_fields != 0;
    body->collision_category_bits = input->collision_category_bits;
    body->collision_mask_bits = input->collision_mask_bits;
    body->collision_group_index = input->collision_group_index;
    body->seen = 1;
}

static uint64_t ymir_session_shape_stable_id(b3ShapeId shape_id)
{
    return (uint64_t)(uintptr_t)b3Shape_GetUserData(shape_id);
}

typedef struct ymir_box3d_stable_pair
{
    uint64_t first;
    uint64_t second;
} ymir_box3d_stable_pair;

typedef struct ymir_box3d_stable_pair_set
{
    ymir_box3d_stable_pair* pairs;
    int count;
    int capacity;
} ymir_box3d_stable_pair_set;

static ymir_box3d_status ymir_session_add_stable_pair(
    ymir_box3d_stable_pair_set* set,
    uint64_t stable_id_a,
    uint64_t stable_id_b,
    int* added)
{
    uint64_t first = stable_id_a < stable_id_b ? stable_id_a : stable_id_b;
    uint64_t second = stable_id_a < stable_id_b ? stable_id_b : stable_id_a;
    for (int i = 0; i < set->count; ++i)
    {
        if (set->pairs[i].first == first && set->pairs[i].second == second)
        {
            *added = 0;
            return YMIR_BOX3D_OK;
        }
    }

    if (set->count == set->capacity)
    {
        int new_capacity = set->capacity > 0 ? set->capacity * 2 : 16;
        ymir_box3d_stable_pair* pairs =
            (ymir_box3d_stable_pair*)realloc(set->pairs, (size_t)new_capacity * sizeof(ymir_box3d_stable_pair));
        if (pairs == NULL)
        {
            return YMIR_BOX3D_OUT_OF_MEMORY;
        }
        set->pairs = pairs;
        set->capacity = new_capacity;
    }

    set->pairs[set->count++] = (ymir_box3d_stable_pair){first, second};
    *added = 1;
    return YMIR_BOX3D_OK;
}

static int ymir_session_contact_from_manifolds(
    const b3ContactData* contact,
    ymir_box3d_session_contact_output* result)
{
    const b3Manifold* selected_manifold = NULL;
    const b3ManifoldPoint* selected_point = NULL;
    for (int manifold_index = 0; manifold_index < contact->manifoldCount; ++manifold_index)
    {
        const b3Manifold* manifold = contact->manifolds + manifold_index;
        for (int point_index = 0; point_index < manifold->pointCount; ++point_index)
        {
            const b3ManifoldPoint* point = manifold->points + point_index;
            if (selected_point == NULL || point->separation < selected_point->separation)
            {
                selected_manifold = manifold;
                selected_point = point;
            }
        }
    }

    if (selected_point == NULL)
    {
        return 0;
    }

    uint64_t stable_id_a = ymir_session_shape_stable_id(contact->shapeIdA);
    uint64_t stable_id_b = ymir_session_shape_stable_id(contact->shapeIdB);
    if (stable_id_a == 0 || stable_id_b == 0 || stable_id_a == stable_id_b)
    {
        return 0;
    }

    b3BodyId body_a = b3Shape_GetBody(contact->shapeIdA);
    b3BodyId body_b = b3Shape_GetBody(contact->shapeIdB);
    b3Pos center_a = b3Body_GetWorldCenterOfMass(body_a);
    b3Pos center_b = b3Body_GetWorldCenterOfMass(body_b);
    b3Pos point_a = {
        center_a.x + selected_point->anchorA.x,
        center_a.y + selected_point->anchorA.y,
        center_a.z + selected_point->anchorA.z,
    };
    b3Pos point_b = {
        center_b.x + selected_point->anchorB.x,
        center_b.y + selected_point->anchorB.y,
        center_b.z + selected_point->anchorB.z,
    };
    *result = (ymir_box3d_session_contact_output){
        .stable_id_a = stable_id_a,
        .stable_id_b = stable_id_b,
        .point_x = 0.5f * (point_a.x + point_b.x),
        .point_y = 0.5f * (point_a.z + point_b.z),
        .normal_x = selected_manifold->normal.x,
        .normal_y = selected_manifold->normal.z,
        .penetration = selected_point->separation < 0.0f ? -selected_point->separation : 0.0f,
        .relative_speed = selected_point->normalVelocity,
    };
    return 1;
}

static uint64_t ymir_session_resolve_event_shape(
    const ymir_box3d_session* session,
    b3ShapeId shape_id)
{
    if (b3Shape_IsValid(shape_id))
    {
        return ymir_session_shape_stable_id(shape_id);
    }

    uint64_t stored_shape_id = b3StoreShapeId(shape_id);
    for (int i = 0; i < session->retired_shape_count; ++i)
    {
        if (session->retired_shapes[i].shape_id == stored_shape_id)
        {
            return session->retired_shapes[i].stable_id;
        }
    }
    return 0;
}

static int ymir_session_event_was_drained(
    const ymir_box3d_session* session,
    int32_t kind,
    b3ContactId contact_id)
{
    uint32_t stored[3];
    b3StoreContactId(contact_id, stored);
    for (int i = 0; i < session->drained_event_count; ++i)
    {
        const ymir_box3d_drained_event* event = session->drained_events + i;
        if (event->kind == kind && event->contact_id[0] == stored[0] &&
            event->contact_id[1] == stored[1] && event->contact_id[2] == stored[2])
        {
            return 1;
        }
    }
    return 0;
}

static ymir_box3d_status ymir_session_mark_event_drained(
    ymir_box3d_session* session,
    int32_t kind,
    b3ContactId contact_id)
{
    ymir_box3d_status status = ymir_session_reserve_drained_events(session, session->drained_event_count + 1);
    if (status != YMIR_BOX3D_OK)
    {
        return status;
    }
    ymir_box3d_drained_event* stored = session->drained_events + session->drained_event_count++;
    stored->kind = kind;
    b3StoreContactId(contact_id, stored->contact_id);
    return YMIR_BOX3D_OK;
}

static ymir_box3d_status ymir_session_append_typed_event(
    ymir_box3d_session* session,
    int pending,
    const ymir_box3d_session_contact_event_output* event)
{
    ymir_box3d_session_contact_event_output** events = pending ? &session->pending_events : &session->contact_events;
    int* count = pending ? &session->pending_event_count : &session->contact_event_count;
    int* capacity = pending ? &session->pending_event_capacity : &session->contact_event_capacity;
    ymir_box3d_status status = ymir_session_reserve_contact_events(events, capacity, *count + 1);
    if (status != YMIR_BOX3D_OK)
    {
        return status;
    }
    (*events)[(*count)++] = *event;
    return YMIR_BOX3D_OK;
}

enum
{
    YMIR_DRAIN_BEGIN = 1,
    YMIR_DRAIN_HIT = 2,
    YMIR_DRAIN_END = 4,
};

static ymir_box3d_status ymir_session_drain_contact_events(
    ymir_box3d_session* session,
    int event_mask,
    int pending)
{
    b3ContactEvents events = b3World_GetContactEvents(session->world_id);
    if ((event_mask & YMIR_DRAIN_BEGIN) != 0)
    {
        for (int i = 0; i < events.beginCount; ++i)
        {
            const b3ContactBeginTouchEvent* source = events.beginEvents + i;
            if (ymir_session_event_was_drained(session, YMIR_BOX3D_CONTACT_BEGIN, source->contactId))
            {
                continue;
            }
            uint64_t stable_id_a = ymir_session_resolve_event_shape(session, source->shapeIdA);
            uint64_t stable_id_b = ymir_session_resolve_event_shape(session, source->shapeIdB);
            if (stable_id_a == 0 || stable_id_b == 0)
            {
                continue;
            }

            ymir_box3d_session_contact_event_output event = {
                .kind = YMIR_BOX3D_CONTACT_BEGIN,
                .stable_id_a = stable_id_a,
                .stable_id_b = stable_id_b,
            };
            b3StoreContactId(source->contactId, event.contact_id);
            if (b3Contact_IsValid(source->contactId))
            {
                b3ContactData data = b3Contact_GetData(source->contactId);
                ymir_box3d_session_contact_output details;
                if (ymir_session_contact_from_manifolds(&data, &details))
                {
                    event.has_details = 1;
                    event.point_x = details.point_x;
                    event.point_y = details.point_y;
                    event.normal_x = details.normal_x;
                    event.normal_y = details.normal_y;
                    event.penetration = details.penetration;
                    event.relative_speed = details.relative_speed;
                }
            }
            ymir_box3d_status status = ymir_session_append_typed_event(session, pending, &event);
            if (status != YMIR_BOX3D_OK)
            {
                return status;
            }
            status = ymir_session_mark_event_drained(session, YMIR_BOX3D_CONTACT_BEGIN, source->contactId);
            if (status != YMIR_BOX3D_OK)
            {
                return status;
            }
        }
    }

    if ((event_mask & YMIR_DRAIN_HIT) != 0)
    {
        for (int i = 0; i < events.hitCount; ++i)
        {
            const b3ContactHitEvent* source = events.hitEvents + i;
            if (ymir_session_event_was_drained(session, YMIR_BOX3D_CONTACT_HIT, source->contactId))
            {
                continue;
            }
            uint64_t stable_id_a = ymir_session_resolve_event_shape(session, source->shapeIdA);
            uint64_t stable_id_b = ymir_session_resolve_event_shape(session, source->shapeIdB);
            if (stable_id_a == 0 || stable_id_b == 0)
            {
                continue;
            }
            ymir_box3d_session_contact_event_output event = {
                .kind = YMIR_BOX3D_CONTACT_HIT,
                .has_details = 1,
                .stable_id_a = stable_id_a,
                .stable_id_b = stable_id_b,
                .point_x = source->point.x,
                .point_y = source->point.z,
                .normal_x = source->normal.x,
                .normal_y = source->normal.z,
                .relative_speed = -source->approachSpeed,
            };
            b3StoreContactId(source->contactId, event.contact_id);
            ymir_box3d_status status = ymir_session_append_typed_event(session, pending, &event);
            if (status != YMIR_BOX3D_OK)
            {
                return status;
            }
            status = ymir_session_mark_event_drained(session, YMIR_BOX3D_CONTACT_HIT, source->contactId);
            if (status != YMIR_BOX3D_OK)
            {
                return status;
            }
        }
    }

    if ((event_mask & YMIR_DRAIN_END) != 0)
    {
        for (int i = 0; i < events.endCount; ++i)
        {
            const b3ContactEndTouchEvent* source = events.endEvents + i;
            if (ymir_session_event_was_drained(session, YMIR_BOX3D_CONTACT_END, source->contactId))
            {
                continue;
            }
            uint64_t stable_id_a = ymir_session_resolve_event_shape(session, source->shapeIdA);
            uint64_t stable_id_b = ymir_session_resolve_event_shape(session, source->shapeIdB);
            if (stable_id_a == 0 || stable_id_b == 0)
            {
                continue;
            }
            ymir_box3d_session_contact_event_output event = {
                .kind = YMIR_BOX3D_CONTACT_END,
                .stable_id_a = stable_id_a,
                .stable_id_b = stable_id_b,
            };
            b3StoreContactId(source->contactId, event.contact_id);
            ymir_box3d_status status = ymir_session_append_typed_event(session, pending, &event);
            if (status != YMIR_BOX3D_OK)
            {
                return status;
            }
            status = ymir_session_mark_event_drained(session, YMIR_BOX3D_CONTACT_END, source->contactId);
            if (status != YMIR_BOX3D_OK)
            {
                return status;
            }
        }
    }
    return YMIR_BOX3D_OK;
}

static ymir_box3d_status ymir_session_emit_contact(
    ymir_box3d_stable_pair_set* seen,
    const ymir_box3d_session_contact_output* contact,
    ymir_box3d_session_contact_output* output,
    int capacity,
    int* total)
{
    int added = 0;
    ymir_box3d_status status =
        ymir_session_add_stable_pair(seen, contact->stable_id_a, contact->stable_id_b, &added);
    if (status != YMIR_BOX3D_OK || !added)
    {
        return status;
    }

    if (output != NULL)
    {
        if (*total >= capacity)
        {
            return YMIR_BOX3D_BUFFER_TOO_SMALL;
        }
        output[*total] = *contact;
    }
    *total += 1;
    return YMIR_BOX3D_OK;
}

static ymir_box3d_status ymir_session_visit_contacts(
    const ymir_box3d_session* session,
    ymir_box3d_session_contact_output* output,
    int capacity,
    int* count)
{
    int total = 0;
    ymir_box3d_stable_pair_set seen = {0};
    for (int body_index = 0; body_index < session->body_count; ++body_index)
    {
        b3ShapeId source_shape = session->bodies[body_index].shape_id;
        int contact_capacity = b3Shape_GetContactCapacity(source_shape);
        if (contact_capacity <= 0)
        {
            continue;
        }

        b3ContactData* contacts = (b3ContactData*)malloc((size_t)contact_capacity * sizeof(b3ContactData));
        if (contacts == NULL)
        {
            free(seen.pairs);
            return YMIR_BOX3D_OUT_OF_MEMORY;
        }

        int contact_count = b3Shape_GetContactData(source_shape, contacts, contact_capacity);
        for (int contact_index = 0; contact_index < contact_count; ++contact_index)
        {
            const b3ContactData* contact = contacts + contact_index;
            uint64_t stable_id_a = ymir_session_shape_stable_id(contact->shapeIdA);
            uint64_t stable_id_b = ymir_session_shape_stable_id(contact->shapeIdB);
            uint64_t source_stable_id = session->bodies[body_index].stable_id;
            uint64_t first_stable_id = stable_id_a < stable_id_b ? stable_id_a : stable_id_b;
            if (stable_id_a == 0 || stable_id_b == 0 || source_stable_id != first_stable_id)
            {
                continue;
            }

            ymir_box3d_session_contact_output candidate;
            if (ymir_session_contact_from_manifolds(contact, &candidate))
            {
                ymir_box3d_status status =
                    ymir_session_emit_contact(&seen, &candidate, output, capacity, &total);
                if (status != YMIR_BOX3D_OK)
                {
                    free(contacts);
                    free(seen.pairs);
                    return status;
                }
            }
        }
        free(contacts);
    }

    b3ContactEvents events = b3World_GetContactEvents(session->world_id);
    for (int event_index = 0; event_index < events.hitCount; ++event_index)
    {
        const b3ContactHitEvent* event = events.hitEvents + event_index;
        if (!b3Shape_IsValid(event->shapeIdA) || !b3Shape_IsValid(event->shapeIdB))
        {
            continue;
        }

        uint64_t stable_id_a = ymir_session_shape_stable_id(event->shapeIdA);
        uint64_t stable_id_b = ymir_session_shape_stable_id(event->shapeIdB);
        if (stable_id_a == 0 || stable_id_b == 0 || stable_id_a == stable_id_b)
        {
            continue;
        }

        ymir_box3d_session_contact_output candidate = {
            .stable_id_a = stable_id_a,
            .stable_id_b = stable_id_b,
            .point_x = event->point.x,
            .point_y = event->point.z,
            .normal_x = event->normal.x,
            .normal_y = event->normal.z,
            .penetration = 0.0f,
            .relative_speed = -event->approachSpeed,
        };
        ymir_box3d_status status = ymir_session_emit_contact(&seen, &candidate, output, capacity, &total);
        if (status != YMIR_BOX3D_OK)
        {
            free(seen.pairs);
            return status;
        }
    }

    for (int event_index = 0; event_index < events.beginCount; ++event_index)
    {
        const b3ContactBeginTouchEvent* event = events.beginEvents + event_index;
        if (!b3Contact_IsValid(event->contactId))
        {
            continue;
        }

        b3ContactData contact = b3Contact_GetData(event->contactId);
        ymir_box3d_session_contact_output candidate;
        if (!ymir_session_contact_from_manifolds(&contact, &candidate))
        {
            continue;
        }

        ymir_box3d_status status = ymir_session_emit_contact(&seen, &candidate, output, capacity, &total);
        if (status != YMIR_BOX3D_OK)
        {
            free(seen.pairs);
            return status;
        }
    }

    free(seen.pairs);
    *count = total;
    return YMIR_BOX3D_OK;
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

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_get_version(int32_t* major, int32_t* minor, int32_t* revision)
{
    b3Version version = b3GetVersion();
    *major = version.major;
    *minor = version.minor;
    *revision = version.revision;
    return b3IsDoublePrecision() ? 1 : 0;
}

YMIR_EXPORT ymir_box3d_overlap_result YMIR_CALL ymir_box3d_overlap_spheres(
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

YMIR_EXPORT ymir_box3d_overlap_result YMIR_CALL ymir_box3d_overlap_capsule_sphere(
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

YMIR_EXPORT ymir_box3d_cast_result YMIR_CALL ymir_box3d_cast_sphere(
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

YMIR_EXPORT void YMIR_CALL ymir_box3d_step_pair(
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

YMIR_EXPORT ymir_box3d_torque_result YMIR_CALL ymir_box3d_torque_lifetime(float torque, float time_step, int32_t sub_step_count)
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

YMIR_EXPORT uint32_t YMIR_CALL ymir_box3d_get_abi_version(void)
{
    return 4;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_get_abi_layout(
    uint32_t* body_input_size,
    uint32_t* field_input_size,
    uint32_t* body_output_size,
    uint32_t* contact_output_size,
    uint32_t* contact_event_output_size)
{
    if (body_input_size == NULL || field_input_size == NULL || body_output_size == NULL ||
        contact_output_size == NULL || contact_event_output_size == NULL)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }

    *body_input_size = (uint32_t)sizeof(ymir_box3d_session_body_input);
    *field_input_size = (uint32_t)sizeof(ymir_box3d_radial_field_input);
    *body_output_size = (uint32_t)sizeof(ymir_box3d_session_body_output);
    *contact_output_size = (uint32_t)sizeof(ymir_box3d_session_contact_output);
    *contact_event_output_size = (uint32_t)sizeof(ymir_box3d_session_contact_event_output);
    return YMIR_BOX3D_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_create(ymir_box3d_session** output)
{
    if (output == NULL)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    *output = NULL;

    ymir_box3d_session* session = (ymir_box3d_session*)calloc(1, sizeof(ymir_box3d_session));
    if (session == NULL)
    {
        return YMIR_BOX3D_OUT_OF_MEMORY;
    }

    b3WorldDef world_def = b3DefaultWorldDef();
    world_def.gravity = b3Vec3_zero;
    world_def.enableSleep = false;
    world_def.workerCount = 1;
    session->world_id = b3CreateWorld(&world_def);
    if (B3_IS_NULL(session->world_id))
    {
        free(session);
        return YMIR_BOX3D_INTERNAL_ERROR;
    }

    *output = session;
    return YMIR_BOX3D_OK;
}

YMIR_EXPORT void YMIR_CALL ymir_box3d_session_destroy(ymir_box3d_session* session)
{
    if (session == NULL)
    {
        return;
    }

    if (B3_IS_NON_NULL(session->world_id) && b3World_IsValid(session->world_id))
    {
        b3DestroyWorld(session->world_id);
    }
    free(session->bodies);
    free(session->contact_events);
    free(session->pending_events);
    free(session->retired_shapes);
    free(session->drained_events);
    free(session);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_sync_bodies(
    ymir_box3d_session* session,
    const ymir_box3d_session_body_input* bodies,
    int32_t body_count)
{
    if (session == NULL || body_count < 0 || (body_count > 0 && bodies == NULL))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }

    for (int i = 0; i < body_count; ++i)
    {
        if (!ymir_session_body_input_is_valid(bodies + i))
        {
            return YMIR_BOX3D_INVALID_ARGUMENT;
        }
        for (int j = 0; j < i; ++j)
        {
            if (bodies[i].stable_id == bodies[j].stable_id)
            {
                return YMIR_BOX3D_DUPLICATE_ID;
            }
        }
    }

    ymir_box3d_status status = ymir_session_reserve_bodies(session, session->body_count + body_count);
    if (status != YMIR_BOX3D_OK)
    {
        return status;
    }

    for (int i = 0; i < session->body_count; ++i)
    {
        session->bodies[i].seen = 0;
    }

    for (int i = 0; i < body_count; ++i)
    {
        int body_index = ymir_session_find_body(session, bodies[i].stable_id);
        if (body_index >= 0)
        {
            ymir_session_update_body(session->bodies + body_index, bodies + i);
            continue;
        }

        ymir_box3d_session_body body;
        status = ymir_session_create_body(session, bodies + i, &body);
        if (status != YMIR_BOX3D_OK)
        {
            return status;
        }
        session->bodies[session->body_count++] = body;
    }

    for (int i = session->body_count - 1; i >= 0; --i)
    {
        if (session->bodies[i].seen)
        {
            continue;
        }

        b3DestroyBody(session->bodies[i].body_id);
        session->bodies[i] = session->bodies[session->body_count - 1];
        session->body_count -= 1;
    }

    return YMIR_BOX3D_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_spawn(
    ymir_box3d_session* session,
    const ymir_box3d_session_body_input* body_input)
{
    if (session == NULL || !ymir_session_body_input_is_valid(body_input))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    if (ymir_session_find_body(session, body_input->stable_id) >= 0)
    {
        return YMIR_BOX3D_DUPLICATE_ID;
    }
    ymir_box3d_status status = ymir_session_reserve_bodies(session, session->body_count + 1);
    if (status != YMIR_BOX3D_OK)
    {
        return status;
    }
    ymir_box3d_session_body body;
    status = ymir_session_create_body(session, body_input, &body);
    if (status == YMIR_BOX3D_OK)
    {
        session->bodies[session->body_count++] = body;
    }
    return status;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_remove(ymir_box3d_session* session, uint64_t stable_id)
{
    if (session == NULL || stable_id == 0)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    int index = ymir_session_find_body(session, stable_id);
    if (index < 0)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    ymir_box3d_status status = ymir_session_reserve_retired_shapes(session, session->retired_shape_count + 1);
    if (status != YMIR_BOX3D_OK)
    {
        return status;
    }
    session->retired_shapes[session->retired_shape_count++] = (ymir_box3d_retired_shape){
        .shape_id = b3StoreShapeId(session->bodies[index].shape_id),
        .stable_id = stable_id,
    };
    b3DestroyBody(session->bodies[index].body_id);
    session->bodies[index] = session->bodies[session->body_count - 1];
    session->body_count -= 1;
    return ymir_session_drain_contact_events(session, YMIR_DRAIN_END, 1);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_teleport(
    ymir_box3d_session* session,
    uint64_t stable_id,
    float position_x,
    float position_y,
    float direction_x,
    float direction_y)
{
    float direction_length_squared = direction_x * direction_x + direction_y * direction_y;
    if (session == NULL || !ymir_is_finite(position_x) || !ymir_is_finite(position_y) ||
        !ymir_is_finite(direction_x) || !ymir_is_finite(direction_y) || direction_length_squared <= 1.0e-8f)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    int index = ymir_session_find_body(session, stable_id);
    if (index < 0)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    ymir_box3d_session_body* body = session->bodies + index;
    b3Body_SetTransform(body->body_id, ymir_vec(position_x, position_y), ymir_session_rotation(direction_x, direction_y));
    body->last_position_x = position_x;
    body->last_position_y = position_y;
    body->last_direction_x = direction_x;
    body->last_direction_y = direction_y;
    return ymir_session_drain_contact_events(session, YMIR_DRAIN_END, 1);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_set_velocity(
    ymir_box3d_session* session,
    uint64_t stable_id,
    float velocity_x,
    float velocity_y,
    float angular_velocity)
{
    if (session == NULL || !ymir_is_finite(velocity_x) || !ymir_is_finite(velocity_y) ||
        !ymir_is_finite(angular_velocity))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    int index = ymir_session_find_body(session, stable_id);
    if (index < 0)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    ymir_box3d_session_body* body = session->bodies + index;
    b3Body_SetLinearVelocity(body->body_id, ymir_vec(velocity_x, velocity_y));
    b3Body_SetAngularVelocity(body->body_id, (b3Vec3){0.0f, angular_velocity, 0.0f});
    body->last_velocity_x = velocity_x;
    body->last_velocity_y = velocity_y;
    body->last_angular_velocity = angular_velocity;
    return YMIR_BOX3D_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_configure(
    ymir_box3d_session* session,
    uint64_t stable_id,
    float radius,
    float mass,
    float restitution,
    uint32_t is_static,
    uint32_t is_kinematic)
{
    if (session == NULL || !ymir_is_finite(radius) || radius <= 0.0f || !ymir_is_finite(mass) ||
        (is_static == 0 && mass <= 0.0f) || (is_static != 0 && is_kinematic != 0) ||
        !ymir_is_finite(restitution) || restitution < 0.0f)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    int index = ymir_session_find_body(session, stable_id);
    if (index < 0)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    ymir_box3d_session_body* body = session->bodies + index;
    int type_changed = body->is_static != (is_static != 0) || body->is_kinematic != (is_kinematic != 0);
    int radius_changed = body->radius != radius;
    int mass_changed = body->mass != mass;
    if (type_changed)
    {
        b3Body_SetType(
            body->body_id,
            is_static != 0 ? b3_staticBody : (is_kinematic != 0 ? b3_kinematicBody : b3_dynamicBody));
    }
    if (radius_changed)
    {
        b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, radius};
        b3Shape_SetSphere(body->shape_id, &sphere);
    }
    if (type_changed || radius_changed || mass_changed)
    {
        float density = is_static != 0 ? 0.0f : ymir_session_density(mass, radius);
        b3Shape_SetDensity(body->shape_id, density, false);
        b3Body_ApplyMassFromShapes(body->body_id);
    }
    if (body->restitution != restitution)
    {
        b3Shape_SetRestitution(body->shape_id, restitution);
    }
    body->radius = radius;
    body->mass = mass;
    body->restitution = restitution;
    body->is_static = is_static != 0;
    body->is_kinematic = is_kinematic != 0;
    ymir_box3d_status status = ymir_session_drain_contact_events(session, YMIR_DRAIN_END, 1);
    return status;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_apply_force(
    ymir_box3d_session* session,
    uint64_t stable_id,
    float force_x,
    float force_y)
{
    if (session == NULL || !ymir_is_finite(force_x) || !ymir_is_finite(force_y))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    int index = ymir_session_find_body(session, stable_id);
    if (index < 0)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    b3Body_ApplyForceToCenter(session->bodies[index].body_id, ymir_vec(force_x, force_y), true);
    return YMIR_BOX3D_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_apply_torque(
    ymir_box3d_session* session,
    uint64_t stable_id,
    float torque)
{
    if (session == NULL || !ymir_is_finite(torque))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    int index = ymir_session_find_body(session, stable_id);
    if (index < 0)
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    b3Body_ApplyTorque(session->bodies[index].body_id, (b3Vec3){0.0f, torque, 0.0f}, true);
    return YMIR_BOX3D_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_step(
    ymir_box3d_session* session,
    float time_step,
    int32_t sub_step_count,
    const ymir_box3d_radial_field_input* fields,
    int32_t field_count)
{
    if (session == NULL || !ymir_is_finite(time_step) || time_step <= 0.0f || sub_step_count <= 0 ||
        field_count < 0 || (field_count > 0 && fields == NULL))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }

    for (int field_index = 0; field_index < field_count; ++field_index)
    {
        const ymir_box3d_radial_field_input* field = fields + field_index;
        if (!ymir_is_finite(field->position_x) || !ymir_is_finite(field->position_y) ||
            !ymir_is_finite(field->strength) || !ymir_is_finite(field->radius))
        {
            return YMIR_BOX3D_INVALID_ARGUMENT;
        }
    }

    session->contact_event_count = 0;
    ymir_box3d_status event_status = ymir_session_reserve_contact_events(
        &session->contact_events, &session->contact_event_capacity, session->pending_event_count);
    if (event_status != YMIR_BOX3D_OK)
    {
        return event_status;
    }
    if (session->pending_event_count > 0)
    {
        memcpy(
            session->contact_events,
            session->pending_events,
            (size_t)session->pending_event_count * sizeof(ymir_box3d_session_contact_event_output));
        session->contact_event_count = session->pending_event_count;
        session->pending_event_count = 0;
    }

    for (int body_index = 0; body_index < session->body_count; ++body_index)
    {
        ymir_box3d_session_body* body = session->bodies + body_index;
        if (body->is_static || body->is_kinematic)
        {
            body->pending_torque = 0.0f;
            continue;
        }

        if (body->pending_torque != 0.0f)
        {
            b3Body_ApplyTorque(body->body_id, (b3Vec3){0.0f, body->pending_torque, 0.0f}, true);
        }
        body->pending_torque = 0.0f;

        if (!body->participates_in_fields)
        {
            continue;
        }

        b3Pos position = b3Body_GetPosition(body->body_id);
        float acceleration_x = 0.0f;
        float acceleration_y = 0.0f;
        for (int field_index = 0; field_index < field_count; ++field_index)
        {
            const ymir_box3d_radial_field_input* field = fields + field_index;
            if (field->radius <= 0.0f || field->strength == 0.0f)
            {
                continue;
            }

            float delta_x = field->position_x - position.x;
            float delta_y = field->position_y - position.z;
            float distance_squared = delta_x * delta_x + delta_y * delta_y;
            if (distance_squared <= 0.0f)
            {
                continue;
            }

            float distance = sqrtf(distance_squared);
            if (distance > field->radius)
            {
                continue;
            }

            float falloff = 1.0f - distance / field->radius;
            float magnitude = field->strength * falloff / distance;
            acceleration_x += delta_x * magnitude;
            acceleration_y += delta_y * magnitude;
        }

        if (acceleration_x != 0.0f || acceleration_y != 0.0f)
        {
            float mass = b3Body_GetMass(body->body_id);
            b3Body_ApplyForceToCenter(body->body_id, ymir_vec(mass * acceleration_x, mass * acceleration_y), true);
        }
    }

    b3World_Step(session->world_id, time_step, sub_step_count);
    session->drained_event_count = 0;
    for (int i = 0; i < session->body_count; ++i)
    {
        b3Pos position = b3Body_GetPosition(session->bodies[i].body_id);
        b3Vec3 velocity = b3Body_GetLinearVelocity(session->bodies[i].body_id);
        b3Vec3 direction = ymir_session_direction(session->bodies[i].body_id);
        b3Vec3 angular_velocity = b3Body_GetAngularVelocity(session->bodies[i].body_id);
        session->bodies[i].last_position_x = position.x;
        session->bodies[i].last_position_y = position.z;
        session->bodies[i].last_velocity_x = velocity.x;
        session->bodies[i].last_velocity_y = velocity.z;
        session->bodies[i].last_direction_x = direction.x;
        session->bodies[i].last_direction_y = direction.z;
        session->bodies[i].last_angular_velocity = angular_velocity.y;
    }

    event_status = ymir_session_drain_contact_events(
        session, YMIR_DRAIN_BEGIN | YMIR_DRAIN_HIT | YMIR_DRAIN_END, 0);
    session->retired_shape_count = 0;
    if (event_status != YMIR_BOX3D_OK)
    {
        return event_status;
    }

    return YMIR_BOX3D_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_get_body_count(const ymir_box3d_session* session)
{
    return session != NULL ? session->body_count : -YMIR_BOX3D_INVALID_ARGUMENT;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_copy_bodies(
    const ymir_box3d_session* session,
    ymir_box3d_session_body_output* output,
    int32_t capacity,
    int32_t* written)
{
    if (session == NULL || written == NULL || capacity < 0 || (capacity > 0 && output == NULL))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    *written = session->body_count;
    if (capacity < session->body_count)
    {
        return YMIR_BOX3D_BUFFER_TOO_SMALL;
    }

    for (int i = 0; i < session->body_count; ++i)
    {
        b3Pos position = b3Body_GetPosition(session->bodies[i].body_id);
        b3Vec3 velocity = b3Body_GetLinearVelocity(session->bodies[i].body_id);
        b3Vec3 direction = ymir_session_direction(session->bodies[i].body_id);
        b3Vec3 angular_velocity = b3Body_GetAngularVelocity(session->bodies[i].body_id);
        output[i] = (ymir_box3d_session_body_output){
            .stable_id = session->bodies[i].stable_id,
            .position_x = position.x,
            .position_y = position.z,
            .velocity_x = velocity.x,
            .velocity_y = velocity.z,
            .direction_x = direction.x,
            .direction_y = direction.z,
            .angular_velocity = angular_velocity.y,
        };
    }
    return YMIR_BOX3D_OK;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_get_contact_count(const ymir_box3d_session* session)
{
    if (session == NULL)
    {
        return -YMIR_BOX3D_INVALID_ARGUMENT;
    }

    int count = 0;
    ymir_box3d_status status = ymir_session_visit_contacts(session, NULL, 0, &count);
    return status == YMIR_BOX3D_OK ? count : -(int)status;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_copy_contacts(
    const ymir_box3d_session* session,
    ymir_box3d_session_contact_output* output,
    int32_t capacity,
    int32_t* written)
{
    if (session == NULL || written == NULL || capacity < 0 || (capacity > 0 && output == NULL))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    *written = 0;
    return ymir_session_visit_contacts(session, output, capacity, written);
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_get_contact_event_count(const ymir_box3d_session* session)
{
    return session != NULL ? session->contact_event_count : -YMIR_BOX3D_INVALID_ARGUMENT;
}

YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_session_copy_contact_events(
    const ymir_box3d_session* session,
    ymir_box3d_session_contact_event_output* output,
    int32_t capacity,
    int32_t* written)
{
    if (session == NULL || written == NULL || capacity < 0 || (capacity > 0 && output == NULL))
    {
        return YMIR_BOX3D_INVALID_ARGUMENT;
    }
    *written = session->contact_event_count;
    if (capacity < session->contact_event_count)
    {
        return YMIR_BOX3D_BUFFER_TOO_SMALL;
    }
    if (session->contact_event_count > 0)
    {
        memcpy(
            output,
            session->contact_events,
            (size_t)session->contact_event_count * sizeof(ymir_box3d_session_contact_event_output));
    }
    return YMIR_BOX3D_OK;
}
