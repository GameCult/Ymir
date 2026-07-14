#include <box3d/base.h>
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

static b3Vec3 ymir_vec(float x, float y)
{
    return (b3Vec3){x, 0.0f, y};
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
