namespace Ymir.Box3D;

internal readonly record struct Box3DSphereCastHit(
    float Fraction,
    float PointX,
    float PointZ,
    float NormalX,
    float NormalZ);

internal readonly record struct Box3DSphereOverlapHit(float Distance, float NormalX, float NormalZ);

internal static class Box3DQueries
{
    internal static Box3DSphereOverlapHit? OverlapSpheres(
        float queryX,
        float queryZ,
        float queryRadius,
        float bodyX,
        float bodyZ,
        float bodyRadius)
    {
        Box3DSession.EnsureCompatibleNativeLibrary();
        var output = Box3DNative.ymir_box3d_overlap_spheres(
            queryX,
            queryZ,
            queryRadius,
            bodyX,
            bodyZ,
            bodyRadius);
        return output.Hit == 0
            ? null
            : new Box3DSphereOverlapHit(output.Distance, output.NormalX, output.NormalZ);
    }

    internal static Box3DSphereCastHit? CastSphere(
        float originX,
        float originZ,
        float translationX,
        float translationZ,
        float queryRadius,
        float bodyX,
        float bodyZ,
        float bodyRadius)
    {
        Box3DSession.EnsureCompatibleNativeLibrary();
        var output = Box3DNative.ymir_box3d_cast_sphere(
            originX,
            originZ,
            translationX,
            translationZ,
            queryRadius,
            bodyX,
            bodyZ,
            bodyRadius,
            canEncroach: 0);
        return output.Hit == 0
            ? null
            : new Box3DSphereCastHit(
                output.Fraction,
                output.PointX,
                output.PointZ,
                output.NormalX,
                output.NormalZ);
    }
}
