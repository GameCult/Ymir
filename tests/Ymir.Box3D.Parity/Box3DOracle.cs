using System.Runtime.InteropServices;

namespace Ymir.Box3D.Parity;

internal static partial class Box3DOracle
{
    private const string LibraryName = "ymir_box3d";

    [LibraryImport(LibraryName)]
    internal static partial int ymir_box3d_get_version(out int major, out int minor, out int revision);

    [LibraryImport(LibraryName)]
    internal static partial Box3DOverlapResult ymir_box3d_overlap_spheres(
        float queryX,
        float queryY,
        float queryRadius,
        float bodyX,
        float bodyY,
        float bodyRadius);

    [LibraryImport(LibraryName)]
    internal static partial Box3DOverlapResult ymir_box3d_overlap_capsule_sphere(
        float startX,
        float startY,
        float endX,
        float endY,
        float capsuleRadius,
        float bodyX,
        float bodyY,
        float bodyRadius);

    [LibraryImport(LibraryName)]
    internal static partial Box3DCastResult ymir_box3d_cast_sphere(
        float originX,
        float originY,
        float translationX,
        float translationY,
        float queryRadius,
        float bodyX,
        float bodyY,
        float bodyRadius,
        int canEncroach);
}

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DOverlapResult(int Hit, float Distance, float NormalX, float NormalY)
{
    public bool IsHit => Hit != 0;
}
[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DCastResult(
    int Hit,
    float Fraction,
    float PointX,
    float PointY,
    float NormalX,
    float NormalY)
{
    public bool IsHit => Hit != 0;
}
