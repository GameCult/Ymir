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

    [LibraryImport(LibraryName)]
    internal static partial void ymir_box3d_step_pair(
        in Box3DBodyInput bodyA,
        in Box3DBodyInput bodyB,
        float timeStep,
        int subStepCount,
        float restitutionThreshold,
        out Box3DPairStepResult result);

    [LibraryImport(LibraryName)]
    internal static partial Box3DTorqueResult ymir_box3d_torque_lifetime(float torque, float timeStep, int subStepCount);
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

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DBodyInput(
    float PositionX,
    float PositionY,
    float VelocityX,
    float VelocityY,
    float Radius,
    float Mass,
    float Restitution,
    int IsStatic);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DBodyOutput(
    float PositionX,
    float PositionY,
    float VelocityX,
    float VelocityY,
    float AngularVelocity);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DPairStepResult(
    Box3DBodyOutput BodyA,
    Box3DBodyOutput BodyB,
    int BeginContactCount,
    int EndContactCount,
    int HitContactCount);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DTorqueResult(
    float AngularVelocityAfterAppliedStep,
    float AngularVelocityAfterUnforcedStep);
