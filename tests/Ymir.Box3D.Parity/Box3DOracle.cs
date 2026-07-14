using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Ymir.Box3D.Parity;

internal static partial class Box3DOracle
{
    private const string LibraryName = "ymir_box3d";

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_get_version(out int major, out int minor, out int revision);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DOverlapResult ymir_box3d_overlap_spheres(
        float queryX,
        float queryY,
        float queryRadius,
        float bodyX,
        float bodyY,
        float bodyRadius);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
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
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
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
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial void ymir_box3d_step_pair(
        in Box3DBodyInput bodyA,
        in Box3DBodyInput bodyB,
        float timeStep,
        int subStepCount,
        float restitutionThreshold,
        out Box3DPairStepResult result);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DTorqueResult ymir_box3d_torque_lifetime(float torque, float timeStep, int subStepCount);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial uint ymir_box3d_get_abi_version();

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_session_create(out nint session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial void ymir_box3d_session_destroy(nint session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial int ymir_box3d_session_spawn(nint session, Box3DSessionBodyInput* body);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_session_remove(nint session, ulong stableId);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial int ymir_box3d_session_step(
        nint session,
        float deltaTime,
        int substepCount,
        void* fields,
        int fieldCount);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_session_get_contact_event_count(nint session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial int ymir_box3d_session_copy_contact_events(
        nint session,
        Box3DSessionContactEvent* events,
        int capacity,
        out int written);
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

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DSessionBodyInput(
    ulong StableId,
    float PositionX,
    float PositionY,
    float VelocityX,
    float VelocityY,
    float DirectionX,
    float DirectionY,
    float AngularVelocity,
    float Torque,
    float Radius,
    float Mass,
    float Restitution,
    uint IsStatic);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DSessionContactEvent(
    int Kind,
    uint HasDetails,
    uint ContactId0,
    uint ContactId1,
    uint ContactId2,
    ulong StableIdA,
    ulong StableIdB,
    float PointX,
    float PointY,
    float NormalX,
    float NormalY,
    float Penetration,
    float RelativeSpeed);
