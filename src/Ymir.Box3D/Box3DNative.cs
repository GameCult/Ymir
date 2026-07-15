using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using Microsoft.Win32.SafeHandles;

namespace Ymir.Box3D;

internal enum Box3DStatus
{
    Ok = 0,
    InvalidArgument = 1,
    DuplicateId = 2,
    BufferTooSmall = 3,
    OutOfMemory = 4,
    InternalError = 5
}

internal static partial class Box3DNative
{
    internal const string LibraryName = "ymir_box3d";

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial uint ymir_box3d_get_abi_version();

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial nint ymir_box3d_get_build_id();

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_get_abi_layout(
        out uint bodyInputSize,
        out uint fieldInputSize,
        out uint bodyOutputSize,
        out uint contactOutputSize,
        out uint contactEventOutputSize);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_get_version(out int major, out int minor, out int revision);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DCastOutput ymir_box3d_cast_sphere(
        float originX,
        float originZ,
        float translationX,
        float translationZ,
        float queryRadius,
        float bodyX,
        float bodyZ,
        float bodyRadius,
        int canEncroach);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DOverlapOutput ymir_box3d_overlap_spheres(
        float queryX,
        float queryZ,
        float queryRadius,
        float bodyX,
        float bodyZ,
        float bodyRadius);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_session_create(out nint session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial void ymir_box3d_session_destroy(nint session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial Box3DStatus ymir_box3d_session_sync_bodies(
        Box3DSessionHandle session,
        Box3DBodyInput* bodies,
        int bodyCount);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial Box3DStatus ymir_box3d_session_spawn(
        Box3DSessionHandle session,
        Box3DBodyInput* body);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_session_remove(Box3DSessionHandle session, ulong stableId);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_session_teleport(
        Box3DSessionHandle session,
        ulong stableId,
        float positionX,
        float positionZ,
        float directionX,
        float directionZ);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_session_set_velocity(
        Box3DSessionHandle session,
        ulong stableId,
        float velocityX,
        float velocityZ,
        float angularVelocity);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_session_configure(
        Box3DSessionHandle session,
        ulong stableId,
        float radius,
        float mass,
        float restitution,
        uint isStatic,
        uint isKinematic);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_session_apply_force(
        Box3DSessionHandle session,
        ulong stableId,
        float forceX,
        float forceZ);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial Box3DStatus ymir_box3d_session_apply_torque(
        Box3DSessionHandle session,
        ulong stableId,
        float torque);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial Box3DStatus ymir_box3d_session_step(
        Box3DSessionHandle session,
        float deltaTime,
        int substepCount,
        Box3DRadialField* fields,
        int fieldCount);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_session_get_body_count(Box3DSessionHandle session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial Box3DStatus ymir_box3d_session_copy_bodies(
        Box3DSessionHandle session,
        Box3DBodyOutput* output,
        int capacity,
        out int written);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_session_get_contact_count(Box3DSessionHandle session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial Box3DStatus ymir_box3d_session_copy_contacts(
        Box3DSessionHandle session,
        Box3DContactOutput* output,
        int capacity,
        out int written);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static partial int ymir_box3d_session_get_contact_event_count(Box3DSessionHandle session);

    [LibraryImport(LibraryName)]
    [UnmanagedCallConv(CallConvs = [typeof(CallConvCdecl)])]
    internal static unsafe partial Box3DStatus ymir_box3d_session_copy_contact_events(
        Box3DSessionHandle session,
        Box3DContactEventOutput* output,
        int capacity,
        out int written);
}

internal sealed class Box3DSessionHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    private Box3DSessionHandle() : base(true)
    {
    }

    internal Box3DSessionHandle(nint value) : base(true) => SetHandle(value);

    protected override bool ReleaseHandle()
    {
        Box3DNative.ymir_box3d_session_destroy(handle);
        return true;
    }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DBodyInput(
    ulong StableId,
    ulong CollisionCategoryBits,
    ulong CollisionMaskBits,
    float PositionX,
    float PositionZ,
    float VelocityX,
    float VelocityZ,
    float DirectionX,
    float DirectionZ,
    float AngularVelocity,
    float Torque,
    float Radius,
    float Mass,
    float Restitution,
    uint IsStatic,
    uint IsKinematic,
    uint IsBullet,
    uint ParticipatesInFields,
    int CollisionGroupIndex);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DRadialField(
    float PositionX,
    float PositionZ,
    float Strength,
    float Radius);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DBodyOutput(
    ulong StableId,
    float PositionX,
    float PositionZ,
    float VelocityX,
    float VelocityZ,
    float DirectionX,
    float DirectionZ,
    float AngularVelocity);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DContactOutput(
    ulong StableIdA,
    ulong StableIdB,
    float PointX,
    float PointZ,
    float NormalX,
    float NormalZ,
    float Penetration,
    float RelativeSpeed);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DContactEventOutput(
    int Kind,
    uint HasDetails,
    uint ContactId0,
    uint ContactId1,
    uint ContactId2,
    ulong StableIdA,
    ulong StableIdB,
    float PointX,
    float PointZ,
    float NormalX,
    float NormalZ,
    float Penetration,
    float RelativeSpeed);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DCastOutput(
    int Hit,
    float Fraction,
    float PointX,
    float PointZ,
    float NormalX,
    float NormalZ);

[StructLayout(LayoutKind.Sequential)]
internal readonly record struct Box3DOverlapOutput(
    int Hit,
    float Distance,
    float NormalX,
    float NormalZ);
