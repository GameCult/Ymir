using System.Runtime.InteropServices;

namespace Ymir.Box3D;

public enum Box3DSoaKind : uint
{
    Sphere = 0,
    StaticBox = 1,
    Capsule = 2,
    Box = 3
}

public enum Box3DSoaJointKind : uint
{
    Revolute = 0,
    Spherical = 1,
    Weld = 2
}

/// <summary>Native layout of <c>ymir_box3d_soa_body_desc</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct Box3DSoaBodyDesc
{
    public ulong StableId;
    public uint Kind;
    public int Parent;
    public int Limb;
    public uint Reserved;
    public float PositionX, PositionY, PositionZ;
    public float RotationX, RotationY, RotationZ, RotationW;
    public float VelocityX, VelocityY, VelocityZ;
    public float Size0, Size1, Size2;
    public float Density, Friction, Restitution, RollingResistance;
}

/// <summary>Native layout of <c>ymir_box3d_soa_joint_desc</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct Box3DSoaJointDesc
{
    public uint Kind;
    public int BodyA;
    public int BodyB;
    public uint EnableLimit;
    public float AnchorX, AnchorY, AnchorZ;
    public float AxisX, AxisY, AxisZ;
    public float Lower, Upper, ConeAngle, ReservedFloat;
}

/// <summary>Native layout of <c>ymir_box3d_soa_view</c>: counts, then one pointer per lane.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct Box3DSoaViewNative
{
    public uint AbiVersion;
    public int Count;
    public int Capacity;
    public int JointCount;
    public int JointCapacity;
    public uint Reserved;
    public ulong StepCount;
    public ulong* StableId;
    public uint* Kind;
    public int* Parent;
    public int* Limb;
    public float* Size0;
    public float* Size1;
    public float* Size2;
    public float* PositionX;
    public float* PositionY;
    public float* PositionZ;
    public float* RotationX;
    public float* RotationY;
    public float* RotationZ;
    public float* RotationW;
    public float* VelocityX;
    public float* VelocityY;
    public float* VelocityZ;
    public float* AngularVelocityX;
    public float* AngularVelocityY;
    public float* AngularVelocityZ;
    public float* ForceX;
    public float* ForceY;
    public float* ForceZ;
    public float* TorqueX;
    public float* TorqueY;
    public float* TorqueZ;
    public uint* JointKind;
    public int* JointBodyA;
    public int* JointBodyB;
    public float* JointAngle;
    public float* JointFrameX;
    public float* JointFrameY;
    public float* JointFrameZ;
    public float* JointFrameW;
    public float* JointRotationX;
    public float* JointRotationY;
    public float* JointRotationZ;
    public float* JointRotationW;
    public float* JointAngularVelocityX;
    public float* JointAngularVelocityY;
    public float* JointAngularVelocityZ;
}

internal static unsafe partial class Box3DSoaNative
{
    private const string Library = Box3DNative.LibraryName;

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_abi_version();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_view_size();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_body_desc_size();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_joint_desc_size();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_create(int capacity, int jointCapacity, float gx, float gy, float gz, out nint world);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial void ymir_box3d_soa_destroy(nint world);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_add_body(nint world, in Box3DSoaBodyDesc desc, out int index);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_add_joint(nint world, in Box3DSoaJointDesc desc, out int index);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_step(nint world, float timeStep, int subStepCount);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_get_view(nint world, out Box3DSoaViewNative view);
}

/// <summary>
/// The lanes of a <see cref="Box3DSoaWorld"/>: contiguous native arrays, one per component, indexed
/// by body. Each accessor is a span over native memory, not a copy. The pointers are fixed for the
/// world's life, the contents change only inside <see cref="Box3DSoaWorld.Step"/>, and every span is
/// invalid once the world is disposed.
/// </summary>
public readonly unsafe struct Box3DSoaView
{
    private readonly Box3DSoaViewNative _v;
    private readonly Box3DSoaWorld? _world;

    private ReadOnlySpan<T> Lane<T>(T* p, int n) where T : unmanaged
    {
        (_world ?? throw new InvalidOperationException("This Box3DSoaView has no world; take it from Box3DSoaWorld.View.")).ThrowIfDisposed();
        return new(p, n);
    }

    private Span<T> Write<T>(T* p, int n) where T : unmanaged
    {
        (_world ?? throw new InvalidOperationException("This Box3DSoaView has no world; take it from Box3DSoaWorld.View.")).ThrowIfDisposed();
        return new(p, n);
    }

    internal Box3DSoaView(Box3DSoaViewNative v, Box3DSoaWorld world)
    {
        _v = v;
        _world = world;
    }

    /// <summary>Bodies at the moment this view was taken: take it after the last Add. Only the lane contents are live.</summary>
    public int Count => _v.Count;
    public int Capacity => _v.Capacity;

    public ReadOnlySpan<ulong> StableId => Lane(_v.StableId, Count);
    public ReadOnlySpan<uint> Kind => Lane(_v.Kind, Count);
    /// <summary>Lane index of the body's parent in the limb tree, or -1 for a root. Set at creation, not used by Box3D.</summary>
    public ReadOnlySpan<int> Parent => Lane(_v.Parent, Count);
    /// <summary>The caller's limb label for the body. Set at creation, not used by Box3D.</summary>
    public ReadOnlySpan<int> Limb => Lane(_v.Limb, Count);
    /// <summary>The body's shape size, read-only, set at creation and never changed by Step. Size0 is the sphere or
    /// capsule radius, or the box half-extent x; Size1 is the capsule half-length (along local Y) or the box half-extent y;
    /// Size2 is the box half-extent z. Unused components are 0. Shapes are centred on the body origin.</summary>
    public ReadOnlySpan<float> Size0 => Lane(_v.Size0, Count);
    public ReadOnlySpan<float> Size1 => Lane(_v.Size1, Count);
    public ReadOnlySpan<float> Size2 => Lane(_v.Size2, Count);
    public ReadOnlySpan<float> PositionX => Lane(_v.PositionX, Count);
    public ReadOnlySpan<float> PositionY => Lane(_v.PositionY, Count);
    public ReadOnlySpan<float> PositionZ => Lane(_v.PositionZ, Count);
    /// <summary>Unit quaternion, x y z w.</summary>
    public ReadOnlySpan<float> RotationX => Lane(_v.RotationX, Count);
    public ReadOnlySpan<float> RotationY => Lane(_v.RotationY, Count);
    public ReadOnlySpan<float> RotationZ => Lane(_v.RotationZ, Count);
    public ReadOnlySpan<float> RotationW => Lane(_v.RotationW, Count);
    public ReadOnlySpan<float> VelocityX => Lane(_v.VelocityX, Count);
    public ReadOnlySpan<float> VelocityY => Lane(_v.VelocityY, Count);
    public ReadOnlySpan<float> VelocityZ => Lane(_v.VelocityZ, Count);
    public ReadOnlySpan<float> AngularVelocityX => Lane(_v.AngularVelocityX, Count);
    public ReadOnlySpan<float> AngularVelocityY => Lane(_v.AngularVelocityY, Count);
    public ReadOnlySpan<float> AngularVelocityZ => Lane(_v.AngularVelocityZ, Count);

    // Write lanes: the caller's per-body world force (N, at the centre of mass) and torque (N m), written
    // in place before a step. Step applies them to the dynamic bodies and clears them, so a lane holds one
    // step's intent. A non-finite value makes Step throw before any body is touched.
    public Span<float> ForceX => Write(_v.ForceX, Count);
    public Span<float> ForceY => Write(_v.ForceY, Count);
    public Span<float> ForceZ => Write(_v.ForceZ, Count);
    public Span<float> TorqueX => Write(_v.TorqueX, Count);
    public Span<float> TorqueY => Write(_v.TorqueY, Count);
    public Span<float> TorqueZ => Write(_v.TorqueZ, Count);

    /// <summary>Joints at the moment this view was taken: take it after the last AddJoint.</summary>
    public int JointCount => _v.JointCount;
    public ReadOnlySpan<uint> JointKind => Lane(_v.JointKind, JointCount);
    /// <summary>Lane indices of the joint's two bodies; the second is the child.</summary>
    public ReadOnlySpan<int> JointBodyA => Lane(_v.JointBodyA, JointCount);
    public ReadOnlySpan<int> JointBodyB => Lane(_v.JointBodyB, JointCount);
    /// <summary>Box3D's own angle: revolute angle or spherical twist angle (radians), zero for a weld.</summary>
    public ReadOnlySpan<float> JointAngle => Lane(_v.JointAngle, JointCount);
    /// <summary>World orientation of the joint frame on body A (quaternion x y z w).</summary>
    public ReadOnlySpan<float> JointFrameX => Lane(_v.JointFrameX, JointCount);
    public ReadOnlySpan<float> JointFrameY => Lane(_v.JointFrameY, JointCount);
    public ReadOnlySpan<float> JointFrameZ => Lane(_v.JointFrameZ, JointCount);
    public ReadOnlySpan<float> JointFrameW => Lane(_v.JointFrameW, JointCount);
    /// <summary>Joint frame B relative to frame A (quaternion x y z w, w >= 0); identity at the creation pose.</summary>
    public ReadOnlySpan<float> JointRotationX => Lane(_v.JointRotationX, JointCount);
    public ReadOnlySpan<float> JointRotationY => Lane(_v.JointRotationY, JointCount);
    public ReadOnlySpan<float> JointRotationZ => Lane(_v.JointRotationZ, JointCount);
    public ReadOnlySpan<float> JointRotationW => Lane(_v.JointRotationW, JointCount);
    /// <summary>Body B's angular velocity minus body A's, expressed in frame A (rad/s).</summary>
    public ReadOnlySpan<float> JointAngularVelocityX => Lane(_v.JointAngularVelocityX, JointCount);
    public ReadOnlySpan<float> JointAngularVelocityY => Lane(_v.JointAngularVelocityY, JointCount);
    public ReadOnlySpan<float> JointAngularVelocityZ => Lane(_v.JointAngularVelocityZ, JointCount);

    /// <summary>Address of a lane, for tests and interop that need to prove nothing was copied.</summary>
    public nint PositionYAddress => (nint)_v.PositionY;
}

/// <summary>
/// A headless 3D Box3D world: N bodies stepped deterministically (one worker, fixed order) with a
/// structure-of-arrays state view read in place. Ymir writes no solver; Box3D decides everything.
/// Not thread-safe: Dispose must not race Step or a lane read. Dispose frees the native world; an
/// undisposed world leaks it (no finalizer by design in this cut).
/// </summary>
public sealed class Box3DSoaWorld : IDisposable
{
    private const uint ExpectedAbi = 3;
    private nint _handle;

    public Box3DSoaWorld(int capacity, float gravityX = 0f, float gravityY = -10f, float gravityZ = 0f, int jointCapacity = 0)
    {
        if (Box3DSoaNative.ymir_box3d_soa_get_abi_version() != ExpectedAbi ||
            Box3DSoaNative.ymir_box3d_soa_get_view_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaViewNative>() ||
            Box3DSoaNative.ymir_box3d_soa_get_body_desc_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaBodyDesc>() ||
            Box3DSoaNative.ymir_box3d_soa_get_joint_desc_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaJointDesc>())
        {
            throw new InvalidOperationException("The native Ymir Box3D SoA ABI does not match this build.");
        }

        Check(Box3DSoaNative.ymir_box3d_soa_create(capacity, jointCapacity, gravityX, gravityY, gravityZ, out _handle));
    }

    /// <summary>The live lanes. Count and step count are read from native memory each call.</summary>
    public Box3DSoaView View
    {
        get
        {
            ObjectDisposedException.ThrowIf(_handle == 0, this);
            Check(Box3DSoaNative.ymir_box3d_soa_get_view(_handle, out var v));
            return new Box3DSoaView(v, this);
        }
    }

    /// <summary>Steps taken so far.</summary>
    public ulong StepCount
    {
        get
        {
            ObjectDisposedException.ThrowIf(_handle == 0, this);
            Check(Box3DSoaNative.ymir_box3d_soa_get_view(_handle, out var v));
            return v.StepCount;
        }
    }

    /// <summary>Adds a dynamic sphere; returns its lane index.</summary>
    public int AddSphere(
        ulong stableId, (float X, float Y, float Z) position, float radius, float density,
        (float X, float Y, float Z) velocity = default, float friction = 0.6f, float restitution = 0f,
        int parent = -1, int limb = 0, float rollingResistance = 0f) =>
        AddBody(Describe(Box3DSoaKind.Sphere, stableId, position, default, (radius, 0f, 0f), density,
            velocity, friction, restitution, parent, limb, rollingResistance));

    /// <summary>Adds a static axis-aligned box centred at <paramref name="position"/>.</summary>
    public int AddStaticBox(
        ulong stableId, (float X, float Y, float Z) position, (float X, float Y, float Z) halfExtents,
        float friction = 0.6f, float restitution = 0f) =>
        AddBody(Describe(Box3DSoaKind.StaticBox, stableId, position, default, halfExtents, 0f,
            default, friction, restitution, -1, 0));

    /// <summary>
    /// Adds a dynamic capsule: a segment of <paramref name="halfLength"/> either side of the centre along
    /// the body's local Y, swept by <paramref name="radius"/>. <paramref name="rotation"/> is a unit quaternion
    /// x y z w; the default (all zero) means identity.
    /// </summary>
    public int AddCapsule(
        ulong stableId, (float X, float Y, float Z) position, float radius, float halfLength, float density,
        (float X, float Y, float Z, float W) rotation = default, float friction = 0.6f, float restitution = 0f,
        int parent = -1, int limb = 0, float rollingResistance = 0f) =>
        AddBody(Describe(Box3DSoaKind.Capsule, stableId, position, rotation, (radius, halfLength, 0f), density,
            default, friction, restitution, parent, limb, rollingResistance));

    /// <summary>Adds a dynamic box with the given half extents; rotation as for <see cref="AddCapsule"/>.</summary>
    public int AddBox(
        ulong stableId, (float X, float Y, float Z) position, (float X, float Y, float Z) halfExtents, float density,
        (float X, float Y, float Z, float W) rotation = default, float friction = 0.6f, float restitution = 0f,
        int parent = -1, int limb = 0) =>
        AddBody(Describe(Box3DSoaKind.Box, stableId, position, rotation, halfExtents, density,
            default, friction, restitution, parent, limb));

    /// <summary>
    /// Joins two bodies at a world-space <paramref name="anchor"/>. The joint frame's z axis is
    /// <paramref name="axis"/> (the hinge axis of a revolute joint); the pose at creation is the joint's zero.
    /// Limits are radians: the revolute angle, or the spherical twist with <paramref name="coneAngle"/> as swing.
    /// A weld takes no limits. Returns the joint's lane index. Body B is the child.
    /// </summary>
    public int AddJoint(
        Box3DSoaJointKind kind, int bodyA, int bodyB, (float X, float Y, float Z) anchor,
        (float X, float Y, float Z) axis, bool limit = false, float lower = 0f, float upper = 0f, float coneAngle = 0f)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        var desc = new Box3DSoaJointDesc
        {
            Kind = (uint)kind, BodyA = bodyA, BodyB = bodyB, EnableLimit = limit ? 1u : 0u,
            AnchorX = anchor.X, AnchorY = anchor.Y, AnchorZ = anchor.Z,
            AxisX = axis.X, AxisY = axis.Y, AxisZ = axis.Z, Lower = lower, Upper = upper, ConeAngle = coneAngle,
        };
        Check(Box3DSoaNative.ymir_box3d_soa_add_joint(_handle, desc, out var index));
        return index;
    }

    private static Box3DSoaBodyDesc Describe(
        Box3DSoaKind kind, ulong stableId, (float X, float Y, float Z) position,
        (float X, float Y, float Z, float W) rotation, (float, float, float) size, float density,
        (float X, float Y, float Z) velocity, float friction, float restitution, int parent, int limb, float rollingResistance = 0f)
    {
        if (rotation == default) rotation = (0f, 0f, 0f, 1f);
        return new Box3DSoaBodyDesc
        {
            StableId = stableId, Kind = (uint)kind, Parent = parent, Limb = limb,
            PositionX = position.X, PositionY = position.Y, PositionZ = position.Z,
            RotationX = rotation.X, RotationY = rotation.Y, RotationZ = rotation.Z, RotationW = rotation.W,
            VelocityX = velocity.X, VelocityY = velocity.Y, VelocityZ = velocity.Z,
            Size0 = size.Item1, Size1 = size.Item2, Size2 = size.Item3,
            Density = density, Friction = friction, Restitution = restitution, RollingResistance = rollingResistance,
        };
    }

    private int AddBody(Box3DSoaBodyDesc desc)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        Check(Box3DSoaNative.ymir_box3d_soa_add_body(_handle, desc, out var index));
        return index;
    }

    public void Step(float timeStep, int subStepCount = 4)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        Check(Box3DSoaNative.ymir_box3d_soa_step(_handle, timeStep, subStepCount));
    }

    internal void ThrowIfDisposed() => ObjectDisposedException.ThrowIf(Volatile.Read(ref _handle) == 0, this);

    public void Dispose()
    {
        var handle = Interlocked.Exchange(ref _handle, 0);
        if (handle != 0)
        {
            Box3DSoaNative.ymir_box3d_soa_destroy(handle);
        }
    }

    private static void Check(int status)
    {
        if (status != 0)
        {
            throw new InvalidOperationException($"Ymir Box3D SoA call failed with status {status}.");
        }
    }
}
