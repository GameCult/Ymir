using System.Runtime.InteropServices;

namespace Ymir.Box3D;

internal enum Box3DSoaKind : uint
{
    Sphere = 0,
    StaticBox = 1
}

/// <summary>Native layout of <c>ymir_box3d_soa_view</c>: counts, then one pointer per lane.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct Box3DSoaViewNative
{
    public uint AbiVersion;
    public int Count;
    public int Capacity;
    public uint Reserved;
    public ulong StepCount;
    public ulong* StableId;
    public uint* Kind;
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
    internal static partial int ymir_box3d_soa_create(int capacity, float gx, float gy, float gz, out nint world);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial void ymir_box3d_soa_destroy(nint world);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_add_sphere(
        nint world, ulong stableId, float px, float py, float pz, float vx, float vy, float vz,
        float radius, float density, float friction, float restitution, out int index);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_add_static_box(
        nint world, ulong stableId, float px, float py, float pz, float hx, float hy, float hz,
        float friction, float restitution, out int index);

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
internal readonly unsafe struct Box3DSoaView
{
    private readonly Box3DSoaViewNative _v;

    internal Box3DSoaView(Box3DSoaViewNative v) => _v = v;

    /// <summary>Bodies at the moment this view was taken: take it after the last Add. Only the lane contents are live.</summary>
    public int Count => _v.Count;
    public int Capacity => _v.Capacity;

    public ReadOnlySpan<ulong> StableId => new(_v.StableId, Count);
    public ReadOnlySpan<uint> Kind => new(_v.Kind, Count);
    public ReadOnlySpan<float> PositionX => new(_v.PositionX, Count);
    public ReadOnlySpan<float> PositionY => new(_v.PositionY, Count);
    public ReadOnlySpan<float> PositionZ => new(_v.PositionZ, Count);
    /// <summary>Unit quaternion, x y z w.</summary>
    public ReadOnlySpan<float> RotationX => new(_v.RotationX, Count);
    public ReadOnlySpan<float> RotationY => new(_v.RotationY, Count);
    public ReadOnlySpan<float> RotationZ => new(_v.RotationZ, Count);
    public ReadOnlySpan<float> RotationW => new(_v.RotationW, Count);
    public ReadOnlySpan<float> VelocityX => new(_v.VelocityX, Count);
    public ReadOnlySpan<float> VelocityY => new(_v.VelocityY, Count);
    public ReadOnlySpan<float> VelocityZ => new(_v.VelocityZ, Count);
    public ReadOnlySpan<float> AngularVelocityX => new(_v.AngularVelocityX, Count);
    public ReadOnlySpan<float> AngularVelocityY => new(_v.AngularVelocityY, Count);
    public ReadOnlySpan<float> AngularVelocityZ => new(_v.AngularVelocityZ, Count);

    /// <summary>Address of a lane, for tests and interop that need to prove nothing was copied.</summary>
    public nint PositionYAddress => (nint)_v.PositionY;
}

/// <summary>
/// A headless 3D Box3D world: N bodies stepped deterministically (one worker, fixed order) with a
/// structure-of-arrays state view read in place. Ymir writes no solver; Box3D decides everything.
/// </summary>
internal sealed class Box3DSoaWorld : IDisposable
{
    private const uint ExpectedAbi = 1;
    private nint _handle;

    public Box3DSoaWorld(int capacity, float gravityX = 0f, float gravityY = -10f, float gravityZ = 0f)
    {
        if (Box3DSoaNative.ymir_box3d_soa_get_abi_version() != ExpectedAbi ||
            Box3DSoaNative.ymir_box3d_soa_get_view_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaViewNative>())
        {
            throw new InvalidOperationException("The native Ymir Box3D SoA ABI does not match this build.");
        }

        Check(Box3DSoaNative.ymir_box3d_soa_create(capacity, gravityX, gravityY, gravityZ, out _handle));
    }

    /// <summary>The live lanes. Count and step count are read from native memory each call.</summary>
    public Box3DSoaView View
    {
        get
        {
            ObjectDisposedException.ThrowIf(_handle == 0, this);
            Check(Box3DSoaNative.ymir_box3d_soa_get_view(_handle, out var v));
            return new Box3DSoaView(v);
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
        (float X, float Y, float Z) velocity = default, float friction = 0.6f, float restitution = 0f)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        Check(Box3DSoaNative.ymir_box3d_soa_add_sphere(
            _handle, stableId, position.X, position.Y, position.Z, velocity.X, velocity.Y, velocity.Z,
            radius, density, friction, restitution, out var index));
        return index;
    }

    /// <summary>Adds a static axis-aligned box centred at <paramref name="position"/>.</summary>
    public int AddStaticBox(
        ulong stableId, (float X, float Y, float Z) position, (float X, float Y, float Z) halfExtents,
        float friction = 0.6f, float restitution = 0f)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        Check(Box3DSoaNative.ymir_box3d_soa_add_static_box(
            _handle, stableId, position.X, position.Y, position.Z, halfExtents.X, halfExtents.Y, halfExtents.Z,
            friction, restitution, out var index));
        return index;
    }

    public void Step(float timeStep, int subStepCount = 4)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        Check(Box3DSoaNative.ymir_box3d_soa_step(_handle, timeStep, subStepCount));
    }

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
