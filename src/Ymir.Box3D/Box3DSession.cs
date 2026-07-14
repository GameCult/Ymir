using System.Runtime.InteropServices;

namespace Ymir.Box3D;

internal readonly record struct Box3DBody(
    ulong StableId,
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
    bool IsStatic);

internal readonly record struct Box3DField(float PositionX, float PositionZ, float Strength, float Radius);

internal readonly record struct Box3DBodyState(
    ulong StableId,
    float PositionX,
    float PositionZ,
    float VelocityX,
    float VelocityZ,
    float DirectionX,
    float DirectionZ,
    float AngularVelocity);

internal readonly record struct Box3DContact(
    ulong StableIdA,
    ulong StableIdB,
    float PointX,
    float PointZ,
    float NormalX,
    float NormalZ,
    float Penetration,
    float RelativeSpeed);

internal sealed class Box3DSession : IDisposable
{
    internal const uint AbiVersion = 2;
    internal const int DefaultSubstepCount = 4;

    private readonly Box3DSessionHandle _handle;
    private bool _disposed;

    internal Box3DSession()
    {
        EnsureCompatibleNativeLibrary();
        ThrowIfFailed(Box3DNative.ymir_box3d_session_create(out var value), "create session");
        if (value == 0)
        {
            throw new InvalidOperationException("Box3D returned a null Ymir session.");
        }

        _handle = new Box3DSessionHandle(value);
    }

    internal unsafe void SyncBodies(ReadOnlySpan<Box3DBody> bodies)
    {
        ThrowIfDisposed();
        var native = new Box3DBodyInput[bodies.Length];
        for (var i = 0; i < bodies.Length; i++)
        {
            var body = bodies[i];
            native[i] = new Box3DBodyInput(
                body.StableId,
                body.PositionX,
                body.PositionZ,
                body.VelocityX,
                body.VelocityZ,
                body.DirectionX,
                body.DirectionZ,
                body.AngularVelocity,
                body.Torque,
                body.Radius,
                body.Mass,
                body.Restitution,
                body.IsStatic ? 1u : 0u);
        }

        fixed (Box3DBodyInput* pointer = native)
        {
            var status = Box3DNative.ymir_box3d_session_sync_bodies(_handle, pointer, native.Length);
            if (status is Box3DStatus.OutOfMemory or Box3DStatus.InternalError)
            {
                Dispose();
            }
            ThrowIfFailed(status, "synchronize bodies");
        }
    }

    internal unsafe void Step(float deltaTime, ReadOnlySpan<Box3DField> fields, int substepCount = DefaultSubstepCount)
    {
        ThrowIfDisposed();
        if (!float.IsFinite(deltaTime) || deltaTime <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(deltaTime), "Delta time must be finite and positive.");
        }

        if (substepCount <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(substepCount), "Substep count must be positive.");
        }

        var native = new Box3DRadialField[fields.Length];
        for (var i = 0; i < fields.Length; i++)
        {
            native[i] = new Box3DRadialField(fields[i].PositionX, fields[i].PositionZ, fields[i].Strength, fields[i].Radius);
        }

        fixed (Box3DRadialField* pointer = native)
        {
            ThrowIfFailed(
                Box3DNative.ymir_box3d_session_step(_handle, deltaTime, substepCount, pointer, native.Length),
                "step session");
        }
    }

    internal unsafe Box3DBodyState[] CopyBodies()
    {
        ThrowIfDisposed();
        var count = Box3DNative.ymir_box3d_session_get_body_count(_handle);
        if (count < 0)
        {
            throw new InvalidOperationException("Box3D returned an invalid body count.");
        }

        var native = new Box3DBodyOutput[count];
        fixed (Box3DBodyOutput* pointer = native)
        {
            ThrowIfFailed(Box3DNative.ymir_box3d_session_copy_bodies(_handle, pointer, count, out var written), "copy bodies");
            if (written != count)
            {
                throw new InvalidOperationException($"Box3D body snapshot changed during copy: expected {count}, received {written}.");
            }
        }

        return native.Select(body => new Box3DBodyState(
            body.StableId,
            body.PositionX,
            body.PositionZ,
            body.VelocityX,
            body.VelocityZ,
            body.DirectionX,
            body.DirectionZ,
            body.AngularVelocity)).ToArray();
    }

    internal unsafe Box3DContact[] CopyContacts()
    {
        ThrowIfDisposed();
        var count = Box3DNative.ymir_box3d_session_get_contact_count(_handle);
        if (count < 0)
        {
            throw new InvalidOperationException("Box3D returned an invalid contact count.");
        }

        var native = new Box3DContactOutput[count];
        fixed (Box3DContactOutput* pointer = native)
        {
            ThrowIfFailed(Box3DNative.ymir_box3d_session_copy_contacts(_handle, pointer, count, out var written), "copy contacts");
            if (written != count)
            {
                throw new InvalidOperationException($"Box3D contact snapshot changed during copy: expected {count}, received {written}.");
            }
        }

        return native.Select(contact => new Box3DContact(
            contact.StableIdA,
            contact.StableIdB,
            contact.PointX,
            contact.PointZ,
            contact.NormalX,
            contact.NormalZ,
            contact.Penetration,
            contact.RelativeSpeed)).ToArray();
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _handle.Dispose();
        _disposed = true;
    }

    internal static void EnsureCompatibleNativeLibrary()
    {
        var abi = Box3DNative.ymir_box3d_get_abi_version();
        if (abi != AbiVersion)
        {
            throw new InvalidOperationException($"Ymir Box3D ABI mismatch. Managed expects {AbiVersion}, native provides {abi}.");
        }

        ThrowIfFailed(
            Box3DNative.ymir_box3d_get_abi_layout(
                out var bodyInputSize,
                out var fieldInputSize,
                out var bodyOutputSize,
                out var contactOutputSize),
            "read native ABI layout");
        EnsureSize<Box3DBodyInput>(bodyInputSize);
        EnsureSize<Box3DRadialField>(fieldInputSize);
        EnsureSize<Box3DBodyOutput>(bodyOutputSize);
        EnsureSize<Box3DContactOutput>(contactOutputSize);

        Box3DNative.ymir_box3d_get_version(out var major, out var minor, out var revision);
        if (major != 0 || minor != 1 || revision != 0)
        {
            throw new InvalidOperationException($"Ymir requires Box3D 0.1.0, but loaded {major}.{minor}.{revision}.");
        }
    }

    private static void EnsureSize<T>(uint nativeSize) where T : struct
    {
        var managedSize = checked((uint)Marshal.SizeOf<T>());
        if (managedSize != nativeSize)
        {
            throw new InvalidOperationException(
                $"Ymir Box3D ABI layout mismatch for {typeof(T).Name}. Managed expects {managedSize} bytes, native provides {nativeSize}.");
        }
    }

    private static void ThrowIfFailed(Box3DStatus status, string operation)
    {
        if (status != Box3DStatus.Ok)
        {
            throw new InvalidOperationException($"Ymir could not {operation}: native Box3D status {status}.");
        }
    }

    private void ThrowIfDisposed() => ObjectDisposedException.ThrowIf(_disposed, this);
}
