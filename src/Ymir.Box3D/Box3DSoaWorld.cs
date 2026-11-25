using System.Buffers;
using System.Runtime.InteropServices;

namespace Ymir.Box3D;

public enum Box3DSoaKind : uint
{
    Sphere = 0,
    StaticBox = 1,
    Capsule = 2,
    Box = 3,
    /// <summary>A kinematic body with no shape: it never collides or is hit, and only <see cref="Box3DSoaWorld.SetAnchorTarget"/> moves it.</summary>
    Anchor = 4
}

public enum Box3DSoaJointKind : uint
{
    Revolute = 0,
    Spherical = 1,
    Weld = 2,
    /// <summary>Position springs pulling body B's frame onto body A's, bounded by a maximum force and torque.</summary>
    Motor = 3,
    /// <summary>The kind lane of a destroyed joint (a tombstone): bodies read -1 and every other lane 0.</summary>
    Removed = 4
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
    public ulong MaterialId;
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
    public float Lower, Upper, ConeAngle;
    public float LinearHertz, LinearDamping, MaxForce, AngularHertz, AngularDamping, MaxTorque;
}

/// <summary>Native layout of <c>ymir_box3d_soa_ray_hit</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct Box3DSoaRayHitNative
{
    public int Hit;
    public int Lane;
    public ulong StableId;
    public float PointX, PointY, PointZ;
    public float NormalX, NormalY, NormalZ;
    public float Fraction;
    public uint Reserved;
}

/// <summary>
/// A particle's identity: its lane and the slot's generation when it was made. Native layout of <c>ymir_handle</c>.
/// Converts to the lane for reading lanes; commands refuse a handle from an earlier occupant of the slot.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public readonly record struct Box3DSoaParticleHandle(int Lane, uint Generation)
{
    public static implicit operator int(Box3DSoaParticleHandle handle) => handle.Lane;
}

/// <summary>The constraint lane sets. Native numbering: a particle is set 0.</summary>
public enum Box3DSoaConstraintType : uint
{
    Distance = 1,
    Bend = 2,
    Attachment = 3
}

/// <summary>A constraint's identity: its type, lane and the slot's generation when it was made.</summary>
public readonly record struct Box3DSoaConstraintHandle(Box3DSoaConstraintType Type, int Lane, uint Generation)
{
    public static implicit operator int(Box3DSoaConstraintHandle handle) => handle.Lane;
}

/// <summary>The phase word: bits 0-19 are the group; the flags are the only other bits.</summary>
public static class Box3DSoaPhase
{
    public const uint GroupMask = 0xFFFFFu;
    /// <summary>Particles of one group collide (both must set it).</summary>
    public const uint SelfCollide = 1u << 20;
    /// <summary>...except pairs closer than 1.5 (ri + rj) in the rest pose (either may carry it).</summary>
    public const uint SelfFilter = 1u << 21;
    /// <summary>A fluid particle: number density, not contact, with other fluid; contact with solids. Fluid holds no distance, bend or attachment.</summary>
    public const uint Fluid = 1u << 22;

    public static uint Of(uint group, bool selfCollide = false, bool selfFilter = false, bool fluid = false) =>
        (group & GroupMask) | (selfCollide ? SelfCollide : 0u) | (selfFilter ? SelfFilter : 0u) | (fluid ? Fluid : 0u);
}

/// <summary>Capacities of the particle state; <c>null</c> or all zero is a world without particles (ABI 8 behaviour).</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly record struct Box3DSoaParticleCapacities(int Particles, int Distances, int Bends, int Attachments, int Tethers, int Materials, int Fluid = 0);

/// <summary>Native layout of <c>ymir_particle_desc</c>: a particle to create. Mass kg (0 = immovable); rest is its place in the rest pose.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct Box3DSoaParticleDesc
{
    public float X, Y, Z;
    public float RestX, RestY, RestZ;
    public float VelocityX, VelocityY, VelocityZ;
    public float Mass, Radius;
    public uint Phase, Material;

    /// <summary>A particle at <paramref name="position"/>, its rest pose the same place unless <paramref name="rest"/> says otherwise.</summary>
    public Box3DSoaParticleDesc((float X, float Y, float Z) position, float mass, float radius, uint phase = 0, uint material = 0, (float X, float Y, float Z)? rest = null, (float X, float Y, float Z) velocity = default)
    {
        (X, Y, Z) = position;
        (RestX, RestY, RestZ) = rest ?? position;
        (VelocityX, VelocityY, VelocityZ) = velocity;
        Mass = mass; Radius = radius; Phase = phase; Material = material;
    }
}

/// <summary>Native layout of <c>ymir_distance_desc</c>. <c>Rest</c> &lt; 0 takes the rest-pose length; compliance is m/N.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct Box3DSoaDistanceDesc
{
    public Box3DSoaParticleHandle A, B;
    public float Rest, Compliance;

    public Box3DSoaDistanceDesc(Box3DSoaParticleHandle a, Box3DSoaParticleHandle b, float compliance = 0f, float rest = -1f)
    {
        A = a; B = b; Rest = rest; Compliance = compliance;
    }
}

/// <summary>
/// Native layout of <c>ymir_bend_desc</c>: a bend over two distances that share one particle. A compliance &lt; 0 switches
/// the soft pass off, a max angle (radians from straight) &lt; 0 the hard limit.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct Box3DSoaBendDesc
{
    public int E1;
    public uint G1;
    public int E2;
    public uint G2;
    public float Compliance, MaxAngle;

    public Box3DSoaBendDesc(Box3DSoaConstraintHandle e1, Box3DSoaConstraintHandle e2, float compliance = 1e-4f, float maxAngle = -1f)
    {
        E1 = e1.Lane; G1 = e1.Generation; E2 = e2.Lane; G2 = e2.Generation; Compliance = compliance; MaxAngle = maxAngle;
    }
}

/// <summary>Native layout of <c>ymir_attachment_desc</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct Box3DSoaAttachmentDesc
{
    public uint Kind;
    public uint Flags;
    public Box3DSoaParticleHandle Particle;
    public int Body;
    public float PointX, PointY, PointZ;
    public float Compliance;
    public uint Reserved;
}

/// <summary>Native layout of <c>ymir_material_desc</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct Box3DSoaMaterialDesc
{
    public float Friction;
    public float Viscosity;
    public ulong MaterialId;
}

public enum Box3DSoaAttachKind : uint
{
    /// <summary>The particle to a point fixed in a body: <c>target</c> is the body lane, <c>point</c> is in the body's frame.</summary>
    Body = 0,
    /// <summary>The particle to a world point (<c>point</c>); <c>target</c> is ignored.</summary>
    Static = 1,
    /// <summary>The kind lane of a removed attachment (a tombstone).</summary>
    Removed = 2
}

/// <summary>
/// A joint's identity: its lane and the slot's generation when it was added. Converts to the lane for reading lanes;
/// <see cref="Box3DSoaWorld.DestroyJoint"/> takes the whole handle and refuses one from an earlier occupant of the slot.
/// </summary>
public readonly record struct Box3DSoaJointHandle(int Lane, uint Generation)
{
    public static implicit operator int(Box3DSoaJointHandle handle) => handle.Lane;
}

/// <summary>The closest hit of a ray. <see cref="Hit"/> false is a miss and every other field is then default.</summary>
public readonly record struct Box3DSoaRayHit(
    bool Hit, int Lane, ulong StableId, (float X, float Y, float Z) Point, (float X, float Y, float Z) Normal, float Fraction);

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
    public int ParticleCount;
    public int ParticleCapacity;
    public int FluidNeighbourOverflow;
    public int DistanceCount;
    public int DistanceCapacity;
    public int BendCount;
    public int BendCapacity;
    public int AttachmentCount;
    public int AttachmentCapacity;
    public int TetherCount;
    public int TetherCapacity;
    public int MaterialCapacity;
    public int ReservedParticles;
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
    public float* JointOriginX;
    public float* JointOriginY;
    public float* JointOriginZ;
    public float* ContactImpulse;
    public int* ContactCount;
    public ulong* MaterialId;
    public uint* JointGeneration;
    public float* ParticleImpulseX;
    public float* ParticleImpulseY;
    public float* ParticleImpulseZ;
    public float* ParticleX;
    public float* ParticleY;
    public float* ParticleZ;
    public float* ParticleVx;
    public float* ParticleVy;
    public float* ParticleVz;
    public float* ParticleRadius;
    public float* ParticleInvMass;
    public float* ParticleRestX;
    public float* ParticleRestY;
    public float* ParticleRestZ;
    public uint* ParticlePhase;
    public uint* ParticleMaterial;
    public uint* ParticleLive;
    public uint* ParticleGeneration;
    public ulong* ParticleRemovedAt;
    public int* ParticleComponent;
    public float* ParticleDensity;
    public int* DistanceA;
    public int* DistanceB;
    public float* DistanceRest;
    public float* DistanceCompliance;
    public float* DistanceTension;
    public uint* DistanceLive;
    public uint* DistanceGeneration;
    public ulong* DistanceRemovedAt;
    public int* BendE1;
    public int* BendE2;
    public int* BendA;
    public int* BendB;
    public int* BendC;
    public float* BendRest;
    public float* BendLimit;
    public float* BendCompliance;
    public uint* BendLive;
    public uint* BendGeneration;
    public ulong* BendRemovedAt;
    public uint* AttachmentKind;
    public uint* AttachmentFlags;
    public uint* AttachmentGeneration;
    public int* AttachmentParticle;
    public int* AttachmentBody;
    public float* AttachmentPointX;
    public float* AttachmentPointY;
    public float* AttachmentPointZ;
    public float* AttachmentCompliance;
    public float* AttachmentForceX;
    public float* AttachmentForceY;
    public float* AttachmentForceZ;
    public ulong* AttachmentRemovedAt;
    public int* TetherAtt;
    public int* TetherParticle;
    public int* TetherParent;
    public int* TetherEdge;
    public int* TetherOther;
    public float* TetherBound;
    public float* MaterialFriction;
    public ulong* MaterialTableId;
    public float* MaterialViscosity;
    public int* SupportBody;
    public int* SupportParticle;
}

/// <summary>Native layout of <c>ymir_box3d_soa_facts</c>: counts, dropped counters, then three pointers.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct Box3DSoaFactsNative
{
    public uint BeginCount;
    public uint EndCount;
    public uint HitCount;
    public uint FactCapacity;
    public uint BeginDropped;
    public uint EndDropped;
    public uint HitDropped;
    public uint Reserved;
    public Box3DSoaTouch* Begin;
    public Box3DSoaTouch* End;
    public Box3DSoaHit* Hit;
}

/// <summary>A begin or end of touch: the two bodies' lane indices and the two shapes' stable ids.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Box3DSoaTouch
{
    public readonly int LaneA;
    public readonly int LaneB;
    public readonly ulong StableIdA;
    public readonly ulong StableIdB;
}

/// <summary>A touch that approached faster than the world's hit threshold. Normal points from A to B.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Box3DSoaHit
{
    public readonly int LaneA;
    public readonly int LaneB;
    public readonly ulong StableIdA;
    public readonly ulong StableIdB;
    public readonly float PointX, PointY, PointZ;
    public readonly float NormalX, NormalY, NormalZ;
    /// <summary>Metres per second, always positive.</summary>
    public readonly float ApproachSpeed;
    /// <summary>Normal impulse (N s) the pair's contact delivered over the whole step: the exact sum of Box3D's stepNormalImpulse over the manifold of the two shapes. Normal channel only.</summary>
    public readonly float Impulse;
    public readonly ulong MaterialA;
    public readonly ulong MaterialB;
}

/// <summary>Facts past <see cref="Box3DSoaWorld"/>'s fact capacity that the last step could not keep.</summary>
public readonly record struct Box3DSoaDropped(uint Begin, uint End, uint Hit);

/// <summary>
/// What Box3D reported for the step just taken, in Box3D's event order: spans over native buffers that
/// are fixed for the world's life and rewritten by every step, so they are valid until the next
/// <see cref="Box3DSoaWorld.Step"/> and invalid once the world is disposed.
/// </summary>
public readonly unsafe ref struct Box3DSoaFacts
{
    private readonly Box3DSoaFactsNative _f;
    private readonly Box3DSoaWorld _world;

    internal Box3DSoaFacts(Box3DSoaFactsNative f, Box3DSoaWorld world)
    {
        _f = f;
        _world = world;
    }

    private ReadOnlySpan<T> Facts<T>(T* p, uint n) where T : unmanaged
    {
        _world.ThrowIfDisposed();
        return new(p, (int)n);
    }

    public ReadOnlySpan<Box3DSoaTouch> Begin => Facts(_f.Begin, _f.BeginCount);
    public ReadOnlySpan<Box3DSoaTouch> End => Facts(_f.End, _f.EndCount);
    public ReadOnlySpan<Box3DSoaHit> Hit => Facts(_f.Hit, _f.HitCount);
    public Box3DSoaDropped Dropped => new(_f.BeginDropped, _f.EndDropped, _f.HitDropped);
    public int Capacity => (int)_f.FactCapacity;
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
    internal static partial uint ymir_box3d_soa_get_facts_size();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_touch_fact_size();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_ray_hit_size();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_hit_fact_size();

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_create(int capacity, int jointCapacity, float gx, float gy, float gz, int factCapacity, float hitThreshold, Box3DSoaParticleCapacities* particles, out nint world);

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

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_get_facts(nint world, out Box3DSoaFactsNative facts);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_destroy_joint(nint world, int joint, uint generation);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_free_joint_slots(nint world);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_set_anchor_target(nint world, int lane, float* position, float* rotation, float timeStep);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_apply_impulse(nint world, int lane, float* impulse, float* point);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_cast_ray_closest(nint world, float* origin, float* translation, out Box3DSoaRayHitNative hit);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial uint ymir_box3d_soa_get_particle_desc_size(uint which);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_create_particles(nint world, int n, Box3DSoaParticleDesc* desc, Box3DSoaParticleHandle* output);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_destroy_particles(nint world, int n, Box3DSoaParticleHandle* handles);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_create_distances(nint world, int n, Box3DSoaDistanceDesc* desc, Box3DSoaParticleHandle* output);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_create_bends(nint world, int n, Box3DSoaBendDesc* desc, Box3DSoaParticleHandle* output);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_attach(nint world, in Box3DSoaAttachmentDesc desc, out Box3DSoaParticleHandle handle);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_remove(nint world, uint type, int n, Box3DSoaParticleHandle* handles);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_set_material(nint world, int index, in Box3DSoaMaterialDesc desc);

    [LibraryImport(Library)]
    [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
    internal static partial int ymir_box3d_soa_free_slots(nint world, uint laneSet);
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

    // Contact lanes, read-only, refreshed by every Step.
    /// <summary>N s: the normal impulse the body's touching contact points delivered over the whole step, exact (the sum of Box3D's stepNormalImpulse). Normal channel only. A resting load is this over dt. 0 for a body touching nothing.</summary>
    public ReadOnlySpan<float> ContactImpulse => Lane(_v.ContactImpulse, Count);
    /// <summary>Number of touching contacts.</summary>
    public ReadOnlySpan<int> ContactCount => Lane(_v.ContactCount, Count);
    /// <summary>The material id the body was added with (0 by default).</summary>
    public ReadOnlySpan<ulong> MaterialId => Lane(_v.MaterialId, Count);

    /// <summary>Per joint slot, the number of times it has been destroyed. A joint's handle is its lane and the generation read
    /// when it was added; <see cref="Box3DSoaWorld.DestroyJoint"/> refuses a handle from an earlier occupant of the slot.</summary>
    public ReadOnlySpan<uint> JointGeneration => Lane(_v.JointGeneration, JointCount);

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
    /// <summary>World position of the joint frame origin on body A (the joint anchor); at creation it is the anchor add_joint was given.</summary>
    public ReadOnlySpan<float> JointOriginX => Lane(_v.JointOriginX, JointCount);
    public ReadOnlySpan<float> JointOriginY => Lane(_v.JointOriginY, JointCount);
    public ReadOnlySpan<float> JointOriginZ => Lane(_v.JointOriginZ, JointCount);

    // Particle lanes: read-only, changed only by Step and by the particle and constraint commands. Lanes are indexed up to
    // the set's count (a high-water mark). A dead lane reads live 0 and every other lane 0 except generation and removed_at;
    // it is reused once a step has passed since its removal.
    public int ParticleCount => _v.ParticleCount;
    public int ParticleCapacity => _v.ParticleCapacity;
    /// <summary>Kernel neighbours past a fluid particle's list of 96 in the last step, summed over substeps; 0 means the density correction and XSPH saw every pair.</summary>
    public int FluidNeighbourOverflow => _v.FluidNeighbourOverflow;
    public ReadOnlySpan<float> ParticleX => Lane(_v.ParticleX, ParticleCount);
    public ReadOnlySpan<float> ParticleY => Lane(_v.ParticleY, ParticleCount);
    public ReadOnlySpan<float> ParticleZ => Lane(_v.ParticleZ, ParticleCount);
    public ReadOnlySpan<float> ParticleVx => Lane(_v.ParticleVx, ParticleCount);
    public ReadOnlySpan<float> ParticleVy => Lane(_v.ParticleVy, ParticleCount);
    public ReadOnlySpan<float> ParticleVz => Lane(_v.ParticleVz, ParticleCount);
    public ReadOnlySpan<float> ParticleRadius => Lane(_v.ParticleRadius, ParticleCount);
    public ReadOnlySpan<float> ParticleInvMass => Lane(_v.ParticleInvMass, ParticleCount);
    public ReadOnlySpan<float> ParticleRestX => Lane(_v.ParticleRestX, ParticleCount);
    public ReadOnlySpan<float> ParticleRestY => Lane(_v.ParticleRestY, ParticleCount);
    public ReadOnlySpan<float> ParticleRestZ => Lane(_v.ParticleRestZ, ParticleCount);
    public ReadOnlySpan<uint> ParticlePhase => Lane(_v.ParticlePhase, ParticleCount);
    public ReadOnlySpan<uint> ParticleMaterial => Lane(_v.ParticleMaterial, ParticleCount);
    public ReadOnlySpan<uint> ParticleLive => Lane(_v.ParticleLive, ParticleCount);
    public ReadOnlySpan<uint> ParticleGeneration => Lane(_v.ParticleGeneration, ParticleCount);
    public ReadOnlySpan<ulong> ParticleRemovedAt => Lane(_v.ParticleRemovedAt, ParticleCount);
    /// <summary>Derived: the lowest lane of the particle's component of distance-connected particles.</summary>
    public ReadOnlySpan<int> ParticleComponent => Lane(_v.ParticleComponent, ParticleCount);
    /// <summary>A fluid particle's number density over its rest density in the last substep it ran (1 is at rest), 0 for a solid: a sensor.</summary>
    public ReadOnlySpan<float> ParticleDensity => Lane(_v.ParticleDensity, ParticleCount);

    public int DistanceCount => _v.DistanceCount;
    public ReadOnlySpan<int> DistanceA => Lane(_v.DistanceA, DistanceCount);
    public ReadOnlySpan<int> DistanceB => Lane(_v.DistanceB, DistanceCount);
    public ReadOnlySpan<float> DistanceRest => Lane(_v.DistanceRest, DistanceCount);
    public ReadOnlySpan<float> DistanceCompliance => Lane(_v.DistanceCompliance, DistanceCount);
    /// <summary>N: the load the constraint carries (its own correction, the bends over it and the tether impulses routed through it), mean over the substeps. A sensor: Ymir never removes a constraint on it.</summary>
    public ReadOnlySpan<float> DistanceTension => Lane(_v.DistanceTension, DistanceCount);
    public ReadOnlySpan<uint> DistanceLive => Lane(_v.DistanceLive, DistanceCount);
    public ReadOnlySpan<uint> DistanceGeneration => Lane(_v.DistanceGeneration, DistanceCount);
    public ReadOnlySpan<ulong> DistanceRemovedAt => Lane(_v.DistanceRemovedAt, DistanceCount);

    public int BendCount => _v.BendCount;
    public ReadOnlySpan<int> BendE1 => Lane(_v.BendE1, BendCount);
    public ReadOnlySpan<int> BendE2 => Lane(_v.BendE2, BendCount);
    public ReadOnlySpan<int> BendA => Lane(_v.BendA, BendCount);
    public ReadOnlySpan<int> BendB => Lane(_v.BendB, BendCount);
    public ReadOnlySpan<int> BendC => Lane(_v.BendC, BendCount);
    /// <summary>Offsets of the shared particle from the centroid of the three, in metres: at rest and at the hard limit (negative: no limit).</summary>
    public ReadOnlySpan<float> BendRest => Lane(_v.BendRest, BendCount);
    public ReadOnlySpan<float> BendLimit => Lane(_v.BendLimit, BendCount);
    public ReadOnlySpan<float> BendCompliance => Lane(_v.BendCompliance, BendCount);
    public ReadOnlySpan<uint> BendLive => Lane(_v.BendLive, BendCount);
    public ReadOnlySpan<uint> BendGeneration => Lane(_v.BendGeneration, BendCount);
    public ReadOnlySpan<ulong> BendRemovedAt => Lane(_v.BendRemovedAt, BendCount);

    public int AttachmentCount => _v.AttachmentCount;
    public ReadOnlySpan<uint> AttachmentKind => Lane(_v.AttachmentKind, AttachmentCount);
    public ReadOnlySpan<uint> AttachmentFlags => Lane(_v.AttachmentFlags, AttachmentCount);
    public ReadOnlySpan<uint> AttachmentGeneration => Lane(_v.AttachmentGeneration, AttachmentCount);
    public ReadOnlySpan<int> AttachmentParticle => Lane(_v.AttachmentParticle, AttachmentCount);
    /// <summary>The body lane (<see cref="Box3DSoaAttachKind.Body"/>) or -1.</summary>
    public ReadOnlySpan<int> AttachmentBody => Lane(_v.AttachmentBody, AttachmentCount);
    public ReadOnlySpan<float> AttachmentPointX => Lane(_v.AttachmentPointX, AttachmentCount);
    public ReadOnlySpan<float> AttachmentPointY => Lane(_v.AttachmentPointY, AttachmentCount);
    public ReadOnlySpan<float> AttachmentPointZ => Lane(_v.AttachmentPointZ, AttachmentCount);
    public ReadOnlySpan<float> AttachmentCompliance => Lane(_v.AttachmentCompliance, AttachmentCount);
    /// <summary>N: what the attachment exerts on its particles, mean over the substeps, tether load included. A sensor.</summary>
    public ReadOnlySpan<float> AttachmentForceX => Lane(_v.AttachmentForceX, AttachmentCount);
    public ReadOnlySpan<float> AttachmentForceY => Lane(_v.AttachmentForceY, AttachmentCount);
    public ReadOnlySpan<float> AttachmentForceZ => Lane(_v.AttachmentForceZ, AttachmentCount);
    public ReadOnlySpan<ulong> AttachmentRemovedAt => Lane(_v.AttachmentRemovedAt, AttachmentCount);

    /// <summary>Derived tethers, up to <see cref="TetherCount"/>: written only by the topology commands, parents before children per attachment.</summary>
    public int TetherCount => _v.TetherCount;
    public int TetherCapacity => _v.TetherCapacity;
    public ReadOnlySpan<int> TetherAttachment => Lane(_v.TetherAtt, TetherCount);
    public ReadOnlySpan<int> TetherParticle => Lane(_v.TetherParticle, TetherCount);
    public ReadOnlySpan<int> TetherParent => Lane(_v.TetherParent, TetherCount);
    public ReadOnlySpan<int> TetherEdge => Lane(_v.TetherEdge, TetherCount);
    /// <summary>The second attachment of a pair tether (a bound between two attachment points, edge -1), or -1.</summary>
    public ReadOnlySpan<int> TetherOther => Lane(_v.TetherOther, TetherCount);
    public ReadOnlySpan<float> TetherBound => Lane(_v.TetherBound, TetherCount);

    public int MaterialCapacity => _v.MaterialCapacity;
    public ReadOnlySpan<float> MaterialFriction => Lane(_v.MaterialFriction, MaterialCapacity);
    public ReadOnlySpan<ulong> MaterialTableId => Lane(_v.MaterialTableId, MaterialCapacity);
    /// <summary>The XSPH coefficient of each material (a fluid pair relaxes its velocities by the mean of the two).</summary>
    public ReadOnlySpan<float> MaterialViscosity => Lane(_v.MaterialViscosity, MaterialCapacity);
    /// <summary>A fluid particle's support in the last substep it ran: the highest body lane it touched, else -1; -1 for a solid or free lane. Contact, not rest: the caller judges speed. Notification only.</summary>
    public ReadOnlySpan<int> SupportBody => Lane(_v.SupportBody, ParticleCount);
    /// <summary>A fluid particle's support in the last substep it ran: the first solid particle lane its row listed touching it, else -1 (resting on fluid alone reads -1); -1 for a solid or free lane.</summary>
    public ReadOnlySpan<int> SupportParticle => Lane(_v.SupportParticle, ParticleCount);

    /// <summary>N s, world: the impulse the particle phase delivered to each body over the last step (0 for a body it did not touch). Box3D has already applied it; this is notification only.</summary>
    public ReadOnlySpan<float> ParticleImpulseX => Lane(_v.ParticleImpulseX, Count);
    public ReadOnlySpan<float> ParticleImpulseY => Lane(_v.ParticleImpulseY, Count);
    public ReadOnlySpan<float> ParticleImpulseZ => Lane(_v.ParticleImpulseZ, Count);

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
    private const uint ExpectedAbi = 13;
    private nint _handle;

    /// <summary>
    /// <paramref name="factCapacity"/> bounds each kind of per-step contact fact (default 4 x capacity; 0 or less is refused);
    /// <paramref name="hitThreshold"/> is the approach speed in m/s above which a touch is a hit (Box3D's default, 1).
    /// </summary>
    public unsafe Box3DSoaWorld(
        int capacity, float gravityX = 0f, float gravityY = -10f, float gravityZ = 0f, int jointCapacity = 0,
        int? factCapacity = null, float hitThreshold = 1f, Box3DSoaParticleCapacities? particles = null)
    {
        RequireNativeAbi(Box3DSoaNative.ymir_box3d_soa_get_abi_version());
        static uint Size(uint which) => Box3DSoaNative.ymir_box3d_soa_get_particle_desc_size(which);
        if (Box3DSoaNative.ymir_box3d_soa_get_view_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaViewNative>() ||
            Box3DSoaNative.ymir_box3d_soa_get_body_desc_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaBodyDesc>() ||
            Box3DSoaNative.ymir_box3d_soa_get_joint_desc_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaJointDesc>() ||
            Box3DSoaNative.ymir_box3d_soa_get_facts_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaFactsNative>() ||
            Box3DSoaNative.ymir_box3d_soa_get_touch_fact_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaTouch>() ||
            Box3DSoaNative.ymir_box3d_soa_get_hit_fact_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaHit>() ||
            Box3DSoaNative.ymir_box3d_soa_get_ray_hit_size() != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaRayHitNative>() ||
            Size(0) != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaParticleDesc>() ||
            Size(1) != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaDistanceDesc>() ||
            Size(2) != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaBendDesc>() ||
            Size(3) != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaAttachmentDesc>() ||
            Size(4) != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaMaterialDesc>() ||
            Size(5) != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaParticleCapacities>() ||
            Size(6) != (uint)System.Runtime.CompilerServices.Unsafe.SizeOf<Box3DSoaParticleHandle>())
        {
            throw new InvalidOperationException("The native Ymir Box3D SoA ABI does not match this build.");
        }

        var caps = particles.GetValueOrDefault();
        Check(Box3DSoaNative.ymir_box3d_soa_create(capacity, jointCapacity, gravityX, gravityY, gravityZ, factCapacity ?? checked(capacity * 4), hitThreshold, particles.HasValue ? &caps : null, out _handle));
    }

    /// <summary>Fails closed when the native library is not the ABI this build expects, in either direction.</summary>
    internal static void RequireNativeAbi(uint nativeAbi)
    {
        if (nativeAbi != ExpectedAbi)
        {
            throw new InvalidOperationException($"The native Ymir Box3D SoA ABI is {nativeAbi}; this build expects {ExpectedAbi}.");
        }
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

    /// <summary>The contact facts of the last Step: empty before the first Step and rewritten by the next.</summary>
    public Box3DSoaFacts Facts
    {
        get
        {
            ObjectDisposedException.ThrowIf(_handle == 0, this);
            Check(Box3DSoaNative.ymir_box3d_soa_get_facts(_handle, out var f));
            return new Box3DSoaFacts(f, this);
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
        int parent = -1, int limb = 0, float rollingResistance = 0f, ulong materialId = 0) =>
        AddBody(Describe(Box3DSoaKind.Sphere, stableId, position, default, (radius, 0f, 0f), density,
            velocity, friction, restitution, parent, limb, rollingResistance, materialId));

    /// <summary>Adds a static axis-aligned box centred at <paramref name="position"/>.</summary>
    public int AddStaticBox(
        ulong stableId, (float X, float Y, float Z) position, (float X, float Y, float Z) halfExtents,
        float friction = 0.6f, float restitution = 0f, ulong materialId = 0) =>
        AddBody(Describe(Box3DSoaKind.StaticBox, stableId, position, default, halfExtents, 0f,
            default, friction, restitution, -1, 0, 0f, materialId));

    /// <summary>
    /// Adds a dynamic capsule: a segment of <paramref name="halfLength"/> either side of the centre along
    /// the body's local Y, swept by <paramref name="radius"/>. <paramref name="rotation"/> is a unit quaternion
    /// x y z w; the default (all zero) means identity.
    /// </summary>
    public int AddCapsule(
        ulong stableId, (float X, float Y, float Z) position, float radius, float halfLength, float density,
        (float X, float Y, float Z, float W) rotation = default, float friction = 0.6f, float restitution = 0f,
        int parent = -1, int limb = 0, float rollingResistance = 0f, ulong materialId = 0) =>
        AddBody(Describe(Box3DSoaKind.Capsule, stableId, position, rotation, (radius, halfLength, 0f), density,
            default, friction, restitution, parent, limb, rollingResistance, materialId));

    /// <summary>Adds a dynamic box with the given half extents; rotation as for <see cref="AddCapsule"/>.</summary>
    public int AddBox(
        ulong stableId, (float X, float Y, float Z) position, (float X, float Y, float Z) halfExtents, float density,
        (float X, float Y, float Z, float W) rotation = default, float friction = 0.6f, float restitution = 0f,
        int parent = -1, int limb = 0, ulong materialId = 0) =>
        AddBody(Describe(Box3DSoaKind.Box, stableId, position, rotation, halfExtents, density,
            default, friction, restitution, parent, limb, 0f, materialId));

    /// <summary>
    /// Joins two bodies at a world-space <paramref name="anchor"/>. The joint frame's z axis is
    /// <paramref name="axis"/> (the hinge axis of a revolute joint); the pose at creation is the joint's zero.
    /// Limits are radians: the revolute angle, or the spherical twist with <paramref name="coneAngle"/> as swing.
    /// A weld takes no limits. Returns the joint's handle (lane and generation). Body B is the child.
    /// </summary>
    public Box3DSoaJointHandle AddJoint(
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
        return HandleOf(index);
    }

    /// <summary>
    /// A motor joint: position springs that pull body B's frame onto body A's. The frame sits at the world-space
    /// <paramref name="anchor"/> at creation (axes aligned with the world), so the spring pulls B's anchor point onto A's.
    /// <c>Hertz</c> 0 turns that spring off; <c>MaxForce</c> (N) and <c>MaxTorque</c> (N m) bound it. All are finite and
    /// not negative. Pair an anchor (body A) with the body to grab (body B). Returns the joint's handle (lane and generation).
    /// Create the anchor only when a joint will follow (check <see cref="FreeJointSlots"/>): the joint validates its spring
    /// fields, so a grab that fails there leaves the anchor behind. One long-lived anchor per cursor is the intended use.
    /// </summary>
    public Box3DSoaJointHandle AddMotorJoint(
        int bodyA, int bodyB, (float X, float Y, float Z) anchor,
        (float Hertz, float Damping, float MaxForce) linear, (float Hertz, float Damping, float MaxTorque) angular)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        var desc = new Box3DSoaJointDesc
        {
            Kind = (uint)Box3DSoaJointKind.Motor, BodyA = bodyA, BodyB = bodyB,
            AnchorX = anchor.X, AnchorY = anchor.Y, AnchorZ = anchor.Z, AxisZ = 1f,
            LinearHertz = linear.Hertz, LinearDamping = linear.Damping, MaxForce = linear.MaxForce,
            AngularHertz = angular.Hertz, AngularDamping = angular.Damping, MaxTorque = angular.MaxTorque,
        };
        Check(Box3DSoaNative.ymir_box3d_soa_add_joint(_handle, desc, out var index));
        return HandleOf(index);
    }

    /// <summary>
    /// Releases a motor joint (a grab); the anchor and the grabbed body stay. The joint's lane becomes a tombstone:
    /// kind <see cref="Box3DSoaJointKind.Removed"/>, bodies -1, every other lane 0. It is reused by a later AddJoint only
    /// once a <see cref="Step"/> has completed, so lanes read this step stay valid. Throws for any other kind of joint
    /// (general removal is another cut's), a lane that is not a live joint, or a handle whose generation is not the slot's:
    /// a release made with a stale handle never destroys a later grab in a reused slot.
    /// </summary>
    public void DestroyJoint(Box3DSoaJointHandle joint)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        Check(Box3DSoaNative.ymir_box3d_soa_destroy_joint(_handle, joint.Lane, joint.Generation));
    }

    private unsafe Box3DSoaJointHandle HandleOf(int lane)
    {
        Check(Box3DSoaNative.ymir_box3d_soa_get_view(_handle, out var v));
        return new Box3DSoaJointHandle(lane, v.JointGeneration[lane]);
    }

    /// <summary>Joints AddJoint could create right now: unused capacity plus tombstones whose step has completed.
    /// Check it before adding an anchor for a grab, so a full world leaves no orphan anchor.</summary>
    public int FreeJointSlots
    {
        get
        {
            ObjectDisposedException.ThrowIf(_handle == 0, this);
            return Box3DSoaNative.ymir_box3d_soa_free_joint_slots(_handle);
        }
    }

    /// <summary>Bodies that can still be added. Bodies are never removed here; keep one anchor per cursor and reuse it.</summary>
    public int FreeBodySlots
    {
        get
        {
            var v = View;
            return v.Capacity - v.Count;
        }
    }

    /// <summary>Adds a kinematic anchor with no shape (rotation as for <see cref="AddCapsule"/>); returns its lane index.</summary>
    public int AddAnchor(ulong stableId, (float X, float Y, float Z) position, (float X, float Y, float Z, float W) rotation = default) =>
        AddBody(Describe(Box3DSoaKind.Anchor, stableId, position, rotation, default, 0f, default, 0f, 0f, -1, 0));

    /// <summary>
    /// Sets an anchor's pose for the coming step. Box3D gives the anchor the velocity that reaches it after
    /// <paramref name="timeStep"/>, so pass the same dt as the next <see cref="Step"/> and the anchor arrives at its end,
    /// where Step stops it: with no new target it holds still (its velocity lanes read 0).
    /// Throws for a lane that is not an anchor, a non-finite or non-unit input, or a non-positive time step.
    /// </summary>
    public unsafe void SetAnchorTarget(
        int lane, (float X, float Y, float Z) position, (float X, float Y, float Z, float W) rotation, float timeStep)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        if (rotation == default) rotation = (0f, 0f, 0f, 1f);
        var p = stackalloc float[] { position.X, position.Y, position.Z };
        var q = stackalloc float[] { rotation.X, rotation.Y, rotation.Z, rotation.W };
        Check(Box3DSoaNative.ymir_box3d_soa_set_anchor_target(_handle, lane, p, q, timeStep));
    }

    /// <summary>
    /// A one-shot world impulse (N s) at a world <paramref name="point"/>: it adds <c>impulse / mass</c> to the linear
    /// velocity and turns the body by <c>(point - centre of mass) x impulse</c>. Dynamic bodies only; throws otherwise
    /// or for non-finite input.
    /// </summary>
    public unsafe void ApplyImpulse(int lane, (float X, float Y, float Z) impulse, (float X, float Y, float Z) point)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        var j = stackalloc float[] { impulse.X, impulse.Y, impulse.Z };
        var p = stackalloc float[] { point.X, point.Y, point.Z };
        Check(Box3DSoaNative.ymir_box3d_soa_apply_impulse(_handle, lane, j, p));
    }

    /// <summary>
    /// The closest shape along <paramref name="origin"/> + t * <paramref name="translation"/>, t in [0, 1], with Box3D's
    /// default filter; anchors have no shape and are never hit. Throws for non-finite input or a zero translation.
    /// </summary>
    public unsafe Box3DSoaRayHit CastRayClosest((float X, float Y, float Z) origin, (float X, float Y, float Z) translation)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        var o = stackalloc float[] { origin.X, origin.Y, origin.Z };
        var t = stackalloc float[] { translation.X, translation.Y, translation.Z };
        Check(Box3DSoaNative.ymir_box3d_soa_cast_ray_closest(_handle, o, t, out var h));
        return new Box3DSoaRayHit(h.Hit != 0, h.Lane, h.StableId, (h.PointX, h.PointY, h.PointZ), (h.NormalX, h.NormalY, h.NormalZ), h.Fraction);
    }

    /// <summary>
    /// Creates particles in the lowest free lanes and writes their handles to <paramref name="handles"/> (empty to discard them).
    /// Throws for a full world, a bad phase or material index or non-finite input; nothing changes when it throws.
    /// </summary>
    public unsafe void CreateParticles(ReadOnlySpan<Box3DSoaParticleDesc> particles, Span<Box3DSoaParticleHandle> handles = default)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        if (!handles.IsEmpty && handles.Length < particles.Length) throw new ArgumentException("One handle per particle.", nameof(handles));
        fixed (Box3DSoaParticleDesc* d = particles)
        fixed (Box3DSoaParticleHandle* h = handles)
            Check(Box3DSoaNative.ymir_box3d_soa_create_particles(_handle, particles.Length, d, handles.IsEmpty ? null : h));
    }

    /// <summary>
    /// Destroys particles and, with them, every constraint that touches them. Their lanes are tombstones (live 0) reusable once a
    /// <see cref="Step"/> has completed. A stale handle is refused.
    /// </summary>
    public unsafe void DestroyParticles(ReadOnlySpan<Box3DSoaParticleHandle> particles)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        fixed (Box3DSoaParticleHandle* h = particles)
            Check(Box3DSoaNative.ymir_box3d_soa_destroy_particles(_handle, particles.Length, h));
    }

    /// <summary>Creates distance constraints; <paramref name="handles"/> may be empty. Throws when the derived tethers would not fit the tether capacity.</summary>
    public unsafe void CreateDistances(ReadOnlySpan<Box3DSoaDistanceDesc> distances, Span<Box3DSoaConstraintHandle> handles = default)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        if (!handles.IsEmpty && handles.Length < distances.Length) throw new ArgumentException("One handle per distance.", nameof(handles));
        Box3DSoaParticleHandle[]? rented = null;
        Span<Box3DSoaParticleHandle> native = distances.Length <= 256 ? stackalloc Box3DSoaParticleHandle[distances.Length] : (rented = ArrayPool<Box3DSoaParticleHandle>.Shared.Rent(distances.Length)).AsSpan(0, distances.Length);
        try
        {
            fixed (Box3DSoaDistanceDesc* d = distances)
            fixed (Box3DSoaParticleHandle* h = native)
                Check(Box3DSoaNative.ymir_box3d_soa_create_distances(_handle, distances.Length, d, h));
            for (var i = 0; i < handles.Length && i < native.Length; i++)
                handles[i] = new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Distance, native[i].Lane, native[i].Generation);
        }
        finally
        {
            if (rented is not null) ArrayPool<Box3DSoaParticleHandle>.Shared.Return(rented);
        }
    }

    /// <summary>Creates bends over pairs of distances that share one particle; a bend dies with either of its distances.</summary>
    public unsafe void CreateBends(ReadOnlySpan<Box3DSoaBendDesc> bends, Span<Box3DSoaConstraintHandle> handles = default)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        if (!handles.IsEmpty && handles.Length < bends.Length) throw new ArgumentException("One handle per bend.", nameof(handles));
        Box3DSoaParticleHandle[]? rented = null;
        Span<Box3DSoaParticleHandle> native = bends.Length <= 256 ? stackalloc Box3DSoaParticleHandle[bends.Length] : (rented = ArrayPool<Box3DSoaParticleHandle>.Shared.Rent(bends.Length)).AsSpan(0, bends.Length);
        try
        {
            fixed (Box3DSoaBendDesc* d = bends)
            fixed (Box3DSoaParticleHandle* h = native)
                Check(Box3DSoaNative.ymir_box3d_soa_create_bends(_handle, bends.Length, d, h));
            for (var i = 0; i < handles.Length && i < native.Length; i++)
                handles[i] = new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Bend, native[i].Lane, native[i].Generation);
        }
        finally
        {
            if (rented is not null) ArrayPool<Box3DSoaParticleHandle>.Shared.Return(rented);
        }
    }

    /// <summary>
    /// Ties <paramref name="particle"/> to a body point (kind Body: <paramref name="target"/> is the body lane, <paramref name="point"/> in
    /// its frame) or a world point (Static). With <paramref name="tether"/> the point also bounds every particle of the component by
    /// the rest path. <paramref name="compliance"/> is m/N, 0 for rigid. Attach where the particle already is.
    /// </summary>
    public Box3DSoaConstraintHandle Attach(
        Box3DSoaAttachKind kind, Box3DSoaParticleHandle particle, int target = -1, (float X, float Y, float Z) point = default,
        float compliance = 0f, bool tether = false)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        var desc = new Box3DSoaAttachmentDesc
        {
            Kind = (uint)kind, Flags = tether ? 1u : 0u, Particle = particle, Body = target,
            PointX = point.X, PointY = point.Y, PointZ = point.Z, Compliance = compliance,
        };
        Check(Box3DSoaNative.ymir_box3d_soa_attach(_handle, desc, out var handle));
        return new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Attachment, handle.Lane, handle.Generation);
    }

    /// <summary>Removes constraints of one type (a distance takes the bends over it with it). A stale handle is refused.</summary>
    public unsafe void Remove(ReadOnlySpan<Box3DSoaConstraintHandle> constraints)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        if (constraints.IsEmpty) return;
        var type = constraints[0].Type;
        Box3DSoaParticleHandle[]? rented = null;
        Span<Box3DSoaParticleHandle> native = constraints.Length <= 256 ? stackalloc Box3DSoaParticleHandle[constraints.Length] : (rented = ArrayPool<Box3DSoaParticleHandle>.Shared.Rent(constraints.Length)).AsSpan(0, constraints.Length);
        try
        {
            for (var i = 0; i < constraints.Length; i++)
            {
                if (constraints[i].Type != type) throw new ArgumentException("One type per call.", nameof(constraints));
                native[i] = new Box3DSoaParticleHandle(constraints[i].Lane, constraints[i].Generation);
            }
            fixed (Box3DSoaParticleHandle* h = native)
                Check(Box3DSoaNative.ymir_box3d_soa_remove(_handle, (uint)type, constraints.Length, h));
        }
        finally
        {
            if (rented is not null) ArrayPool<Box3DSoaParticleHandle>.Shared.Return(rented);
        }
    }

    /// <summary>Removes one constraint.</summary>
    public void Remove(Box3DSoaConstraintHandle constraint) => Remove([constraint]);

    /// <summary>
    /// Sets a material table entry: the friction particles of that material have against bodies, the caller's id for it and the
    /// XSPH viscosity coefficient (dimensionless, &gt;= 0) fluid particles of it relax their velocities by.
    /// </summary>
    public void SetMaterial(int index, float friction, ulong materialId = 0, float viscosity = 0f)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        Check(Box3DSoaNative.ymir_box3d_soa_set_material(_handle, index, new Box3DSoaMaterialDesc { Friction = friction, Viscosity = viscosity, MaterialId = materialId }));
    }

    /// <summary>Lanes a new particle could take right now (never used, or dead for a step).</summary>
    public int FreeParticleSlots { get { ObjectDisposedException.ThrowIf(_handle == 0, this); return Box3DSoaNative.ymir_box3d_soa_free_slots(_handle, 0); } }
    public int FreeDistanceSlots => FreeSlots(Box3DSoaConstraintType.Distance);
    public int FreeBendSlots => FreeSlots(Box3DSoaConstraintType.Bend);
    public int FreeAttachmentSlots => FreeSlots(Box3DSoaConstraintType.Attachment);

    private int FreeSlots(Box3DSoaConstraintType type)
    {
        ObjectDisposedException.ThrowIf(_handle == 0, this);
        return Box3DSoaNative.ymir_box3d_soa_free_slots(_handle, (uint)type);
    }

    private static Box3DSoaBodyDesc Describe(
        Box3DSoaKind kind, ulong stableId, (float X, float Y, float Z) position,
        (float X, float Y, float Z, float W) rotation, (float, float, float) size, float density,
        (float X, float Y, float Z) velocity, float friction, float restitution, int parent, int limb, float rollingResistance = 0f, ulong materialId = 0)
    {
        if (rotation == default) rotation = (0f, 0f, 0f, 1f);
        return new Box3DSoaBodyDesc
        {
            StableId = stableId, Kind = (uint)kind, Parent = parent, Limb = limb,
            PositionX = position.X, PositionY = position.Y, PositionZ = position.Z,
            RotationX = rotation.X, RotationY = rotation.Y, RotationZ = rotation.Z, RotationW = rotation.W,
            VelocityX = velocity.X, VelocityY = velocity.Y, VelocityZ = velocity.Z,
            Size0 = size.Item1, Size1 = size.Item2, Size2 = size.Item3,
            Density = density, Friction = friction, Restitution = restitution, RollingResistance = rollingResistance, MaterialId = materialId,
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
