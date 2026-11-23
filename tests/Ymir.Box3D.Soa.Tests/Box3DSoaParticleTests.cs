using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

/// <summary>Particle chains and attachments: the contract of docs/soa-world.md, particle section.</summary>
public sealed class Box3DSoaParticleTests
{
    private const float Dt = 1f / 120f;
    private const int Sub = 4;

    private static ulong Fnv(ulong h, ReadOnlySpan<byte> bytes)
    {
        foreach (var b in bytes) { h ^= b; h *= 1099511628211ul; }
        return h;
    }

    private static ulong HashBodyLanes(Box3DSoaView v)
    {
        var h = 1469598103934665603ul;
        void F(ReadOnlySpan<float> lane) => h = Fnv(h, MemoryMarshal.AsBytes(lane));
        F(v.PositionX); F(v.PositionY); F(v.PositionZ);
        F(v.RotationX); F(v.RotationY); F(v.RotationZ); F(v.RotationW);
        F(v.VelocityX); F(v.VelocityY); F(v.VelocityZ);
        F(v.AngularVelocityX); F(v.AngularVelocityY); F(v.AngularVelocityZ);
        F(v.ContactImpulse);
        h = Fnv(h, MemoryMarshal.AsBytes(v.ContactCount));
        return h;
    }

    /// <summary>The ABI 8 determinism scene (a slab and 25 dropped spheres), created without particle capacities.</summary>
    private static Box3DSoaWorld Abi8Scene()
    {
        var world = new Box3DSoaWorld(capacity: 64);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        ulong id = 100;
        for (var x = 0; x < 5; x++)
            for (var z = 0; z < 5; z++)
                world.AddSphere(id++, (x * 1.5f - 3f, 3f + 0.37f * ((x * 5 + z) % 7), z * 1.5f - 3f), radius: 0.5f, density: 1f,
                    velocity: (0.1f * z, 0f, -0.05f * x));
        return world;
    }

    // Recorded on the commit before any particle code existed (ABI 8, 5104b17), one hash per 50 steps at 1/60 s.
    private static readonly ulong[] Abi8Hashes = [13041258221528527995ul, 10457196283436582934ul, 7549714013947433797ul, 13409007737149865513ul, 2368270135655444881ul, 5732644473329431104ul];

    [Fact]
    public void Zero_capacity_world_is_ABI8_identical()
    {
        using var world = Abi8Scene();
        var got = new ulong[6];
        for (var c = 0; c < 6; c++)
        {
            for (var i = 0; i < 50; i++) world.Step(1f / 60f);
            got[c] = HashBodyLanes(world.View);
        }
        Assert.True(Abi8Hashes.AsSpan().SequenceEqual(got), string.Join(",", got.Select(g => g + "ul")));
    }
}
