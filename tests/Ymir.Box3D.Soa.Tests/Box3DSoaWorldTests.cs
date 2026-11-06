using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

public sealed unsafe class Box3DSoaWorldTests
{
    private const float Dt = 1f / 60f;
    private const ulong GroundId = 1;

    /// <summary>A ground slab whose top face is y = 0 and a 5 x 5 grid of unit spheres dropped from varied heights.</summary>
    private static Box3DSoaWorld Build(float firstDropHeight = 3f)
    {
        var world = new Box3DSoaWorld(capacity: 64);
        world.AddStaticBox(GroundId, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        ulong id = 100;
        for (var x = 0; x < 5; x++)
        {
            for (var z = 0; z < 5; z++)
            {
                var h = x == 0 && z == 0 ? firstDropHeight : 3f + 0.37f * ((x * 5 + z) % 7);
                world.AddSphere(id++, (x * 1.5f - 3f, h, z * 1.5f - 3f), radius: 0.5f, density: 1f,
                    velocity: (0.1f * z, 0f, -0.05f * x));
            }
        }
        return world;
    }

    private static void Run(Box3DSoaWorld world, int steps)
    {
        for (var i = 0; i < steps; i++) world.Step(Dt);
    }

    private static byte[] Lanes(Box3DSoaView v)
    {
        // Every lane, byte for byte, in lane order. Spans are read in place; only this
        // comparison buffer copies.
        var parts = new List<byte[]>();
        void Add(ReadOnlySpan<float> lane) => parts.Add(MemoryMarshal.AsBytes(lane).ToArray());
        parts.Add(MemoryMarshal.AsBytes(v.StableId).ToArray());
        parts.Add(MemoryMarshal.AsBytes(v.Kind).ToArray());
        Add(v.PositionX); Add(v.PositionY); Add(v.PositionZ);
        Add(v.RotationX); Add(v.RotationY); Add(v.RotationZ); Add(v.RotationW);
        Add(v.VelocityX); Add(v.VelocityY); Add(v.VelocityZ);
        Add(v.AngularVelocityX); Add(v.AngularVelocityY); Add(v.AngularVelocityZ);
        return parts.SelectMany(p => p).ToArray();
    }

    [Fact]
    public void A_body_falls_under_gravity_and_rests_on_the_ground_read_in_place()
    {
        using var world = Build();
        var view = world.View; // taken once: every read below goes through this same view
        const int sphere = 1; // lane 0 is the ground slab, lane 1 the first sphere, dropped from y = 3
        Assert.Equal((uint)Box3DSoaKind.StaticBox, view.Kind[0]);
        Assert.Equal((uint)Box3DSoaKind.Sphere, view.Kind[sphere]);
        Assert.Equal(26, view.Count);
        Assert.Equal(3f, view.PositionY[sphere]);

        Run(world, 10);
        Assert.True(view.PositionY[sphere] < 3f, "falls under gravity");
        Assert.True(view.VelocityY[sphere] < -1f, "accelerates downward");
        Assert.Equal(10ul, world.StepCount);

        Run(world, 350);
        // Rests on the slab top (y = 0): centre one radius above, essentially still.
        Assert.InRange(view.PositionY[sphere], 0.5f - 0.02f, 0.5f + 0.02f);
        Assert.True(MathF.Abs(view.VelocityY[sphere]) < 0.05f);
        for (var i = 1; i < view.Count; i++)
        {
            Assert.InRange(view.PositionY[i], 0.45f, 0.55f + 0.02f);
        }

        // The ground did not move and its quaternion is still identity.
        Assert.Equal(-0.5f, view.PositionY[0]);
        Assert.Equal(1f, view.RotationW[0]);
    }

    [Fact]
    public void The_view_is_read_in_place_with_no_copy()
    {
        using var world = Build();
        var before = world.View;
        fixed (float* y0 = before.PositionY)
        {
            Assert.Equal(before.PositionYAddress, (nint)y0); // the span starts at the native lane
        }
        var heldFirst = before.PositionY[1];

        Run(world, 30);

        var after = world.View;
        Assert.Equal(before.PositionYAddress, after.PositionYAddress); // same memory after stepping
        // The view taken before the step shows the stepped state: it is not a snapshot.
        Assert.NotEqual(heldFirst, before.PositionY[1]);
        Assert.Equal(after.PositionY[1], before.PositionY[1]);
        Assert.Equal(30ul, world.StepCount);
    }

    [Fact]
    public void Two_runs_with_the_same_inputs_match_bit_for_bit_at_every_checkpoint()
    {
        using var a = Build();
        using var b = Build();
        for (var checkpoint = 0; checkpoint < 6; checkpoint++)
        {
            Run(a, 50);
            Run(b, 50);
            Assert.True(Lanes(a.View).AsSpan().SequenceEqual(Lanes(b.View)), $"diverged by step {50 * (checkpoint + 1)}");
        }
        // The spheres collided with each other and the ground on the way: the lanes are not trivially equal zeros.
        Assert.Contains(a.View.VelocityX.ToArray(), v => v != 0f);
    }

    [Fact]
    public void The_bit_comparison_can_fail_one_input_one_ulp_apart_diverges()
    {
        using var a = Build();
        using var b = Build(firstDropHeight: BitConverter.UInt32BitsToSingle(BitConverter.SingleToUInt32Bits(3f) + 1));
        Run(a, 300);
        Run(b, 300);
        Assert.False(Lanes(a.View).AsSpan().SequenceEqual(Lanes(b.View)));
    }

    [Fact]
    public void Refuses_bad_input_and_a_full_or_duplicate_world()
    {
        using var world = new Box3DSoaWorld(capacity: 2);
        world.AddStaticBox(1, (0f, 0f, 0f), (1f, 1f, 1f));
        Assert.Throws<InvalidOperationException>(() => world.AddStaticBox(1, (0f, 0f, 0f), (1f, 1f, 1f))); // duplicate id
        world.AddSphere(2, (0f, 5f, 0f), 0.5f, 1f);
        Assert.Throws<InvalidOperationException>(() => world.AddSphere(3, (0f, 5f, 0f), 0.5f, 1f)); // capacity
        Assert.Throws<InvalidOperationException>(() => world.Step(0f));
        Assert.Throws<InvalidOperationException>(() => world.Step(float.NaN));
        using var other = new Box3DSoaWorld(capacity: 2);
        Assert.Throws<InvalidOperationException>(() => other.AddSphere(1, (float.NaN, 0f, 0f), 0.5f, 1f));
        Assert.Throws<InvalidOperationException>(() => other.AddSphere(1, (0f, 0f, 0f), 0f, 1f));
        Assert.Throws<InvalidOperationException>(() => new Box3DSoaWorld(capacity: 0));
    }

    [Fact]
    public void A_disposed_world_refuses_use()
    {
        var world = Build();
        world.Dispose();
        world.Dispose();
        Assert.Throws<ObjectDisposedException>(() => world.Step(Dt));
        Assert.Throws<ObjectDisposedException>(() => world.View);
    }
}
