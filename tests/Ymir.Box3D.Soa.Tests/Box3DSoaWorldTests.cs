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
    public void Each_kind_reads_back_its_size_lanes_at_its_lane_index_and_step_never_changes_them()
    {
        // All four kinds interleaved, so a wrong lane-major offset would read one kind's size from another's slot.
        using var world = new Box3DSoaWorld(capacity: 16);
        var ground = world.AddStaticBox(1, (0f, -0.5f, 0f), (40f, 0.5f, 30f));
        var sphere = world.AddSphere(2, (0f, 4f, 0f), radius: 0.7f, density: 1f);
        var capsule = world.AddCapsule(3, (3f, 4f, 0f), radius: 0.3f, halfLength: 0.9f, density: 1f);
        var box = world.AddBox(4, (-3f, 4f, 0f), (0.4f, 0.6f, 0.8f), density: 1f);
        var wall = world.AddStaticBox(5, (0f, 2f, 20f), (2f, 3f, 0.25f));
        var sphere2 = world.AddSphere(6, (6f, 4f, 0f), radius: 0.2f, density: 1f);

        (float, float, float)[] expected = new (float, float, float)[6];
        expected[ground] = (40f, 0.5f, 30f);
        expected[sphere] = (0.7f, 0f, 0f);
        expected[capsule] = (0.3f, 0.9f, 0f);
        expected[box] = (0.4f, 0.6f, 0.8f);
        expected[wall] = (2f, 3f, 0.25f);
        expected[sphere2] = (0.2f, 0f, 0f);

        var view = world.View;
        byte[] Sizes(Box3DSoaView v) =>
            MemoryMarshal.AsBytes(v.Size0).ToArray().Concat(MemoryMarshal.AsBytes(v.Size1).ToArray())
                .Concat(MemoryMarshal.AsBytes(v.Size2).ToArray()).ToArray();
        void AssertSizes(Box3DSoaView v)
        {
            Assert.Equal(6, v.Size0.Length);
            for (var i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], (v.Size0[i], v.Size1[i], v.Size2[i]));
            }
        }

        AssertSizes(view);
        var before = Sizes(view);
        Run(world, 120);
        AssertSizes(world.View);
        Assert.True(before.AsSpan().SequenceEqual(Sizes(world.View)));
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

    [Fact]
    public void A_view_taken_before_dispose_refuses_lane_access_after()
    {
        var world = Build();
        var view = world.View;
        Assert.True(view.PositionY.Length > 0);
        world.Dispose();
        Assert.Throws<ObjectDisposedException>(() => view.PositionY.Length);
        Assert.Throws<ObjectDisposedException>(() => view.ForceX.Length);
        Assert.Throws<ObjectDisposedException>(() => view.JointAngle.Length);
    }

    [Fact]
    public void A_default_view_refuses_lane_access()
    {
        var view = default(Box3DSoaView);
        Assert.Throws<InvalidOperationException>(() => view.PositionY.Length);
    }

    // Box3D's own mass is the owner; the formulas below live here only as the oracle of it.
    [Fact]
    public void Mass_is_density_times_the_shapes_volume()
    {
        const float rho = 800f, r = 0.3f, h = 0.45f;
        using var world = new Box3DSoaWorld(capacity: 4);
        var sphere = world.AddSphere(1, (0f, 1f, 0f), r, rho);
        var capsule = world.AddCapsule(2, (3f, 1f, 0f), r, h, rho);
        var box = world.AddBox(3, (6f, 1f, 0f), (0.2f, 0.3f, 0.5f), rho);
        var view = world.View;
        Assert.Equal(rho * 4f / 3f * MathF.PI * r * r * r, view.Mass[sphere], rho * 4f / 3f * MathF.PI * r * r * r * 1e-5f);
        var capsuleVolume = MathF.PI * r * r * 2f * h + 4f / 3f * MathF.PI * r * r * r;
        Assert.Equal(rho * capsuleVolume, view.Mass[capsule], rho * capsuleVolume * 1e-5f);
        var boxVolume = 8f * 0.2f * 0.3f * 0.5f;
        Assert.Equal(rho * boxVolume, view.Mass[box], rho * boxVolume * 1e-5f);
    }

    [Fact]
    public void Static_and_removed_lanes_read_zero_mass()
    {
        using var world = new Box3DSoaWorld(capacity: 4);
        var ground = world.AddStaticBox(1, (0f, -0.5f, 0f), (5f, 0.5f, 5f));
        var ball = world.AddSphere(2, (0f, 2f, 0f), 0.5f, 1000f);
        var view = world.View;
        Assert.Equal(0f, view.Mass[ground]);
        Assert.True(view.Mass[ball] > 0f);
        world.RemoveBody(ball);
        Assert.Equal(0f, view.Mass[ball]);
    }

    [Fact]
    public void A_hung_capsules_joint_carries_mass_times_gravity()
    {
        using var world = new Box3DSoaWorld(capacity: 4, jointCapacity: 2);
        var pin = world.AddStaticBox(1, (0f, 5f, 0f), (0.1f, 0.1f, 0.1f));
        var capsule = world.AddCapsule(2, (0f, 4f, 0f), 0.05f, 0.4f, 1000f);
        var joint = world.AddJoint(Box3DSoaJointKind.Spherical, pin, capsule, (0f, 5f, 0f), (0f, 0f, 1f));
        var view = world.View;
        for (var i = 0; i < 180; i++) world.Step(Dt);
        var g = world.Gravity;
        var weight = view.Mass[capsule] * MathF.Sqrt(g.X * g.X + g.Y * g.Y + g.Z * g.Z);
        var load = MathF.Sqrt(view.JointConstraintForceX[joint] * view.JointConstraintForceX[joint] +
                              view.JointConstraintForceY[joint] * view.JointConstraintForceY[joint] +
                              view.JointConstraintForceZ[joint] * view.JointConstraintForceZ[joint]);
        Assert.Equal(weight, load, weight * 0.01f);
    }

    [Fact]
    public void Gravity_reads_back_the_constructor_arguments()
    {
        using (var world = new Box3DSoaWorld(capacity: 1)) Assert.Equal((0f, -10f, 0f), world.Gravity);
        using (var world = new Box3DSoaWorld(capacity: 1, gravityX: 0f, gravityY: 0f, gravityZ: 0f)) Assert.Equal((0f, 0f, 0f), world.Gravity);
        using (var world = new Box3DSoaWorld(capacity: 1, gravityX: 1f, gravityY: -9.81f, gravityZ: 2f)) Assert.Equal((1f, -9.81f, 2f), world.Gravity);
    }
}
