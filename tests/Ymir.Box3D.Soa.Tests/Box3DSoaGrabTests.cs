using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

/// <summary>Kinematic anchors, the motor joint, point impulses and the closest-hit ray: the input plumbing for grab and poke.</summary>
public sealed class Box3DSoaGrabTests
{
    private const float Dt = 1f / 60f;
    private const float Gravity = 10f;
    private const float Radius = 0.2f;
    private const float Density = 500f;
    private static readonly float Mass = Density * 4f / 3f * MathF.PI * Radius * Radius * Radius;

    private static float Distance(Box3DSoaView v, int lane, (float X, float Y, float Z) p)
    {
        var dx = v.PositionX[lane] - p.X; var dy = v.PositionY[lane] - p.Y; var dz = v.PositionZ[lane] - p.Z;
        return MathF.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    /// <summary>A sphere hung from an anchor at the same point by a motor joint with the given linear spring.</summary>
    private static (Box3DSoaWorld World, int Anchor, int Sphere, int Joint) Grab(float maxForce, float hertz = 5f)
    {
        var world = new Box3DSoaWorld(capacity: 4, gravityY: -Gravity, jointCapacity: 2);
        var sphere = world.AddSphere(1, (0f, 5f, 0f), Radius, Density);
        var anchor = world.AddAnchor(2, (0f, 5f, 0f));
        var joint = world.AddMotorJoint(anchor, sphere, (0f, 5f, 0f), (hertz, 0.7f, maxForce), (0f, 0f, 0f));
        return (world, anchor, sphere, joint);
    }

    [Fact]
    public void A_strong_grab_follows_its_target_and_the_anchor_shows_its_pose()
    {
        var (world, anchor, sphere, joint) = Grab(maxForce: 20f * Mass * Gravity);
        using var _ = world;
        var view = world.View;
        Assert.Equal((uint)Box3DSoaKind.Anchor, view.Kind[anchor]);
        Assert.Equal((uint)Box3DSoaJointKind.Motor, view.JointKind[joint]);
        var target = (1f, 5f, 0f);
        for (var i = 0; i < 60; i++)
        {
            world.SetAnchorTarget(anchor, target, default, Dt);
            world.Step(Dt);
        }
        Assert.True(Distance(view, sphere, target) < 0.02f, $"sphere {Distance(view, sphere, target)} m from the target");
        Assert.Equal(target.Item1, view.PositionX[anchor], 1e-3f); // the anchor reaches the pose at the end of the step
        Assert.Equal(0f, view.JointAngle[joint]);
        Assert.Equal(0f, view.Size0[anchor]); // no shape, so no size
    }

    [Fact]
    public void A_weak_grab_is_bounded_by_its_max_force()
    {
        var maxForce = 0.5f * Mass * Gravity;
        var (world, anchor, sphere, _) = Grab(maxForce);
        using var _ = world;
        var view = world.View;
        var target = (0f, 6f, 0f);
        for (var i = 0; i < 60; i++)
        {
            world.SetAnchorTarget(anchor, target, default, Dt);
            world.Step(Dt);
        }
        // Whatever the spring does, it adds at most maxForce / m upward: the fall is at least that of a body
        // under g - F/m = 5 m/s^2, 2.5 m in a second. A spring that exceeded its bound would hold it higher.
        Assert.True(5f - view.PositionY[sphere] >= 2.4f, $"fell only {5f - view.PositionY[sphere]} m");
        Assert.True(view.PositionY[sphere] < target.Item2 - 1f, "the sphere hangs below the target and never reaches it");
    }

    [Fact]
    public void Anchors_do_not_collide_and_are_never_hit_by_a_ray()
    {
        static float DropY(bool withAnchor)
        {
            using var world = new Box3DSoaWorld(capacity: 3, gravityY: -Gravity);
            var sphere = world.AddSphere(1, (0f, 3f, 0f), Radius, Density);
            if (withAnchor) world.AddAnchor(2, (0f, 1.5f, 0f));
            var view = world.View;
            for (var i = 0; i < 60; i++) world.Step(Dt);
            return view.PositionY[sphere];
        }

        Assert.True(DropY(false) < 1f, "the sphere fell past y = 1.5");
        Assert.Equal(BitConverter.SingleToInt32Bits(DropY(false)), BitConverter.SingleToInt32Bits(DropY(true)));

        using var w = new Box3DSoaWorld(capacity: 2);
        w.AddAnchor(2, (0f, 1.5f, 0f));
        Assert.False(w.CastRayClosest((0f, 3f, 0f), (0f, -3f, 0f)).Hit);
    }

    [Fact]
    public void The_force_lanes_never_move_an_anchor()
    {
        using var world = new Box3DSoaWorld(capacity: 2, gravityY: -Gravity);
        var anchor = world.AddAnchor(1, (1f, 2f, 3f));
        var view = world.View;
        for (var i = 0; i < 30; i++)
        {
            view.ForceY[anchor] = 1e6f;
            view.TorqueZ[anchor] = 1e6f;
            world.Step(Dt);
        }
        Assert.Equal((1f, 2f, 3f), (view.PositionX[anchor], view.PositionY[anchor], view.PositionZ[anchor]));
        Assert.Equal(1f, view.RotationW[anchor]);
    }

    [Fact]
    public void An_off_centre_impulse_turns_the_body()
    {
        const float r = 0.25f, density = 100f;
        var mass = density * 4f / 3f * MathF.PI * r * r * r;
        using var world = new Box3DSoaWorld(capacity: 2, gravityY: 0f);
        var sphere = world.AddSphere(1, (0f, 0f, 0f), r, density);
        var centred = world.AddSphere(2, (5f, 0f, 0f), r, density);
        var view = world.View;
        const float j = 2f;
        world.ApplyImpulse(sphere, (j, 0f, 0f), (0f, 0.1f, 0f));
        world.ApplyImpulse(centred, (j, 0f, 0f), (5f, 0f, 0f));
        world.Step(Dt);
        Assert.Equal(j / mass, view.VelocityX[sphere], 1e-4f);
        Assert.Equal(0f, view.VelocityY[sphere], 1e-4f);
        // r x J = (0, 0.1, 0) x (j, 0, 0) = (0, 0, -0.1 j): the spin is about -z, I = 2/5 m r^2.
        Assert.True(view.AngularVelocityZ[sphere] < 0f);
        Assert.Equal(-0.1f * j / (0.4f * mass * r * r), view.AngularVelocityZ[sphere], 0.02f);
        Assert.Equal(0f, view.AngularVelocityX[sphere], 1e-4f);
        // The same impulse through the centre of mass gives no spin.
        Assert.Equal(j / mass, view.VelocityX[centred], 1e-4f);
        Assert.Equal(0f, view.AngularVelocityZ[centred], 1e-4f);
    }

    [Fact]
    public void The_ray_finds_the_nearest_body_with_its_lane_and_stable_id()
    {
        using var world = new Box3DSoaWorld(capacity: 4, gravityY: 0f);
        var far = world.AddSphere(12, (6f, 0f, 0f), 0.5f, 1f);
        var near = world.AddSphere(11, (3f, 0f, 0f), 0.5f, 1f);
        var hit = world.CastRayClosest((0f, 0f, 0f), (10f, 0f, 0f));
        Assert.True(hit.Hit);
        Assert.Equal(near, hit.Lane);
        Assert.NotEqual(far, hit.Lane);
        Assert.Equal(11ul, hit.StableId);
        Assert.Equal(2.5f, hit.Point.X, 1e-3f);
        Assert.Equal(0.25f, hit.Fraction, 1e-4f);
        Assert.Equal(-1f, hit.Normal.X, 1e-3f);
        Assert.False(world.CastRayClosest((0f, 0f, 0f), (0f, 10f, 0f)).Hit);
        Assert.Equal(default, world.CastRayClosest((0f, 0f, 0f), (0f, 10f, 0f)));
    }

    [Fact]
    public void Refused_calls_throw_and_change_nothing()
    {
        static Box3DSoaWorld Build(out int anchor, out int sphere, out int box)
        {
            var w = new Box3DSoaWorld(capacity: 5, gravityY: -Gravity, jointCapacity: 2);
            sphere = w.AddSphere(1, (0f, 2f, 0f), Radius, Density);
            box = w.AddStaticBox(2, (0f, -1f, 0f), (5f, 1f, 5f));
            anchor = w.AddAnchor(3, (0f, 2f, 0f));
            return w;
        }

        using var world = Build(out var anchor, out var sphere, out var box);
        using var control = Build(out _, out _, out _);
        var nan = float.NaN;
        Assert.Throws<InvalidOperationException>(() => world.SetAnchorTarget(sphere, (1f, 1f, 1f), default, Dt));
        Assert.Throws<InvalidOperationException>(() => world.SetAnchorTarget(box, (1f, 1f, 1f), default, Dt));
        Assert.Throws<InvalidOperationException>(() => world.SetAnchorTarget(9, (1f, 1f, 1f), default, Dt));
        Assert.Throws<InvalidOperationException>(() => world.SetAnchorTarget(anchor, (nan, 1f, 1f), default, Dt));
        Assert.Throws<InvalidOperationException>(() => world.SetAnchorTarget(anchor, (1f, 1f, 1f), (0f, 0f, nan, 1f), Dt));
        Assert.Throws<InvalidOperationException>(() => world.SetAnchorTarget(anchor, (1f, 1f, 1f), (0f, 0f, 0f, 2f), Dt));
        Assert.Throws<InvalidOperationException>(() => world.SetAnchorTarget(anchor, (1f, 1f, 1f), default, 0f));
        Assert.Throws<InvalidOperationException>(() => world.ApplyImpulse(box, (1f, 0f, 0f), (0f, 0f, 0f)));
        Assert.Throws<InvalidOperationException>(() => world.ApplyImpulse(anchor, (1f, 0f, 0f), (0f, 2f, 0f)));
        Assert.Throws<InvalidOperationException>(() => world.ApplyImpulse(sphere, (nan, 0f, 0f), (0f, 2f, 0f)));
        Assert.Throws<InvalidOperationException>(() => world.ApplyImpulse(sphere, (1f, 0f, 0f), (0f, float.PositiveInfinity, 0f)));
        Assert.Throws<InvalidOperationException>(() => world.CastRayClosest((nan, 0f, 0f), (1f, 0f, 0f)));
        Assert.Throws<InvalidOperationException>(() => world.CastRayClosest((0f, 0f, 0f), (0f, 0f, 0f)));
        foreach (var bad in new[] { -1f, nan, float.PositiveInfinity })
        {
            Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(anchor, sphere, (0f, 2f, 0f), (bad, 0.7f, 100f), (0f, 0f, 0f)));
            Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(anchor, sphere, (0f, 2f, 0f), (5f, bad, 100f), (0f, 0f, 0f)));
            Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(anchor, sphere, (0f, 2f, 0f), (5f, 0.7f, bad), (0f, 0f, 0f)));
            Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(anchor, sphere, (0f, 2f, 0f), (5f, 0.7f, 100f), (bad, 0f, 0f)));
            Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(anchor, sphere, (0f, 2f, 0f), (5f, 0.7f, 100f), (1f, bad, 0f)));
            Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(anchor, sphere, (0f, 2f, 0f), (5f, 0.7f, 100f), (1f, 0f, bad)));
        }
        Assert.Equal(0, world.View.JointCount);

        for (var i = 0; i < 30; i++) { world.Step(Dt); control.Step(Dt); }
        Assert.Equal(Snapshot(control), Snapshot(world));
    }

    [Fact]
    public void Two_identical_worlds_with_the_same_grab_and_impulses_match_bit_for_bit()
    {
        var a = Snapshot(RunGrab(0f));
        Assert.Equal(a, Snapshot(RunGrab(0f)));
        Assert.NotEqual(a, Snapshot(RunGrab(MathF.BitIncrement(1f) - 1f))); // one ulp of impulse is seen
    }

    private static Box3DSoaWorld RunGrab(float impulseNudge)
    {
        var world = new Box3DSoaWorld(capacity: 5, gravityY: -Gravity, jointCapacity: 2);
        var slab = world.AddStaticBox(1, (0f, -1f, 0f), (10f, 1f, 10f));
        var ball = world.AddSphere(2, (0f, 1f, 0f), Radius, Density);
        var other = world.AddSphere(3, (1.5f, 0.2f, 0f), Radius, Density);
        var anchor = world.AddAnchor(4, (0f, 1f, 0f));
        world.AddMotorJoint(anchor, ball, (0f, 1f, 0f), (4f, 0.5f, 8f * Mass * Gravity), (2f, 0.5f, 5f));
        for (var i = 0; i < 300; i++)
        {
            var t = i * Dt;
            world.SetAnchorTarget(anchor, (MathF.Sin(t) * 1.5f, 1f + 0.5f * MathF.Cos(2f * t), 0.3f * t), default, Dt);
            if (i == 100) world.ApplyImpulse(other, (0.5f + impulseNudge, 0.2f, 0f), (1.5f, 0.3f, 0f));
            world.Step(Dt);
        }
        _ = slab;
        return world;
    }

    /// <summary>Every state lane of the world as bytes, disposing the world.</summary>
    private static byte[] Snapshot(Box3DSoaWorld world)
    {
        using var _ = world;
        var v = world.View;
        var bytes = new List<byte>();
        void Add(ReadOnlySpan<float> lane) => bytes.AddRange(MemoryMarshal.AsBytes(lane).ToArray());
        Add(v.PositionX); Add(v.PositionY); Add(v.PositionZ);
        Add(v.RotationX); Add(v.RotationY); Add(v.RotationZ); Add(v.RotationW);
        Add(v.VelocityX); Add(v.VelocityY); Add(v.VelocityZ);
        Add(v.AngularVelocityX); Add(v.AngularVelocityY); Add(v.AngularVelocityZ);
        for (var j = 0; j < v.JointCount; j++) bytes.AddRange(BitConverter.GetBytes(v.JointRotationW[j]));
        return [.. bytes];
    }
}
