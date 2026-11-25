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
    public void An_anchor_reaches_its_target_at_the_end_of_the_very_next_step()
    {
        using var world = new Box3DSoaWorld(capacity: 1, gravityY: 0f);
        var anchor = world.AddAnchor(1, (0f, 0f, 0f));
        var view = world.View;
        world.SetAnchorTarget(anchor, (1f, 2f, 0f), default, Dt);
        world.Step(Dt);
        Assert.Equal(1f, view.PositionX[anchor], 1e-3f);
        Assert.Equal(2f, view.PositionY[anchor], 1e-3f);
    }

    [Fact]
    public void An_anchor_holds_still_once_it_arrives_and_a_skipped_frame_flings_nothing()
    {
        using var world = new Box3DSoaWorld(capacity: 3, gravityY: 0f, jointCapacity: 1);
        var sphere = world.AddSphere(1, (0f, 0f, 0f), Radius, Density);
        var anchor = world.AddAnchor(2, (0f, 0f, 0f));
        world.AddMotorJoint(anchor, sphere, (0f, 0f, 0f), (5f, 0.7f, 1e6f), (0f, 0f, 0f));
        var view = world.View;
        world.SetAnchorTarget(anchor, (1f, 0f, 0f), default, Dt); // one target, 1 m away, then the caller sets no more
        for (var i = 0; i < 150; i++) world.Step(Dt);
        Assert.Equal(1f, view.PositionX[anchor], 1e-4f);
        Assert.Equal(0f, view.VelocityX[anchor]);
        Assert.Equal(1f, view.PositionX[sphere], 0.02f);
        Assert.True(MathF.Abs(view.VelocityX[sphere]) < 0.05f, "the grabbed body is not flung");
    }

    [Fact]
    public void A_grab_can_be_released_and_repeated_without_the_slots_growing()
    {
        using var world = new Box3DSoaWorld(capacity: 4, gravityY: -Gravity, jointCapacity: 1);
        var a = world.AddSphere(1, (0f, 5f, 0f), Radius, Density);
        var b = world.AddSphere(2, (3f, 5f, 0f), Radius, Density);
        var anchor = world.AddAnchor(3, (0f, 5f, 0f));
        var view = world.View;
        var strong = 20f * Mass * Gravity;
        for (var cycle = 0; cycle < 1000; cycle++)
        {
            var grabbed = cycle % 2 == 0 ? a : b;
            var at = (view.PositionX[grabbed], view.PositionY[grabbed], view.PositionZ[grabbed]);
            world.SetAnchorTarget(anchor, at, default, Dt);
            world.Step(Dt); // the anchor is on the body before the joint is made
            Assert.Equal(1, world.FreeJointSlots);
            var joint = world.AddMotorJoint(anchor, grabbed, at, (5f, 0.7f, strong), (0f, 0f, 0f));
            Assert.Equal(0, joint);
            world.Step(Dt);
            Assert.Equal(at.Item2, world.View.JointOriginY[joint], 0.05f); // live, the origin is the grab point
            world.DestroyJoint(joint);
            var jv = world.View; // joint count is as of when the view was taken
            // A destroyed lane is a tombstone this step and is not reused within it.
            Assert.Equal((uint)Box3DSoaJointKind.Removed, jv.JointKind[joint]);
            Assert.Equal((-1, -1), (jv.JointBodyA[joint], jv.JointBodyB[joint]));
            Assert.Equal(0f, jv.JointFrameW[joint]);
            Assert.Equal((0f, 0f, 0f), (jv.JointOriginX[joint], jv.JointOriginY[joint], jv.JointOriginZ[joint]));
            Assert.Equal(0, world.FreeJointSlots);
            Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(anchor, grabbed, at, (5f, 0.7f, strong), (0f, 0f, 0f)));
            world.Step(Dt);
        }
        Assert.Equal(3, world.View.Count);
        Assert.Equal(1, world.View.JointCount);
        // Released, the sphere is free: it fell.
        Assert.True(view.PositionY[b] < 5f - 0.01f, "a released body falls");
    }

    [Fact]
    public void A_regrab_after_release_follows_the_anchor()
    {
        using var world = new Box3DSoaWorld(capacity: 3, gravityY: 0f, jointCapacity: 1);
        var sphere = world.AddSphere(1, (0f, 5f, 0f), Radius, Density);
        var anchor = world.AddAnchor(2, (0f, 5f, 0f));
        var view = world.View;
        var strong = 20f * Mass * Gravity;
        for (var round = 0; round < 3; round++)
        {
            var at = (view.PositionX[sphere], view.PositionY[sphere], view.PositionZ[sphere]);
            world.SetAnchorTarget(anchor, at, default, Dt);
            world.Step(Dt);
            var joint = world.AddMotorJoint(anchor, sphere, at, (5f, 0.7f, strong), (0f, 0f, 0f));
            var target = (1f + round, at.Item2, 0f);
            for (var i = 0; i < 90; i++) { world.SetAnchorTarget(anchor, target, default, Dt); world.Step(Dt); }
            Assert.True(Distance(view, sphere, target) < 0.02f, $"round {round}: {Distance(view, sphere, target)} m from the target");
            world.DestroyJoint(joint);
            world.Step(Dt);
            // Released: the anchor can go elsewhere and the sphere stays where it was left.
            var away = (target.Item1 + 5f, target.Item2, 0f);
            for (var i = 0; i < 30; i++) { world.SetAnchorTarget(anchor, away, default, Dt); world.Step(Dt); }
            Assert.True(MathF.Abs(view.PositionX[sphere] - target.Item1) < 0.1f, "the released sphere did not follow the anchor");
        }
    }

    [Fact]
    public void A_stale_release_cannot_destroy_a_later_grab_in_a_reused_slot()
    {
        using var world = new Box3DSoaWorld(capacity: 3, gravityY: 0f, jointCapacity: 1);
        var sphere = world.AddSphere(1, (0f, 0f, 0f), Radius, Density);
        var anchor = world.AddAnchor(2, (0f, 0f, 0f));
        var first = world.AddMotorJoint(anchor, sphere, (0f, 0f, 0f), (5f, 0.7f, 1e6f), (0f, 0f, 0f));
        world.Step(Dt);
        world.DestroyJoint(first);
        world.Step(Dt); // the slot may now be reused
        var second = world.AddMotorJoint(anchor, sphere, (0f, 0f, 0f), (5f, 0.7f, 1e6f), (0f, 0f, 0f));
        Assert.Equal(first.Lane, second.Lane);
        Assert.NotEqual(first.Generation, second.Generation);
        Assert.Equal(second.Generation, world.View.JointGeneration[second.Lane]);

        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(first)); // the UI's second release click
        var view = world.View;
        Assert.Equal((uint)Box3DSoaJointKind.Motor, view.JointKind[second.Lane]);
        for (var i = 0; i < 90; i++) { world.SetAnchorTarget(anchor, (1f, 0f, 0f), default, Dt); world.Step(Dt); }
        Assert.Equal(1f, view.PositionX[sphere], 0.02f); // the second grab still holds

        world.DestroyJoint(second);
        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(second));
    }

    [Fact]
    public void The_managed_side_refuses_a_native_library_of_another_abi_in_either_direction()
    {
        Assert.Equal(14u, Box3DSoaNative.ymir_box3d_soa_get_abi_version());
        Box3DSoaWorld.RequireNativeAbi(14);
        Assert.Throws<InvalidOperationException>(() => Box3DSoaWorld.RequireNativeAbi(13)); // an older native under this managed build
        Assert.Throws<InvalidOperationException>(() => Box3DSoaWorld.RequireNativeAbi(15)); // a newer one
    }

    [Fact]
    public void Only_a_live_motor_joint_can_be_destroyed()
    {
        using var world = new Box3DSoaWorld(capacity: 4, gravityY: 0f, jointCapacity: 3);
        var pin = world.AddStaticBox(1, (0f, 5f, 0f), (0.1f, 0.1f, 0.1f));
        var sphere = world.AddSphere(2, (1f, 5f, 0f), Radius, Density);
        var anchor = world.AddAnchor(3, (1f, 5f, 0f));
        var hinge = world.AddJoint(Box3DSoaJointKind.Revolute, pin, sphere, (0f, 5f, 0f), (0f, 0f, 1f));
        var weld = world.AddJoint(Box3DSoaJointKind.Weld, anchor, sphere, (1f, 5f, 0f), (0f, 0f, 1f));
        var motor = world.AddMotorJoint(anchor, sphere, (1f, 5f, 0f), (5f, 0.7f, 100f), (0f, 0f, 0f));
        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(hinge));
        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(weld));
        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(new Box3DSoaJointHandle(-1, 0)));
        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(new Box3DSoaJointHandle(3, 0)));
        var view = world.View;
        Assert.Equal((uint)Box3DSoaJointKind.Revolute, view.JointKind[hinge]);
        world.DestroyJoint(motor);
        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(motor)); // already gone
        world.Step(Dt); // the live joints still step
        Assert.Equal((uint)Box3DSoaJointKind.Weld, view.JointKind[weld]);
    }

    [Fact]
    public void A_world_with_no_joint_slot_is_seen_before_an_anchor_is_added()
    {
        using var world = new Box3DSoaWorld(capacity: 4, jointCapacity: 0);
        var sphere = world.AddSphere(1, (0f, 5f, 0f), Radius, Density);
        Assert.Equal(0, world.FreeJointSlots);
        Assert.Equal(3, world.FreeBodySlots);
        var anchorLane = -1;
        if (world.FreeJointSlots > 0) anchorLane = world.AddAnchor(2, (0f, 5f, 0f)); // the caller's check: no grab, no anchor
        Assert.Equal(-1, anchorLane);
        Assert.Equal(3, world.FreeBodySlots);
        Assert.Throws<InvalidOperationException>(() => world.AddMotorJoint(sphere, sphere, (0f, 5f, 0f), (5f, 0.7f, 1f), (0f, 0f, 0f)));
        Assert.Equal(1, world.View.Count);
    }

    [Fact]
    public void The_angular_spring_turns_the_grabbed_body_to_the_anchors_rotation_and_off_leaves_it_free()
    {
        static float Turned(float angularHertz)
        {
            using var world = new Box3DSoaWorld(capacity: 3, gravityY: 0f, jointCapacity: 1);
            var box = world.AddBox(1, (0f, 5f, 0f), (0.2f, 0.2f, 0.2f), Density);
            var anchor = world.AddAnchor(2, (0f, 5f, 0f));
            world.AddMotorJoint(anchor, box, (0f, 5f, 0f), (5f, 0.7f, 1e5f), (angularHertz, 0.7f, 1e5f));
            var view = world.View;
            var quarterTurnZ = (0f, 0f, MathF.Sin(MathF.PI / 4), MathF.Cos(MathF.PI / 4));
            for (var i = 0; i < 90; i++)
            {
                world.SetAnchorTarget(anchor, (0f, 5f, 0f), quarterTurnZ, Dt);
                world.Step(Dt);
            }
            return 2f * MathF.Atan2(view.RotationZ[box], view.RotationW[box]);
        }

        Assert.Equal(MathF.PI / 2, Turned(angularHertz: 5f), 0.05f);
        Assert.Equal(0f, Turned(angularHertz: 0f), 1e-3f);
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
