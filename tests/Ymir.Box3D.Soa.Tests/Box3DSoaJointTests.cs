using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

public sealed class Box3DSoaJointTests
{
    private const float Dt = 1f / 60f;
    private static readonly (float, float, float, float) QuarterTurnZ = (0f, 0f, MathF.Sin(MathF.PI / 4), MathF.Cos(MathF.PI / 4));

    /// <summary>A static anchor at the origin and a capsule hinged at x = 0, lying along -X, free to swing about Z.</summary>
    private static (Box3DSoaWorld World, int Anchor, int Capsule, int Joint) Pendulum(Box3DSoaJointKind kind = Box3DSoaJointKind.Revolute)
    {
        var world = new Box3DSoaWorld(capacity: 4, jointCapacity: 2);
        var anchor = world.AddStaticBox(1, (0f, 5f, 0f), (0.1f, 0.1f, 0.1f));
        var capsule = world.AddCapsule(2, (-0.5f, 5f, 0f), radius: 0.1f, halfLength: 0.5f, density: 1f,
            rotation: QuarterTurnZ, parent: anchor, limb: 7);
        var joint = world.AddJoint(kind, anchor, capsule, anchor: (0f, 5f, 0f), axis: (0f, 0f, 1f));
        return (world, anchor, capsule, joint);
    }

    [Fact]
    public void A_revolute_joint_starts_at_zero_and_its_state_lanes_follow_the_swing()
    {
        var (world, anchor, capsule, joint) = Pendulum();
        using var _ = world;
        var view = world.View;
        Assert.Equal(1, view.JointCount);
        Assert.Equal((uint)Box3DSoaJointKind.Revolute, view.JointKind[joint]);
        Assert.Equal(anchor, view.JointBodyA[joint]);
        Assert.Equal(capsule, view.JointBodyB[joint]);
        Assert.Equal(0f, view.JointAngle[joint], 1e-5f);
        Assert.Equal(1f, view.JointRotationW[joint], 1e-5f); // the creation pose is the joint's zero

        for (var i = 0; i < 30; i++) world.Step(Dt);
        var angle = view.JointAngle[joint];
        Assert.True(MathF.Abs(angle) > 0.5f, "gravity swings the capsule down about the hinge");
        // Box3D's own angle and the angle derived from the relative-rotation lanes are the same quantity.
        Assert.Equal(angle, 2f * MathF.Atan2(view.JointRotationZ[joint], view.JointRotationW[joint]), 1e-3f);
        // The hinge permits only Z: the off-axis part of the relative rotation stays small.
        Assert.True(MathF.Abs(view.JointRotationX[joint]) < 0.02f && MathF.Abs(view.JointRotationY[joint]) < 0.02f);
        Assert.True(MathF.Abs(view.JointAngularVelocityZ[joint]) > 0.5f);
        // The anchor never moves; the capsule's end stays on the pin (distance from the hinge is the arm length 1 - radius-free).
        var dx = view.PositionX[capsule]; var dy = view.PositionY[capsule] - 5f;
        Assert.Equal(0.5f, MathF.Sqrt(dx * dx + dy * dy), 0.02f);
    }

    [Fact]
    public void Joint_frame_lane_is_the_world_orientation_of_frame_a()
    {
        var (world, anchor, _, joint) = Pendulum();
        using var _ = world;
        var view = world.View;
        // Anchor is unrotated and the joint axis is Z, so frame A is identity in world space.
        Assert.Equal(0f, view.JointFrameX[joint], 1e-5f);
        Assert.Equal(0f, view.JointFrameY[joint], 1e-5f);
        Assert.Equal(0f, view.JointFrameZ[joint], 1e-5f);
        Assert.Equal(1f, MathF.Abs(view.JointFrameW[joint]), 1e-5f);
        Assert.Equal(-1, view.Parent[anchor]);
    }

    [Fact]
    public void An_off_axis_hinge_gets_a_frame_rotated_onto_its_axis()
    {
        using var world = new Box3DSoaWorld(capacity: 2, jointCapacity: 1);
        var a = world.AddStaticBox(1, (0f, 5f, 0f), (0.1f, 0.1f, 0.1f));
        var b = world.AddBox(2, (0f, 5f, 1f), (0.2f, 0.2f, 0.2f), 1f, parent: a); // level with the hinge, so it falls
        // Hinge about world X: frame z = X, so frame A is the quarter turn about Y that takes Z to X.
        var j = world.AddJoint(Box3DSoaJointKind.Revolute, a, b, (0f, 5f, 0f), (1f, 0f, 0f));
        var v = world.View;
        Assert.Equal(MathF.Sin(MathF.PI / 4), v.JointFrameY[j], 1e-4f);
        Assert.Equal(MathF.Cos(MathF.PI / 4), v.JointFrameW[j], 1e-4f);
        for (var i = 0; i < 40; i++) world.Step(Dt);
        // It swings about world X only: the box's rotation quaternion has no Y or Z part.
        Assert.True(MathF.Abs(v.RotationX[b]) > 0.05f);
        Assert.True(MathF.Abs(v.RotationY[b]) < 0.02f && MathF.Abs(v.RotationZ[b]) < 0.02f);
    }

    [Fact]
    public void A_revolute_limit_stops_the_swing_and_a_weld_holds_the_pose()
    {
        using var world = new Box3DSoaWorld(capacity: 6, jointCapacity: 3);
        var a = world.AddStaticBox(1, (0f, 5f, 0f), (0.1f, 0.1f, 0.1f));
        var limited = world.AddCapsule(2, (-0.5f, 5f, 0f), 0.1f, 0.5f, 1f, rotation: QuarterTurnZ, parent: a);
        var j = world.AddJoint(Box3DSoaJointKind.Revolute, a, limited, (0f, 5f, 0f), (0f, 0f, 1f), limit: true, lower: -0.3f, upper: 0.3f);
        var welded = world.AddCapsule(3, (0.5f, 5f, 0f), 0.1f, 0.5f, 1f, rotation: QuarterTurnZ, parent: a);
        var w = world.AddJoint(Box3DSoaJointKind.Weld, a, welded, (0f, 5f, 0f), (0f, 0f, 1f));
        for (var i = 0; i < 180; i++) world.Step(Dt);
        var v = world.View;
        Assert.InRange(v.JointAngle[j], -0.32f, 0.32f);
        Assert.True(MathF.Abs(v.JointAngle[j]) > 0.25f, "it reached the stop rather than hanging free");
        Assert.Equal((uint)Box3DSoaJointKind.Weld, v.JointKind[w]);
        Assert.Equal(0.5f, v.PositionX[welded], 0.05f); // a cantilevered weld sags only a little
        Assert.Equal(5f, v.PositionY[welded], 0.1f);
    }

    [Fact]
    public void A_spherical_joint_swings_in_every_direction_and_reports_a_relative_rotation()
    {
        var (world, _, capsule, joint) = Pendulum(Box3DSoaJointKind.Spherical);
        using var _ = world;
        var v = world.View;
        for (var i = 0; i < 20; i++) world.Step(Dt);
        // A push along Z leaves the plane of the swing, which a hinge would refuse.
        v.ForceZ[capsule] = 60f;
        world.Step(Dt);
        for (var i = 0; i < 20; i++) world.Step(Dt);
        Assert.True(MathF.Abs(v.JointRotationX[joint]) > 0.02f || MathF.Abs(v.JointRotationY[joint]) > 0.02f);
        Assert.True(MathF.Abs(v.PositionZ[capsule]) > 0.005f);
        var n = v.JointRotationX[joint] * v.JointRotationX[joint] + v.JointRotationY[joint] * v.JointRotationY[joint]
              + v.JointRotationZ[joint] * v.JointRotationZ[joint] + v.JointRotationW[joint] * v.JointRotationW[joint];
        Assert.Equal(1f, n, 1e-3f);
        Assert.True(v.JointRotationW[joint] >= 0f);
    }

    [Fact]
    public void Force_and_torque_lanes_act_before_the_step_and_clear_after()
    {
        using var world = new Box3DSoaWorld(capacity: 3, gravityY: 0f);
        var ball = world.AddSphere(1, (0f, 0f, 0f), 0.5f, 1f);
        var ground = world.AddStaticBox(2, (0f, -10f, 0f), (1f, 1f, 1f));
        var v = world.View;
        var mass = 4f / 3f * MathF.PI * 0.5f * 0.5f * 0.5f;
        var inertia = 0.4f * mass * 0.25f;

        v.ForceX[ball] = 6f;
        v.TorqueZ[ball] = 2f;
        v.ForceX[ground] = 1000f; // a static body ignores its lanes, and they clear all the same
        world.Step(Dt);
        Assert.Equal(6f / mass * Dt, v.VelocityX[ball], 6f / mass * Dt * 0.02f);
        Assert.Equal(2f / inertia * Dt, v.AngularVelocityZ[ball], 2f / inertia * Dt * 0.02f);
        Assert.Equal(0f, v.VelocityX[ground]);
        Assert.Equal(0f, v.ForceX[ball]);
        Assert.Equal(0f, v.TorqueZ[ball]);
        Assert.Equal(0f, v.ForceX[ground]);

        var vx = v.VelocityX[ball];
        world.Step(Dt);
        Assert.Equal(vx, v.VelocityX[ball], 1e-6f); // nothing was applied twice
    }

    [Fact]
    public void A_non_finite_write_lane_refuses_the_step_and_touches_nothing()
    {
        using var world = new Box3DSoaWorld(capacity: 3, gravityY: 0f);
        var a = world.AddSphere(1, (0f, 0f, 0f), 0.5f, 1f);
        var b = world.AddSphere(2, (5f, 0f, 0f), 0.5f, 1f);
        var v = world.View;
        v.ForceX[a] = 10f;
        v.TorqueY[b] = float.NaN;
        Assert.Throws<InvalidOperationException>(() => world.Step(Dt));
        Assert.Equal(0ul, world.StepCount);
        Assert.Equal(10f, v.ForceX[a]); // not applied, not cleared
        Assert.Equal(0f, v.VelocityX[a]);
        v.TorqueY[b] = 0f;
        world.Step(Dt);
        Assert.True(v.VelocityX[a] > 0f);
    }

    [Fact]
    public void Capsules_and_boxes_rest_on_the_ground_at_their_own_heights_with_parent_and_limb_lanes()
    {
        using var world = new Box3DSoaWorld(capacity: 4);
        var ground = world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f));
        var capsule = world.AddCapsule(2, (-2f, 2f, 0f), 0.2f, 0.5f, 1f, rotation: QuarterTurnZ, parent: ground, limb: 3);
        var box = world.AddBox(3, (2f, 2f, 0f), (0.3f, 0.4f, 0.5f), 1f, parent: capsule, limb: 9);
        for (var i = 0; i < 300; i++) world.Step(Dt);
        var v = world.View;
        Assert.Equal((uint)Box3DSoaKind.Capsule, v.Kind[capsule]);
        Assert.Equal((uint)Box3DSoaKind.Box, v.Kind[box]);
        Assert.Equal(0.2f, v.PositionY[capsule], 0.02f); // lying down: centre one radius up
        Assert.Equal(0.4f, v.PositionY[box], 0.02f);     // half height
        Assert.Equal(-1, v.Parent[ground]);
        Assert.Equal(ground, v.Parent[capsule]);
        Assert.Equal(capsule, v.Parent[box]);
        Assert.Equal(3, v.Limb[capsule]);
        Assert.Equal(9, v.Limb[box]);
    }

    [Fact]
    public void Refuses_bad_bodies_and_bad_joints()
    {
        using var world = new Box3DSoaWorld(capacity: 3, jointCapacity: 1);
        var a = world.AddSphere(1, (0f, 5f, 0f), 0.5f, 1f);
        var b = world.AddSphere(2, (1f, 5f, 0f), 0.5f, 1f);
        Assert.Throws<InvalidOperationException>(() => world.AddCapsule(3, (0f, 0f, 0f), 0.2f, 0f, 1f));            // no length
        Assert.Throws<InvalidOperationException>(() => world.AddBox(3, (0f, 0f, 0f), (1f, 0f, 1f), 1f));           // flat box
        Assert.Throws<InvalidOperationException>(() => world.AddBox(3, (0f, 0f, 0f), (1f, 1f, 1f), 0f));           // massless
        Assert.Throws<InvalidOperationException>(() => world.AddBox(3, (0f, 0f, 0f), (1f, 1f, 1f), 1f, rotation: (0f, 0f, 0.5f, 0.5f))); // not unit
        Assert.Throws<InvalidOperationException>(() => world.AddSphere(3, (0f, 0f, 0f), 0.5f, 1f, parent: 2));     // parent not yet created
        Assert.Throws<InvalidOperationException>(() => world.AddSphere(3, (0f, 0f, 0f), 0.5f, 1f, friction: -1f));
        Assert.Throws<InvalidOperationException>(() => world.AddJoint(Box3DSoaJointKind.Weld, a, a, (0f, 0f, 0f), (0f, 0f, 1f)));
        Assert.Throws<InvalidOperationException>(() => world.AddJoint(Box3DSoaJointKind.Weld, a, 9, (0f, 0f, 0f), (0f, 0f, 1f)));
        Assert.Throws<InvalidOperationException>(() => world.AddJoint(Box3DSoaJointKind.Weld, a, b, (0f, 0f, 0f), (0f, 0f, 0f)));
        Assert.Throws<InvalidOperationException>(() => world.AddJoint(Box3DSoaJointKind.Revolute, a, b, (0f, 0f, 0f), (0f, 0f, 1f), limit: true, lower: 1f, upper: -1f));
        world.AddJoint(Box3DSoaJointKind.Weld, a, b, (0.5f, 5f, 0f), (0f, 0f, 1f));
        Assert.Throws<InvalidOperationException>(() => world.AddJoint(Box3DSoaJointKind.Weld, a, b, (0.5f, 5f, 0f), (0f, 0f, 1f))); // joint capacity
        Assert.Equal(2, world.View.Count); // refused bodies left no trace
        Assert.Equal(1, world.View.JointCount);
    }

    [Fact]
    public void A_jointed_body_driven_by_torque_lanes_repeats_bit_for_bit()
    {
        static byte[] Run()
        {
            var (world, anchor, capsule, joint) = Pendulum(Box3DSoaJointKind.Spherical);
            using var _ = world;
            var v = world.View;
            for (var i = 0; i < 200; i++)
            {
                v.TorqueZ[capsule] = 3f * MathF.Sin(i * 0.1f);
                v.TorqueX[capsule] = 1f * MathF.Cos(i * 0.07f);
                world.Step(Dt);
            }
            var floats = new List<float>();
            void Add(ReadOnlySpan<float> lane) => floats.AddRange(lane.ToArray());
            Add(v.PositionX); Add(v.PositionY); Add(v.PositionZ);
            Add(v.RotationX); Add(v.RotationY); Add(v.RotationZ); Add(v.RotationW);
            Add(v.JointAngle); Add(v.JointRotationX); Add(v.JointRotationY); Add(v.JointRotationZ); Add(v.JointRotationW);
            Add(v.JointAngularVelocityX); Add(v.JointAngularVelocityY); Add(v.JointAngularVelocityZ);
            return MemoryMarshal.AsBytes(floats.ToArray().AsSpan()).ToArray();
        }
        var first = Run();
        Assert.True(first.AsSpan().SequenceEqual(Run()));
        Assert.Contains(first, b => b != 0);
    }
}
