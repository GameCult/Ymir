using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

/// <summary>Body removal: docs/soa-world.md, Release. RemoveBody is the one body-ending path; it is refused while a liquid sensor or attachment names the body.</summary>
public sealed class Box3DSoaRemoveTests
{
    private const float Dt = 1f / 60f;
    private const float R = 0.0125f;

    private static Box3DSoaWorld World(int bodies = 8, int joints = 4, int fluid = 16, int wets = 8, int sensors = 4) =>
        new(bodies, gravityY: -10f, jointCapacity: joints,
            particles: new Box3DSoaParticleCapacities(16, 16, 16, 4, 16, 2, fluid, wets, sensors));

    private static void Run(Box3DSoaWorld world, int steps)
    {
        for (var i = 0; i < steps; i++) world.Step(Dt);
    }

    /// <summary>The state a refusal must leave alone: every body and joint lane that RemoveBody writes.</summary>
    private static string Snapshot(Box3DSoaWorld world)
    {
        var v = world.View;
        return string.Join(",", v.Kind.ToArray()) + "|" + string.Join(",", v.BodyGeneration.ToArray()) + "|" + string.Join(",", v.StableId.ToArray()) +
               "|" + string.Join(",", v.JointKind.ToArray()) + "|" + string.Join(",", v.JointGeneration.ToArray()) + "|" + string.Join(",", v.WetLive.ToArray()) +
               "|" + string.Join(",", v.SensorLive.ToArray()) + "|" + string.Join(",", v.AttachmentKind.ToArray()) + "|" + world.StepCount;
    }

    [Fact]
    public void Removing_a_body_retires_its_joints_and_zeroes_its_lane()
    {
        using var world = World();
        var pin = world.AddStaticBox(1, (0f, 5f, 0f), (0.1f, 0.1f, 0.1f));
        var upper = world.AddCapsule(2, (0f, 4.5f, 0f), 0.1f, 0.4f, 1000f, parent: pin, limb: 3, materialId: 9);
        var lower = world.AddCapsule(3, (0f, 3.5f, 0f), 0.1f, 0.4f, 1000f, parent: upper);
        var elsewhere = world.AddSphere(4, (3f, 5f, 0f), 0.1f, 1000f);
        var j1 = world.AddJoint(Box3DSoaJointKind.Spherical, pin, upper, (0f, 5f, 0f), (0f, 0f, 1f));
        var j2 = world.AddJoint(Box3DSoaJointKind.Revolute, upper, lower, (0f, 4f, 0f), (0f, 0f, 1f));
        var j3 = world.AddJoint(Box3DSoaJointKind.Weld, pin, elsewhere, (1f, 5f, 0f), (0f, 0f, 1f));
        Run(world, 10);
        var view = world.View;
        Assert.Equal(upper.Generation, view.BodyGeneration[upper]);

        world.RemoveBody(upper);
        Assert.Equal((uint)Box3DSoaKind.Removed, view.Kind[upper]);
        Assert.Equal(upper.Generation + 1, view.BodyGeneration[upper]);
        Assert.Equal(0ul, view.StableId[upper]);
        Assert.Equal(-1, view.Parent[upper]);
        Assert.Equal(0, view.Limb[upper]);
        Assert.Equal(0ul, view.MaterialId[upper]);
        Assert.Equal(0, view.ContactCount[upper]);
        Assert.Equal(0f, view.PositionX[upper] + view.PositionY[upper] + view.PositionZ[upper]);
        Assert.Equal(0f, view.RotationX[upper] + view.RotationW[upper] + view.VelocityY[upper]);
        Assert.Equal(0f, view.Size0[upper] + view.Size1[upper] + view.ContactImpulse[upper]);
        foreach (var joint in new[] { j1, j2 })
        {
            Assert.Equal((uint)Box3DSoaJointKind.Removed, view.JointKind[joint]);
            Assert.Equal(joint.Generation + 1, view.JointGeneration[joint]);
            Assert.Equal(-1, view.JointBodyA[joint]);
            Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(joint)); // the old handles are stale
        }
        Assert.Equal((uint)Box3DSoaJointKind.Weld, view.JointKind[j3]); // a joint that does not name it stays

        Assert.Throws<InvalidOperationException>(() => world.RemoveBody(upper)); // already gone
        Assert.Throws<InvalidOperationException>(() => world.RemoveBody(new Box3DSoaBodyHandle(lower.Lane, lower.Generation + 1))); // stale
        Assert.Throws<InvalidOperationException>(() => world.RemoveBody(new Box3DSoaBodyHandle(8, 0))); // out of range
        Assert.Throws<InvalidOperationException>(() => world.RemoveBody(new Box3DSoaBodyHandle(-1, 0)));

        world.Step(Dt); // the removals are published by the next step, in removal order
        var bodies = world.Facts.BodiesRemoved;
        Assert.Equal(1, bodies.Length);
        Assert.Equal((upper.Lane, upper.Generation, 2ul, 0u, 0f), (bodies[0].Lane, bodies[0].Generation, bodies[0].StableId, bodies[0].WetPatches, bodies[0].WetVolume));
        var joints = world.Facts.JointsRemoved;
        Assert.Equal(2, joints.Length);
        Assert.Equal((j1.Lane, j1.Generation, pin.Lane, upper.Lane, 1u), (joints[0].Lane, joints[0].Generation, joints[0].BodyA, joints[0].BodyB, joints[0].WithBody));
        Assert.Equal((j2.Lane, j2.Generation, upper.Lane, lower.Lane, 1u), (joints[1].Lane, joints[1].Generation, joints[1].BodyA, joints[1].BodyB, joints[1].WithBody));
        world.Step(Dt);
        Assert.Equal(0, world.Facts.BodiesRemoved.Length + world.Facts.JointsRemoved.Length); // facts last one step
        Run(world, 29); // the world steps on, the removed lane stays a tombstone, the orphan falls free
        Assert.Equal((uint)Box3DSoaKind.Removed, view.Kind[upper]);
        Assert.Equal(0f, view.PositionY[upper]);
        Assert.True(view.PositionY[lower] < 3.5f - 0.5f, "the child of a removed body is free");
        Assert.Equal(lower.Generation, view.BodyGeneration[lower]);
    }

    [Fact]
    public void Removal_is_refused_while_an_attachment_or_a_sensor_names_the_body()
    {
        using var world = World();
        var floor = world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f));
        var box = world.AddBox(2, (0f, 0.1f, 0f), (0.1f, 0.1f, 0.1f), 500f);
        var other = world.AddSphere(3, (1f, 0.1f, 0f), 0.1f, 500f);
        var joint = world.AddJoint(Box3DSoaJointKind.Weld, box, other, (0.5f, 0.1f, 0f), (0f, 0f, 1f));
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new((0f, 0.1f, 0f), 0.02f, R)], p);
        var attachment = world.Attach(Box3DSoaAttachKind.Body, p[0], box, (0f, 0f, 0f));
        var sensors = new Box3DSoaConstraintHandle[1];
        world.CreateSensors([(box, (0f, 0f, 0f), 0.2f)], sensors);
        Run(world, 5);
        var before = Snapshot(world);

        Assert.Throws<InvalidOperationException>(() => world.RemoveBody(box)); // both name it
        Assert.Equal(before, Snapshot(world));
        world.Remove(attachment);
        Assert.Throws<InvalidOperationException>(() => world.RemoveBody(box)); // the sensor still does
        Assert.Equal((uint)Box3DSoaKind.Box, world.View.Kind[box]);
        Assert.Equal((uint)Box3DSoaJointKind.Weld, world.View.JointKind[joint]); // a refusal retired nothing
        world.Remove(sensors[0]);
        world.RemoveBody(box);

        var removed = box.Lane;
        Assert.Equal((uint)Box3DSoaKind.Removed, world.View.Kind[removed]);
        Assert.Throws<InvalidOperationException>(() => world.Attach(Box3DSoaAttachKind.Body, p[0], removed, (0f, 0f, 0f)));
        Assert.Throws<InvalidOperationException>(() => world.CreateSensors([(removed, (0f, 0f, 0f), 0.2f)]));
        Assert.Throws<InvalidOperationException>(() => world.AddJoint(Box3DSoaJointKind.Weld, floor, removed, (0f, 0f, 0f), (0f, 0f, 1f)));
        Assert.Throws<InvalidOperationException>(() => world.AddSphere(10, (2f, 1f, 0f), 0.1f, 500f, parent: removed));
        Assert.Equal(0, world.View.AttachmentCount == 0 ? 0 : world.View.AttachmentKind.ToArray().Count(k => k == (uint)Box3DSoaAttachKind.Body));
    }

    [Fact]
    public void A_removed_anchor_ends_its_grab()
    {
        using var world = World();
        var anchor = world.AddAnchor(1, (0f, 5f, 0f));
        var sphere = world.AddSphere(2, (0f, 5f, 0f), 0.1f, 1000f);
        var grab = world.AddMotorJoint(anchor, sphere, (0f, 5f, 0f), (5f, 0.7f, 5000f), (0f, 0f, 0f));
        Run(world, 30);
        var view = world.View;
        Assert.Equal(5f, view.PositionY[sphere], 0.3f); // held

        world.RemoveBody(anchor);
        Assert.Equal((uint)Box3DSoaJointKind.Removed, view.JointKind[grab]);
        Assert.Throws<InvalidOperationException>(() => world.DestroyJoint(grab)); // the grab handle is stale
        Run(world, 60);
        Assert.True(view.PositionY[sphere] < 3f, "the grabbed body falls");
    }

    [Fact]
    public void A_drop_resting_on_a_removed_body_cannot_settle()
    {
        using var world = World();
        var floor = world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new((0f, R, 0f), 0.015625f, R, Box3DSoaPhase.Of(0, fluid: true), 0u, null, default)], p);
        Run(world, 20);
        var view = world.View;
        Assert.Equal(floor.Lane, view.SupportBody[0]);
        world.RemoveBody(floor);
        Assert.Equal(floor.Lane, view.SupportBody[0]); // last step's fact: it may name the dead lane until the next substep
        var before = Snapshot(world);
        Assert.Throws<InvalidOperationException>(() => world.Settle(p));
        Assert.Equal(before, Snapshot(world));
        Assert.Equal(1u, view.ParticleLive[0]);
    }

    [Fact]
    public void Wet_patches_go_with_the_body()
    {
        using var world = World(fluid: 16, wets: 16);
        var a = world.AddStaticBox(1, (-1f, -0.5f, 0f), (0.5f, 0.5f, 0.5f), friction: 0.3f);
        var b = world.AddStaticBox(2, (1f, -0.5f, 0f), (0.5f, 0.5f, 0.5f), friction: 0.3f);
        var at = new[] { -1.3f, -1f, -0.7f, 0.9f, 1.2f };
        var phase = Box3DSoaPhase.Of(0, fluid: true);
        world.CreateParticles(at.Select(x => new Box3DSoaParticleDesc((x, R, 0f), 0.015625f, R, phase, 0u, null, default)).ToArray());
        Run(world, 20);
        var view = world.View;
        Assert.Equal(new[] { a.Lane, a.Lane, a.Lane, b.Lane, b.Lane }, Enumerable.Range(0, 5).Select(i => view.SupportBody[i]).ToArray());
        world.Settle(Enumerable.Range(0, 5).Select(i => new Box3DSoaParticleHandle(i, view.ParticleGeneration[i])).ToArray());
        view = world.View;
        var onA = Enumerable.Range(0, view.WetCount).Where(w => view.WetLive[w] != 0 && view.WetBody[w] == a.Lane).ToArray();
        var onB = Enumerable.Range(0, view.WetCount).Where(w => view.WetLive[w] != 0 && view.WetBody[w] == b.Lane).ToArray();
        Assert.Equal(3, onA.Length);
        Assert.Equal(2, onB.Length);
        var volumeB = view.WetVolume[b.Lane];
        var generationsB = onB.Select(w => view.WetGeneration[w]).ToArray();
        Assert.True(view.WetVolume[a.Lane] > 0f && volumeB > 0f);
        var generationsA = onA.Select(w => view.WetGeneration[w]).ToArray();

        var volumeA = 0f;
        foreach (var w in onA) volumeA += view.WetPatchVolume[w];
        world.RemoveBody(a);
        world.Step(Dt);
        var removed = world.Facts.BodiesRemoved;
        Assert.Equal(1, removed.Length);
        Assert.Equal(3u, removed[0].WetPatches);
        Assert.Equal(volumeA, removed[0].WetVolume, volumeA * 1e-5f);
        for (var k = 0; k < onA.Length; k++)
        {
            Assert.Equal(0u, view.WetLive[onA[k]]);
            Assert.Equal(generationsA[k] + 1, view.WetGeneration[onA[k]]);
        }
        Assert.Equal(0f, view.WetVolume[a.Lane]);
        Assert.Equal(volumeB, view.WetVolume[b.Lane]); // the other body's patches are untouched
        for (var k = 0; k < onB.Length; k++)
        {
            Assert.Equal(1u, view.WetLive[onB[k]]);
            Assert.Equal(generationsB[k], view.WetGeneration[onB[k]]);
        }
    }

    [Fact]
    public void A_removed_toucher_ends_its_touch_by_name()
    {
        using var world = World();
        var floor = world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f));
        var ball = world.AddSphere(77, (0f, 0.12f, 0f), 0.1f, 1000f);
        var other = world.AddSphere(78, (1f, 0.12f, 0f), 0.1f, 1000f);
        Run(world, 40);
        Assert.True(world.View.ContactCount[ball] > 0);
        world.RemoveBody(ball);
        world.Step(Dt);
        var ends = world.Facts.End;
        Assert.Equal(1, ends.Length);
        Assert.Equal(new[] { (ball.Lane, 77ul), (floor.Lane, 1ul) }.OrderBy(x => x.Item1), new[] { (ends[0].LaneA, ends[0].StableIdA), (ends[0].LaneB, ends[0].StableIdB) }.OrderBy(x => x.Item1));
        world.Step(Dt);
        Assert.Equal(0, world.Facts.End.Length);
        _ = other;
    }

    [Fact]
    public void Destroying_a_joint_is_a_removal_fact_too()
    {
        using var world = World();
        var pin = world.AddStaticBox(1, (0f, 5f, 0f), (0.1f, 0.1f, 0.1f));
        var bob = world.AddSphere(2, (0f, 4f, 0f), 0.1f, 1000f);
        var joint = world.AddJoint(Box3DSoaJointKind.Weld, pin, bob, (0f, 5f, 0f), (0f, 0f, 1f));
        world.Step(Dt);
        world.DestroyJoint(joint);
        world.Step(Dt);
        var joints = world.Facts.JointsRemoved;
        Assert.Equal(1, joints.Length);
        Assert.Equal((joint.Lane, joint.Generation, pin.Lane, bob.Lane, 0u), (joints[0].Lane, joints[0].Generation, joints[0].BodyA, joints[0].BodyB, joints[0].WithBody));
        Assert.Equal(0, world.Facts.BodiesRemoved.Length);
    }

    [Fact]
    public void A_refused_step_publishes_nothing_and_keeps_the_removal_for_the_next()
    {
        using var world = World();
        var ball = world.AddSphere(1, (0f, 5f, 0f), 0.1f, 1000f);
        var other = world.AddSphere(2, (1f, 5f, 0f), 0.1f, 1000f);
        world.RemoveBody(ball);
        world.View.ForceX[other] = float.NaN;
        Assert.Throws<InvalidOperationException>(() => world.Step(Dt)); // a non-finite write lane refuses the whole step
        world.View.ForceX[other] = 0f;
        world.Step(Dt);
        Assert.Equal(1, world.Facts.BodiesRemoved.Length);
    }

    [Fact]
    public void Lanes_survive_the_removal_of_others()
    {
        using var world = World(bodies: 8, joints: 0);
        var h = Enumerable.Range(0, 6).Select(i => world.AddSphere((ulong)(i + 1), (i * 0.5f, 5f + i, 0f), 0.1f, 1000f, velocity: (0.1f * i, 0f, 0f))).ToArray();
        Run(world, 10);
        var view = world.View;
        var keep = new[] { 0, 1, 2, 4, 5 };
        var posBefore = keep.Select(i => (view.PositionX[h[i]], view.PositionY[h[i]], view.VelocityX[h[i]])).ToArray();
        world.RemoveBody(h[3]);
        foreach (var i in keep)
        {
            Assert.Equal((uint)Box3DSoaKind.Sphere, view.Kind[h[i]]);
            Assert.Equal((ulong)(i + 1), view.StableId[h[i]]);
            Assert.Equal(h[i].Generation, view.BodyGeneration[h[i]]);
        }
        Assert.Equal(posBefore, keep.Select(i => (view.PositionX[h[i]], view.PositionY[h[i]], view.VelocityX[h[i]])).ToArray()); // continuity: nothing moved
        Run(world, 5);
        Assert.Equal(6, view.Count); // the high-water mark
        Assert.All(keep, i => Assert.True(view.PositionY[h[i]] < 5f + i, "the others keep falling"));
    }

    [Fact]
    public void A_removed_lane_is_reused_one_step_later_with_a_new_generation()
    {
        using var world = World(bodies: 3, joints: 0);
        var a = world.AddSphere(1, (0f, 5f, 0f), 0.1f, 1000f);
        var b = world.AddSphere(2, (1f, 5f, 0f), 0.1f, 1000f);
        var c = world.AddSphere(3, (2f, 5f, 0f), 0.1f, 1000f);
        Assert.Equal(0, world.FreeBodySlots);
        Assert.Throws<InvalidOperationException>(() => world.AddSphere(4, (3f, 5f, 0f), 0.1f, 1000f)); // full
        world.RemoveBody(b);
        Assert.Equal(0, world.FreeBodySlots); // not within the removing step
        Assert.Throws<InvalidOperationException>(() => world.AddSphere(4, (3f, 5f, 0f), 0.1f, 1000f));
        world.Step(Dt);
        Assert.Equal(1, world.FreeBodySlots);
        var d = world.AddSphere(2, (3f, 5f, 0f), 0.1f, 1000f); // the removed stable id may be reused
        Assert.Equal(b.Lane, d.Lane);
        Assert.Equal(b.Generation + 1, d.Generation);
        var view = world.View;
        Assert.Equal((uint)Box3DSoaKind.Sphere, view.Kind[d]);
        Assert.Equal(3f, view.PositionX[d], 1e-5f);
        Assert.Equal(3, view.Count);
        Assert.Throws<InvalidOperationException>(() => world.RemoveBody(b)); // the old handle is stale against the new occupant
        Assert.Equal((uint)Box3DSoaKind.Sphere, view.Kind[d]);
        Assert.Throws<InvalidOperationException>(() => world.AddSphere(2, (4f, 5f, 0f), 0.1f, 1000f)); // a live stable id still collides (and the world is full)
        world.RemoveBody(a);
        world.Step(Dt);
        Assert.Equal(a.Lane, world.AddSphere(7, (5f, 5f, 0f), 0.1f, 1000f).Lane); // lowest free lane first
        _ = c;
    }

    private static byte[] Replay()
    {
        using var world = World(bodies: 8, joints: 4);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (4f, 0.5f, 4f));
        var a = world.AddCapsule(2, (0f, 1f, 0f), 0.1f, 0.3f, 800f);
        var b = world.AddSphere(3, (0.3f, 2f, 0f), 0.1f, 800f);
        var c = world.AddBox(4, (-0.3f, 3f, 0f), (0.1f, 0.1f, 0.1f), 800f);
        var j1 = world.AddJoint(Box3DSoaJointKind.Spherical, a, b, (0.1f, 1.5f, 0f), (0f, 0f, 1f), forceThreshold: 50f);
        var j2 = world.AddJoint(Box3DSoaJointKind.Weld, a, c, (-0.1f, 1.5f, 0f), (0f, 0f, 1f));
        var view = world.View;
        var o = new MemoryStream();
        void B<T>(ReadOnlySpan<T> lane) where T : unmanaged => o.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes(lane));
        for (var i = 0; i < 300; i++)
        {
            if (i == 60) world.RemoveBody(b);
            if (i == 120) world.SetJointThresholds(j2, 5f, float.PositiveInfinity);
            world.Step(Dt);
            B(view.PositionX); B(view.PositionY); B(view.RotationW); B(view.VelocityY); B(view.Kind); B(view.BodyGeneration); B(view.StableId);
            B(view.JointKind); B(view.JointGeneration); B(view.JointConstraintForceY); B(view.JointConstraintTorqueZ);
            B(world.Facts.Overloads);
        }
        Assert.Equal((uint)Box3DSoaJointKind.Removed, view.JointKind[j1]);
        return o.ToArray();
    }

    [Fact]
    public void A_world_with_removals_and_thresholds_repeats_bit_for_bit()
    {
        var first = Replay();
        Assert.True(first.AsSpan().SequenceEqual(Replay()));
        Assert.Contains(first, x => x != 0);
    }

    [Fact]
    public void Removing_a_body_and_the_step_after_allocate_nothing_managed()
    {
        using var world = World(bodies: 8, joints: 2);
        var a = world.AddSphere(1, (0f, 5f, 0f), 0.1f, 1000f);
        var b = world.AddSphere(2, (1f, 5f, 0f), 0.1f, 1000f);
        world.AddJoint(Box3DSoaJointKind.Weld, a, b, (0.5f, 5f, 0f), (0f, 0f, 1f));
        Run(world, 5);
        var before = GC.GetAllocatedBytesForCurrentThread();
        world.RemoveBody(a);
        world.Step(Dt);
        Assert.Equal(0L, GC.GetAllocatedBytesForCurrentThread() - before);
    }
}
