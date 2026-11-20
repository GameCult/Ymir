using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

public sealed class Box3DSoaContactTests
{
    private const float Dt = 1f / 60f;
    private const float G = 10f;
    private const float Radius = 0.5f;
    private const ulong GroundId = 1;
    private const ulong BallId = 2;

    private static float BallMass => 4f / 3f * MathF.PI * Radius * Radius * Radius; // density 1

    /// <summary>A ground slab with its top face at y = 0 (lane 0) and a ball (lane 1) dropped from 1.4 m above it.</summary>
    private static Box3DSoaWorld Drop(float height = 1.4f, int? factCapacity = null, float hitThreshold = 1f)
    {
        var world = new Box3DSoaWorld(capacity: 8, gravityY: -G, factCapacity: factCapacity, hitThreshold: hitThreshold);
        world.AddStaticBox(GroundId, (0f, -0.5f, 0f), (50f, 0.5f, 50f), materialId: 7);
        world.AddSphere(BallId, (0f, Radius + height, 0f), Radius, density: 1f, materialId: 42);
        return world;
    }

    /// <summary>Steps until the first step that reports a begin; returns false if none within the budget.</summary>
    private static bool StepToFirstBegin(Box3DSoaWorld world, int budget = 120)
    {
        for (var i = 0; i < budget; i++)
        {
            world.Step(Dt);
            if (world.Facts.Begin.Length > 0) return true;
        }
        return false;
    }

    [Fact]
    public void A_drop_gives_one_begin_and_one_hit()
    {
        using var world = Drop();
        int begins = 0, hits = 0, ends = 0;
        Box3DSoaHit hit = default;
        Box3DSoaTouch begin = default;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt);
            var facts = world.Facts;
            if (facts.Begin.Length > 0) begin = facts.Begin[0];
            if (facts.Hit.Length > 0) hit = facts.Hit[0];
            begins += facts.Begin.Length;
            hits += facts.Hit.Length;
            ends += facts.End.Length;
            Assert.Equal(0u, facts.Dropped.Begin + facts.Dropped.End + facts.Dropped.Hit);
        }
        Assert.Equal((1, 1, 0), (begins, hits, ends));

        Assert.Equal(new[] { 0, 1 }, new[] { begin.LaneA, begin.LaneB }.Order());
        Assert.Equal(new[] { GroundId, BallId }, new[] { begin.StableIdA, begin.StableIdB }.Order());
        Assert.Equal((begin.LaneA, begin.LaneB, begin.StableIdA, begin.StableIdB), (hit.LaneA, hit.LaneB, hit.StableIdA, hit.StableIdB));
        Assert.Equal(begin.LaneA == 0 ? GroundId : BallId, begin.StableIdA); // the stable id is the lane's own

        var expected = MathF.Sqrt(2f * G * 1.4f);
        Assert.InRange(hit.ApproachSpeed, expected * 0.95f, expected * 1.05f);
        Assert.InRange(hit.PointY, -0.02f, 0.02f);
        Assert.InRange(MathF.Abs(hit.PointX), 0f, 0.05f);
        Assert.InRange(MathF.Abs(hit.NormalY), 0.99f, 1.001f);
        // A to B: the normal points from the ground up to the ball when the ground is A.
        Assert.True(hit.LaneA == 0 ? hit.NormalY > 0f : hit.NormalY < 0f);
    }

    [Fact]
    public void Lifting_gives_one_end()
    {
        using var world = Drop();
        Run(world, 240);
        Assert.Equal(1, world.View.ContactCount[1]);

        world.View.ForceY[1] = 100f; // one step's push: about 3 m/s upward, well clear of the slab within a few steps
        var begins = 0;
        var ends = new List<Box3DSoaTouch>();
        for (var i = 0; i < 20; i++)
        {
            world.Step(Dt);
            begins += world.Facts.Begin.Length;
            ends.AddRange(world.Facts.End.ToArray());
        }
        Assert.Equal(0, begins);
        var end = Assert.Single(ends);
        Assert.Equal(new[] { 0, 1 }, new[] { end.LaneA, end.LaneB }.Order());
        Assert.Equal(new[] { GroundId, BallId }, new[] { end.StableIdA, end.StableIdB }.Order());
        Assert.Equal(0, world.View.ContactCount[1]);
    }

    [Fact]
    public void Resting_last_substep_force_is_the_weight_and_a_body_touching_nothing_reads_zero()
    {
        using var world = Drop();
        world.AddSphere(3, (10f, 1000f, 10f), Radius, density: 1f); // lane 2: still falling after the whole run (5 s drops 125 m)
        Run(world, 300);
        var view = world.View;
        var weight = BallMass * G;
        Assert.InRange(view.LastSubstepForce[1], weight * 0.98f, weight * 1.02f);
        Assert.Equal(1, view.ContactCount[1]);
        Assert.InRange(view.LastSubstepForce[0], weight * 0.98f, weight * 1.02f); // the slab carries the same load
        Assert.Equal(0f, view.LastSubstepForce[2]);
        Assert.Equal(0, view.ContactCount[2]);
    }

    [Fact]
    public void Materials_ride_hits_in_pair_order_and_read_back_as_a_lane()
    {
        using var world = Drop();
        Assert.Equal(new ulong[] { 7, 42 }, world.View.MaterialId.ToArray());
        Assert.True(StepToFirstBegin(world));
        var hit = Assert.Single(world.Facts.Hit.ToArray());
        var material = new Dictionary<int, ulong> { [0] = 7, [1] = 42 };
        Assert.Equal((material[hit.LaneA], material[hit.LaneB]), (hit.MaterialA, hit.MaterialB));
        Assert.Equal(new ulong[] { 7, 42 }, world.View.MaterialId.ToArray()); // unchanged by stepping
    }

    [Fact]
    public void Facts_last_one_step()
    {
        using var world = Drop();
        Assert.True(StepToFirstBegin(world));
        Assert.Equal(1, world.Facts.Hit.Length);
        world.Step(Dt);
        var facts = world.Facts;
        Assert.Equal((0, 0, 0), (facts.Begin.Length, facts.End.Length, facts.Hit.Length));
    }

    [Fact]
    public void Overflow_is_counted_and_a_zero_capacity_is_refused()
    {
        using var world = new Box3DSoaWorld(capacity: 8, gravityY: -G, factCapacity: 1);
        world.AddStaticBox(GroundId, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        world.AddSphere(2, (-3f, 1.9f, 0f), Radius, density: 1f);
        world.AddSphere(3, (3f, 1.9f, 0f), Radius, density: 1f);
        Assert.True(StepToFirstBegin(world));
        var facts = world.Facts;
        Assert.Equal(1, facts.Begin.Length);
        Assert.Equal(1, facts.Hit.Length);
        Assert.Equal(1u, facts.Dropped.Begin);
        Assert.Equal(1u, facts.Dropped.Hit);
        Assert.Equal(1, facts.Capacity);

        Assert.Throws<InvalidOperationException>(() => new Box3DSoaWorld(capacity: 8, factCapacity: 0));
    }

    [Fact]
    public void Dropped_counters_reset_every_step()
    {
        using var world = new Box3DSoaWorld(capacity: 8, gravityY: -G, factCapacity: 1);
        world.AddStaticBox(GroundId, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        world.AddSphere(2, (-3f, 1.9f, 0f), Radius, density: 1f);
        world.AddSphere(3, (3f, 1.9f, 0f), Radius, density: 1f);
        Assert.True(StepToFirstBegin(world));
        Assert.Equal(1u, world.Facts.Dropped.Begin);
        world.Step(Dt);
        Assert.Equal(new Box3DSoaDropped(0, 0, 0), world.Facts.Dropped);
    }

    [Theory]
    [InlineData(60, 1, 0f)]
    [InlineData(60, 4, 0f)]
    [InlineData(60, 1, 0.5f)]
    [InlineData(60, 4, 0.5f)]
    [InlineData(240, 1, 0f)]
    [InlineData(240, 4, 0f)]
    [InlineData(240, 1, 0.5f)]
    [InlineData(240, 4, 0.5f)]
    public void The_hit_impulse_estimate_matches_the_balls_momentum_change_over_the_impact(int hz, int subSteps, float restitution)
    {
        // Oracle: the only forces on the ball are gravity and the slab's contact, so over a window that starts
        // at the hit step the contact impulse is m (v_after - v_before) + m g T. The impact is that impulse less
        // the weight the slab then carries while the ball lies at rest, so steps spent at rest (touching, nearly
        // still) are left out of T.
        var dt = 1f / hz;
        using var world = new Box3DSoaWorld(capacity: 8, gravityY: -G);
        world.AddStaticBox(GroundId, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        world.AddSphere(BallId, (0f, Radius + 1.4f, 0f), Radius, density: 1f, restitution: restitution);
        var mass = BallMass;
        var before = 0f;
        var window = (int)MathF.Ceiling(0.05f * hz);
        var inWindow = 0;
        var hitSteps = 0;
        var activeSteps = 0;
        Box3DSoaHit hit = default;
        float vBefore = 0f, vAfter = 0f;
        for (var i = 0; i < 4 * hz && inWindow < window; i++)
        {
            world.Step(dt, subSteps);
            var vy = world.View.VelocityY[1];
            var atRest = world.View.ContactCount[1] > 0 && MathF.Abs(vy) < 0.05f;
            if (world.Facts.Hit.Length > 0 && inWindow == 0)
            {
                hit = world.Facts.Hit[0];
                vBefore = before;
                hitSteps++;
                inWindow = 1;
                activeSteps = atRest ? 0 : 1;
            }
            else if (inWindow > 0)
            {
                hitSteps += world.Facts.Hit.Length;
                inWindow++;
                activeSteps += atRest ? 0 : 1;
            }
            vAfter = vy;
            before = vy;
        }
        Assert.Equal(1, hitSteps); // one hit in the window: no bounce-and-rehit mixed into the oracle
        var truth = mass * (vAfter - vBefore) + mass * G * (activeSteps * dt);
        Assert.InRange(hit.ImpulseEstimate, truth * 0.92f, truth * 1.08f);
    }

    private static Box3DSoaWorld Pile()
    {
        var world = new Box3DSoaWorld(capacity: 32, gravityY: -G);
        world.AddStaticBox(GroundId, (0f, -0.5f, 0f), (50f, 0.5f, 50f), materialId: 3);
        ulong id = 100;
        for (var x = 0; x < 4; x++)
        {
            for (var z = 0; z < 3; z++)
            {
                world.AddSphere(id++, (x * 0.9f - 1.4f, 2f + 0.6f * (x + z), z * 0.9f - 0.9f), Radius, density: 1f,
                    velocity: (0.2f * z, 0f, -0.1f * x), materialId: (ulong)(x + 10));
            }
        }
        return world;
    }

    private static byte[] Snapshot(Box3DSoaWorld world)
    {
        var facts = world.Facts;
        var view = world.View;
        return MemoryMarshal.AsBytes(facts.Begin).ToArray()
            .Concat(MemoryMarshal.AsBytes(facts.End).ToArray())
            .Concat(MemoryMarshal.AsBytes(facts.Hit).ToArray())
            .Concat(MemoryMarshal.AsBytes(view.LastSubstepForce).ToArray())
            .Concat(MemoryMarshal.AsBytes(view.ContactCount).ToArray())
            .Concat(BitConverter.GetBytes(facts.Dropped.Begin + facts.Dropped.End + facts.Dropped.Hit))
            .ToArray();
    }

    [Fact]
    public void Determinism_covers_facts_and_contact_lanes_at_every_step()
    {
        using var a = Pile();
        using var b = Pile();
        var factsSeen = 0;
        for (var step = 0; step < 300; step++)
        {
            a.Step(Dt);
            b.Step(Dt);
            var snapshot = Snapshot(a);
            Assert.True(snapshot.AsSpan().SequenceEqual(Snapshot(b)), $"facts or contact lanes diverged at step {step}");
            factsSeen += a.Facts.Begin.Length + a.Facts.Hit.Length + a.Facts.End.Length;
        }
        Assert.True(factsSeen > 12, "the pile produced facts, so equality is not trivially empty");
        Assert.Contains(a.View.LastSubstepForce.ToArray(), f => f > 0f);
    }

    [Fact]
    public void A_steady_state_step_with_fact_reads_allocates_nothing()
    {
        using var world = Pile();
        Run(world, 120); // warm up: JIT, and the pile has settled into steady contact
        long Read()
        {
            var facts = world.Facts;
            var view = world.View;
            return facts.Begin.Length + facts.End.Length + facts.Hit.Length + (long)view.LastSubstepForce[1] + view.ContactCount[1];
        }
        Read();
        var before = GC.GetAllocatedBytesForCurrentThread();
        long sink = 0;
        for (var i = 0; i < 100; i++)
        {
            world.Step(Dt);
            sink += Read();
        }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0L, allocated);
        Assert.True(sink != long.MinValue);
    }

    private static void Run(Box3DSoaWorld world, int steps)
    {
        for (var i = 0; i < steps; i++) world.Step(Dt);
    }
}
