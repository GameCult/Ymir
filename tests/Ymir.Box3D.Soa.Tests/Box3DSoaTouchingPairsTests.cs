using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

/// <summary>
/// The touching-pair span against an independent oracle: the set built from the Begin and End facts.
/// The ground is added first in one run and last in the other, so a static partner sits below and above the dynamic lanes.
/// </summary>
public sealed class Box3DSoaTouchingPairsTests
{
    private const float Dt = 1f / 60f;

    private static (int, int) Key(int a, int b) => (Math.Min(a, b), Math.Max(a, b));

    private static void Track(Box3DSoaWorld world, HashSet<(int, int)> oracle)
    {
        foreach (var t in world.Facts.End) oracle.Remove(Key(t.LaneA, t.LaneB));
        foreach (var t in world.Facts.Begin) oracle.Add(Key(t.LaneA, t.LaneB));
    }

    /// <summary>Structural checks on the span; returns the pair set, or fails the test with the step named.</summary>
    private static HashSet<(int, int)> ReadSpan(Box3DSoaWorld world, HashSet<int> dead, int step, bool checkCounts)
    {
        var facts = world.Facts;
        var view = world.View;
        var seen = new HashSet<(int, int)>();
        var count = new int[view.ContactCount.Length];
        var last = -1;
        foreach (var t in facts.Touching)
        {
            Assert.True(t.LaneA < t.LaneB, $"pair {t.LaneA},{t.LaneB} unordered at step {step}");
            Assert.True(seen.Add((t.LaneA, t.LaneB)), $"pair {t.LaneA},{t.LaneB} repeated at step {step}");
            Assert.True(t.LaneA >= last, $"lane_a not monotone at step {step}");
            last = t.LaneA;
            Assert.False(dead.Contains(t.LaneA) || dead.Contains(t.LaneB), $"pair {t.LaneA},{t.LaneB} names a removed lane at step {step}");
            Assert.True(view.StableId[t.LaneA] == t.StableIdA && view.StableId[t.LaneB] == t.StableIdB, $"stable ids of pair {t.LaneA},{t.LaneB} do not match their lanes at step {step}");
            count[t.LaneA]++;
            count[t.LaneB]++;
        }
        if (checkCounts)
        {
            for (var lane = 0; lane < count.Length; lane++)
            {
                Assert.True(view.ContactCount[lane] == count[lane], $"lane {lane}: ContactCount {view.ContactCount[lane]} but {count[lane]} pairs at step {step}");
            }
        }
        return seen;
    }

    private static Box3DSoaWorld Build(bool groundLast, out List<Box3DSoaBodyHandle> bodies, int capacity = 40, int? factCapacity = null)
    {
        var world = new Box3DSoaWorld(capacity: capacity, gravityY: -10f, factCapacity: factCapacity);
        bodies = new List<Box3DSoaBodyHandle>();
        if (!groundLast) world.AddStaticBox(1, (0f, -0.5f, 0f), (50f, 0.5f, 50f), materialId: 3);
        ulong id = 100;
        for (var x = 0; x < 4; x++)
        {
            for (var z = 0; z < 3; z++)
            {
                bodies.Add(world.AddSphere(id++, (x * 0.9f - 1.4f, 2f + 0.6f * (x + z), z * 0.9f - 0.9f), 0.5f, density: 1f,
                    velocity: (0.2f * z, 0f, -0.1f * x), materialId: (ulong)(x + 10)));
            }
        }
        bodies.Add(world.AddBox(300, (4f, 0.3f, 0f), (0.3f, 0.3f, 0.3f), 1f));
        bodies.Add(world.AddBox(301, (4f, 0.9f, 0f), (0.3f, 0.3f, 0.3f), 1f));
        if (groundLast) world.AddStaticBox(1, (0f, -0.5f, 0f), (50f, 0.5f, 50f), materialId: 3);
        return world;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void The_touching_span_equals_the_begin_end_oracle_whether_the_ground_is_added_first_or_last(bool groundLast)
    {
        using var world = Build(groundLast, out _);
        var oracle = new HashSet<(int, int)>();
        var dead = new HashSet<int>();
        var sawGround = false;
        for (var step = 0; step < 1500; step++)
        {
            world.Step(Dt);
            Track(world, oracle);
            var seen = ReadSpan(world, dead, step, checkCounts: world.Facts.Dropped.Touching == 0);
            Assert.True(seen.SetEquals(oracle), $"span differs from the Begin/End oracle at step {step}: touching {seen.Count}, oracle {oracle.Count}");
            foreach (var t in world.Facts.Touching) sawGround |= t.StableIdA == 1 || t.StableIdB == 1;
        }
        Assert.True(sawGround, "bodies rested on the static ground, so the static partner was exercised");
    }

    [Fact]
    public void Removed_lanes_leave_the_span_and_a_reused_lane_pairs_afresh()
    {
        using var world = Build(false, out var bodies);
        var oracle = new HashSet<(int, int)>();
        var dead = new HashSet<int>();
        var reusedLane = -1;
        var sawNewPair = false;
        for (var step = 0; step < 900; step++)
        {
            if (step == 200) { var h = bodies[5]; world.RemoveBody(h); dead.Add(h.Lane); }
            if (step == 205) { var h = bodies[2]; world.RemoveBody(h); dead.Add(h.Lane); }
            if (step == 300)
            {
                reusedLane = world.AddSphere(900, (0.1f, 3f, 0.1f), 0.5f, density: 1f).Lane;
                dead.Remove(reusedLane);
            }
            world.Step(Dt);
            Track(world, oracle);
            if (step >= 300) foreach (var t in world.Facts.Touching) sawNewPair |= t.LaneA == reusedLane || t.LaneB == reusedLane;
            var seen = ReadSpan(world, dead, step, checkCounts: world.Facts.Dropped.Touching == 0);
            Assert.True(seen.SetEquals(oracle), $"span differs from the Begin/End oracle at step {step}");
        }
        Assert.True(sawNewPair, "the reused lane touched something");
    }

    [Fact]
    public void An_overfull_span_keeps_what_fits_and_counts_the_rest()
    {
        using var world = Build(false, out _, factCapacity: 5);
        var oracle = new HashSet<(int, int)>();
        var sawDropped = false;
        for (var step = 0; step < 600; step++)
        {
            world.Step(Dt);
            Track(world, oracle);
            var facts = world.Facts;
            Assert.True(facts.Touching.Length <= 5, $"span holds {facts.Touching.Length} pairs past its capacity at step {step}");
            Assert.True(facts.Touching.Length + facts.Dropped.Touching == oracle.Count, $"kept {facts.Touching.Length} + dropped {facts.Dropped.Touching} != oracle {oracle.Count} at step {step}");
            sawDropped |= facts.Dropped.Touching > 0;
        }
        Assert.True(sawDropped, "the span overflowed, so the drop count was exercised");
    }
}
