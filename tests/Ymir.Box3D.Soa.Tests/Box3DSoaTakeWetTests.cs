using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>take_wet (ABI 21): pooled liquid leaves the wet patches a liquid sensor reads, every patch the same fraction (docs/soa-world.md, Take).</summary>
public sealed partial class Box3DSoaFluidTests
{
    // A floor (body 0, its frame origin y -0.5) with three patches: A (x -0.3, two drops, 2 D), B (x -0.1, D) and C (x 0.4, D), and bare floor between.
    // Sensor centres are given by x on the floor's top (body-frame y 0.5).
    private static (Box3DSoaWorld World, Box3DSoaConstraintHandle[] Sensors) TakeFloor(params (float X, float Radius)[] sensors)
    {
        var (world, drops) = DropsOnFloor([(-0.3f, 0f, 0f), (-0.285f, 0f, 0f), (-0.1f, 0f, 0f), (0.4f, 0f, 0f)], 8, 4);
        var h = new Box3DSoaConstraintHandle[sensors.Length];
        world.CreateSensors(sensors.Select(s => (0, (s.X, 0.5f, 0f), s.Radius)).ToArray(), h);
        world.Settle(drops);
        world.Step(Dt, Sub);
        return (world, h);
    }

    private static float WetRadiusOf(float volume) => MathF.Sqrt(volume / (3.14159265358979f * 1.0e-3f));

    [Fact]
    public void TakeWet_a_partial_take_leaves_every_patch_in_the_sphere_the_same_fraction()
    {
        var (world, s) = TakeFloor((-0.2f, 0.15f));
        using var _ = world;
        var before = world.View;
        var vol = before.WetPatchVolume.ToArray();
        var rad = before.WetRadius.ToArray();
        var x = before.WetX.ToArray();
        var live = before.WetLive.ToArray();
        var inside = Enumerable.Range(0, before.WetCount).Where(w => live[w] != 0 && x[w] < 0f).ToArray();
        var outside = Enumerable.Range(0, before.WetCount).Where(w => live[w] != 0 && x[w] > 0f).ToArray();
        Assert.Equal(2, inside.Length);
        Assert.Single(outside);
        var pooled = before.SensorWetVolume[s[0]];
        Assert.Equal(inside.Sum(w => vol[w]), pooled, pooled * 1e-5f);
        Assert.NotEqual(vol[inside[0]], vol[inside[1]]); // 2 D and D: an equal share would differ from a proportional one
        var bodyBefore = before.WetVolume[0];
        var ask = pooled / 3f;
        var taken = new float[1];
        world.TakeWet(s, [ask], taken);
        var v = world.View;
        Assert.Equal(ask, taken[0], ask * 1e-5f);
        var keep = 1f - ask / pooled;
        foreach (var w in inside)
        {
            Assert.Equal(vol[w] * keep, v.WetPatchVolume[w], vol[w] * 1e-5f);
            Assert.Equal(WetRadiusOf(v.WetPatchVolume[w]), v.WetRadius[w], 1e-6f);
            Assert.Equal(1u, v.WetLive[w]);
        }
        foreach (var w in outside)
        {
            Assert.Equal(vol[w], v.WetPatchVolume[w]);
            Assert.Equal(rad[w], v.WetRadius[w]);
        }
        Assert.Equal(bodyBefore - taken[0], v.WetVolume[0], bodyBefore * 1e-5f);
        Assert.Equal(pooled, v.SensorWetVolume[s[0]]); // a reading: it changes at the next step
        world.Step(Dt, Sub);
        v = world.View;
        Assert.Equal(pooled - ask, v.SensorWetVolume[s[0]], pooled * 1e-5f);
        Assert.Equal(PooledInside(v, (-0.2f, 0f, 0f), 0.15f).Volume, v.SensorWetVolume[s[0]]);
        Readings.Add($"TAKE partial: pooled {pooled:E4} asked {ask:E4} taken {taken[0]:E4}, keep {keep:F4} in both patches");
    }

    [Fact]
    public void TakeWet_taking_at_least_what_is_there_empties_the_sphere()
    {
        var (world, s) = TakeFloor((-0.2f, 0.15f));
        using var _ = world;
        var before = world.View;
        var vol = before.WetPatchVolume.ToArray();
        var x = before.WetX.ToArray();
        var live = before.WetLive.ToArray();
        var gen = before.WetGeneration.ToArray();
        var inside = Enumerable.Range(0, before.WetCount).Where(w => live[w] != 0 && x[w] < 0f).ToArray();
        var outside = Enumerable.Range(0, before.WetCount).Single(w => live[w] != 0 && x[w] > 0f);
        var pooled = before.SensorWetVolume[s[0]];
        var bodyBefore = before.WetVolume[0];
        var taken = new float[1];
        world.TakeWet(s, [10f], taken);
        var v = world.View;
        Assert.Equal(pooled, taken[0]);
        foreach (var w in inside)
        {
            Assert.Equal(0u, v.WetLive[w]);
            Assert.Equal(gen[w] + 1, v.WetGeneration[w]);
        }
        Assert.Equal(vol[outside], v.WetPatchVolume[outside]);
        Assert.Equal(1u, v.WetLive[outside]);
        Assert.Equal(bodyBefore - pooled, v.WetVolume[0], bodyBefore * 1e-5f);
        world.Step(Dt, Sub);
        Assert.Equal(0f, world.View.SensorWetVolume[s[0]]);
    }

    [Fact]
    public void TakeWet_nothing_to_take_changes_nothing()
    {
        var (world, s) = TakeFloor((-0.2f, 0.15f), (0.1f, 0.05f)); // the second is over bare floor
        using var _ = world;
        var wet = WetBytes(world.View);
        var body = world.View.WetVolume[0];
        var taken = new float[] { 9f, 9f };
        world.TakeWet(s, [0f, 1f], taken);
        Assert.Equal([0f, 0f], taken);
        Assert.True(wet.AsSpan().SequenceEqual(WetBytes(world.View)), "a wet lane changed");
        Assert.Equal(body, world.View.WetVolume[0]);
        world.TakeWet([], [], default);
        Assert.True(wet.AsSpan().SequenceEqual(WetBytes(world.View)));
    }

    [Fact]
    public void TakeWet_a_patch_on_a_solid_particle_is_taken_as_one_on_a_body()
    {
        using var world = new Box3DSoaWorld(4, gravityY: -9.81f, particles: new Box3DSoaParticleCapacities(256, 160, 140, 16, 1024, 2, 16, 4, 2));
        world.AddStaticBox(1, (0f, -50f, 0f), (1f, 1f, 1f));
        var sheet = Graph.Sheet(world, (0f, 0.6f, 0f), (0.04f, 0f, 0f), (0f, 0f, 0.04f), 8, 8, 0.02f, 0.02f, Box3DSoaPhase.Of(4, true, true));
        for (var j = 0; j < 8; j++)
            for (var i = 0; i < 8; i += 7)
                world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[j * 8 + i], point: (0.04f * i, 0.6f, 0.04f * j));
        var drops = new Box3DSoaParticleHandle[1];
        world.CreateParticles([Fluid((0.12f, 0.64f, 0.12f))], drops);
        for (var tick = 0; tick < 20; tick++) world.Step(1f / 60f, Sub);
        var under = world.View.SupportParticle[drops[0].Lane];
        Assert.True(under >= 0);
        world.Settle(drops);
        var v0 = world.View;
        var h = new Box3DSoaConstraintHandle[1];
        world.CreateSensors([(0, (v0.ParticleX[under], v0.ParticleY[under] + 50f, v0.ParticleZ[under]), 0.02f)], h);
        world.Step(Dt, Sub);
        var pooled = world.View.SensorWetVolume[h[0]];
        var vol = world.View.WetPatchVolume[0];
        Assert.True(pooled > 0f, "the patch on the solid particle is inside the sphere");
        Assert.Equal(pooled, vol);
        var taken = new float[1];
        world.TakeWet(h, [pooled / 4f], taken);
        Assert.Equal(vol * 0.75f, world.View.WetPatchVolume[0], vol * 1e-5f);
        Assert.Equal(WetRadiusOf(world.View.WetPatchVolume[0]), world.View.WetRadius[0], 1e-6f);
        Assert.Equal(pooled / 4f, taken[0], pooled * 1e-5f);
        world.TakeWet(h, [1f], taken);
        Assert.Equal(0u, world.View.WetLive[0]);
        Assert.Equal(1u, world.View.WetGeneration[0]);
        Assert.Equal(-1, world.View.WetParticle[0]);
    }

    [Fact]
    public void TakeWet_two_sensors_in_one_call()
    {
        // disjoint: the first over A and B, the second over C.
        var (world, s) = TakeFloor((-0.2f, 0.15f), (0.4f, 0.1f));
        using (world)
        {
            var (pa, pc) = (world.View.SensorWetVolume[s[0]], world.View.SensorWetVolume[s[1]]);
            Assert.True(pa > 0f && pc > 0f);
            var taken = new float[2];
            world.TakeWet(s, [pa / 2f, pc / 2f], taken);
            Assert.Equal(pa / 2f, taken[0], pa * 1e-5f);
            Assert.Equal(pc / 2f, taken[1], pc * 1e-5f);
            world.Step(Dt, Sub);
            Assert.Equal(pa / 2f, world.View.SensorWetVolume[s[0]], pa * 1e-5f);
            Assert.Equal(pc / 2f, world.View.SensorWetVolume[s[1]], pc * 1e-5f);
        }
        // overlapping: the same sphere twice: the second sees what the first left, asks for more than is there, and no patch goes below 0.
        var (world2, t) = TakeFloor((-0.2f, 0.15f), (-0.2f, 0.15f));
        using (world2)
        {
            var before = world2.View.WetPatchVolume.ToArray();
            var x = world2.View.WetX.ToArray();
            var pooled = world2.View.SensorWetVolume[t[0]];
            var taken = new float[2];
            var first = pooled / 3f;
            world2.TakeWet(t, [first, 10f], taken);
            Assert.Equal(first, taken[0], first * 1e-5f);
            Assert.Equal(pooled - first, taken[1], pooled * 1e-5f);
            var v = world2.View;
            for (var w = 0; w < v.WetCount; w++) Assert.True(v.WetPatchVolume[w] >= 0f && v.WetPatchVolume[w] <= before[w]);
            world2.Step(Dt, Sub);
            Assert.Equal(0f, world2.View.SensorWetVolume[t[0]]);
            Assert.Equal(0f, world2.View.SensorWetVolume[t[1]]);
            for (var w = 0; w < v.WetCount; w++) if (x[w] > 0f) Assert.Equal(before[w], world2.View.WetPatchVolume[w]); // C untouched
        }
    }

    [Fact]
    public void TakeWet_a_later_sensor_finds_its_patch_after_an_earlier_take_removed_one()
    {
        // A lies first in the body's index by centre x: removing it resets its centre to 0, which would put it between B and C and hide B from the second sensor's search.
        var (world, s) = TakeFloor((-0.3f, 0.05f), (-0.1f, 0.05f));
        using var _ = world;
        var pb = world.View.SensorWetVolume[s[1]];
        Assert.True(pb > 0f && world.View.SensorWetVolume[s[0]] > 0f);
        var taken = new float[2];
        world.TakeWet(s, [10f, 10f], taken);
        Assert.Equal(pb, taken[1]);
        Assert.Equal(1, Enumerable.Range(0, world.View.WetCount).Count(w => world.View.WetLive[w] != 0)); // only C is left
    }

    [Theory]
    [InlineData(1f / 3f)]
    [InlineData(1e-6f)]
    [InlineData(1e-9f)]
    public void TakeWet_taken_is_what_left_the_patches(float share)
    {
        // A sensor over A and B: taken is the patches' own loss (float rounding included), not the ask.
        var (world, s) = TakeFloor((-0.2f, 0.15f));
        using var _ = world;
        var pooled = world.View.SensorWetVolume[s[0]];
        var before = world.View.WetPatchVolume.ToArray();
        var taken = new float[1];
        world.TakeWet(s, [pooled * share], taken);
        var after = world.View.WetPatchVolume;
        double lost = 0;
        for (var w = 0; w < before.Length; w++) lost += (double)before[w] - after[w];
        Assert.Equal((float)lost, taken[0]);
    }

    [Fact]
    public void TakeWet_a_kill_takes_the_whole_pool()
    {
        var (world, s) = TakeFloor((-0.2f, 0.15f));
        using var _ = world;
        var pooled = world.View.SensorWetVolume[s[0]];
        var taken = new float[1];
        world.TakeWet(s, [pooled * 2f], taken);
        Assert.Equal(pooled, taken[0], pooled * 1e-6f);
    }

    [Fact]
    public void TakeWet_a_patch_an_earlier_sensor_killed_is_not_killed_again()
    {
        // the second sphere also holds A, which the first took to nothing: A's dead lane keeps its centre, and the walk must pass over it
        var (world, s) = TakeFloor((-0.3f, 0.05f), (-0.2f, 0.15f));
        using var _ = world;
        var gen = world.View.WetGeneration.ToArray();
        var live = world.View.WetLive.ToArray();
        var taken = new float[2];
        world.TakeWet(s, [10f, 10f], taken);
        Assert.True(taken[0] > 0f);
        Assert.True(taken[1] > 0f);
        var killed = 0;
        for (var w = 0; w < gen.Length; w++)
        {
            if (live[w] != 0 && world.View.WetLive[w] == 0)
            {
                killed++;
                Assert.Equal(gen[w] + 1, world.View.WetGeneration[w]);
            }
            else Assert.Equal(gen[w], world.View.WetGeneration[w]);
        }
        Assert.True(killed > 0);
    }

    [Fact]
    public void TakeWet_rederives_the_wet_index_once()
    {
        // A command ends in one rederive_wet; a kill never repairs the index mid-call (pinned on the source, as Remove_wet_never_reaches_the_topology_derivation is).
        var source = NativeSource("ymir_particles.c").Replace("\r\n", "\n");
        var at = source.IndexOf("int32_t ymir_particles_take_wet(", StringComparison.Ordinal);
        Assert.True(at > 0);
        var body = source.Substring(at, source.IndexOf("ymir_particles_create_sensors(", at, StringComparison.Ordinal) - at);
        Assert.Equal(1, System.Text.RegularExpressions.Regex.Matches(body, @"rederive_wet\(").Count);
    }

    [Fact]
    public void TakeWet_refusals_change_nothing()
    {
        var (world, s) = TakeFloor((-0.2f, 0.15f), (0.4f, 0.1f));
        using var _ = world;
        var wet = WetBytes(world.View);
        var good = world.View.SensorWetVolume[s[0]] * 0.5f;
        void Unchanged() => Assert.True(wet.AsSpan().SequenceEqual(WetBytes(world.View)), "a wet lane changed on a refusal");
        // the first sensor is valid and would take; each refusal is in the second place, so the first must not have run
        Assert.ThrowsAny<Exception>(() => world.TakeWet(s, [good, -1f])); Unchanged();
        Assert.ThrowsAny<Exception>(() => world.TakeWet(s, [good, float.NaN])); Unchanged();
        Assert.ThrowsAny<Exception>(() => world.TakeWet(s, [good, float.PositiveInfinity])); Unchanged();
        Assert.ThrowsAny<Exception>(() => world.TakeWet([s[0], s[0]], [good, good])); Unchanged();
        Assert.ThrowsAny<Exception>(() => world.TakeWet([s[0], s[1] with { Generation = s[1].Generation + 1 }], [good, good])); Unchanged();
        var gone = s[1];
        world.Remove(gone);
        Assert.ThrowsAny<Exception>(() => world.TakeWet([s[0], gone], [good, good])); Unchanged();
        Assert.Throws<ArgumentException>(() => world.TakeWet([new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Wet, 0, 0)], [good])); Unchanged();
        Assert.Throws<ArgumentException>(() => world.TakeWet([s[0]], [good, good])); Unchanged();
        Assert.Throws<ArgumentException>(() => world.TakeWet([s[0], s[0]], [good, good], new float[1])); Unchanged();
    }
}
