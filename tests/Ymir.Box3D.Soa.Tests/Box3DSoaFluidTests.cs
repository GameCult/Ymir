using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Fluid as a phase bit of the one particle solver: docs/soa-world.md, particles section, Fluid.</summary>
public sealed class Box3DSoaFluidTests
{
    private const float Dt = Box3DSoaParticleTests.Dt;
    private const int Sub = Box3DSoaParticleTests.Sub;
    private static readonly uint FluidPhase = Box3DSoaPhase.Of(0, fluid: true);

    private static Box3DSoaParticleDesc Fluid(Pos at, float mass = 0.02f, float radius = 0.0125f, Pos velocity = default, uint material = 0) =>
        new(at, mass, radius, FluidPhase, material, null, velocity);

    private sealed record Graphs(int Count, int[] Att, int[] Particle, int[] Parent, int[] Edge, int[] Other, float[] Bound, int[] Component);

    private static Graphs Snapshot(Box3DSoaView v) =>
        new(v.TetherCount, v.TetherAttachment.ToArray(), v.TetherParticle.ToArray(), v.TetherParent.ToArray(), v.TetherEdge.ToArray(),
            v.TetherOther.ToArray(), v.TetherBound.ToArray(), v.ParticleComponent.ToArray());

    private static void SameTethers(Graphs a, Graphs b)
    {
        Assert.Equal(a.Count, b.Count);
        Assert.Equal(a.Att, b.Att);
        Assert.Equal(a.Particle, b.Particle);
        Assert.Equal(a.Parent, b.Parent);
        Assert.Equal(a.Edge, b.Edge);
        Assert.Equal(a.Other, b.Other);
        Assert.True(a.Bound.AsSpan().SequenceEqual(b.Bound), "tether bounds differ");
    }

    [Fact]
    public void Fluid_refusals()
    {
        using var world = Box3DSoaParticleTests.World(particles: 16, distances: 16, bends: 16, attachments: 4, tethers: 64);
        var ps = new Box3DSoaParticleHandle[3];
        world.CreateParticles([new((0f, 0f, 0f), 0.02f, 0.0125f), Fluid((0.05f, 0f, 0f)), Fluid((0.1f, 0f, 0f))], ps);
        var view = world.View;
        Assert.Equal(FluidPhase, view.ParticlePhase[1]);
        // a fluid particle holds no distance, bend or attachment; every refusal happens before anything changes
        Assert.Throws<InvalidOperationException>(() => world.CreateDistances([new Box3DSoaDistanceDesc(ps[0], ps[1])]));
        Assert.Throws<InvalidOperationException>(() => world.CreateDistances([new Box3DSoaDistanceDesc(ps[1], ps[2])]));
        Assert.Throws<InvalidOperationException>(() => world.Attach(Box3DSoaAttachKind.Static, ps[1], point: (0.05f, 0f, 0f), tether: true));
        Assert.Throws<InvalidOperationException>(() => world.Attach(Box3DSoaAttachKind.Static, ps[2], point: (0.1f, 0f, 0f)));
        Assert.Equal(0, view.DistanceCount);
        Assert.Equal(0, view.AttachmentCount);
        Assert.Equal(0, view.TetherCount);
        Assert.Equal(16, world.FreeDistanceSlots);
        // the solids still take both
        var other = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new((0f, 0.05f, 0f), 0.02f, 0.0125f)], other);
        world.CreateDistances([new Box3DSoaDistanceDesc(ps[0], other[0])]);
        world.Attach(Box3DSoaAttachKind.Static, ps[0], point: (0f, 0f, 0f), tether: true);
        Assert.Equal(1, world.View.DistanceCount); // the view is a snapshot of the counts
    }

    [Fact]
    public void Fluid_emission_keeps_the_graph()
    {
        // A 32 x 32 sheet at 20 pins, then 64 fluid particles created every tick for 60 ticks and destroyed the tick after: creating a
        // particle changes no constraint and destroying fluid touches none, so the tether lanes never change and the full derivation
        // (3.6 to 4.0 ms on the 20th pin) is never run for them.
        using var world = Box3DSoaParticleTests.World(particles: 1024 + 256, distances: 2048, bends: 2048, attachments: 32, tethers: 4096);
        const float sp = 0.04f;
        var sheet = Graph.Sheet(world, (0f, 2f, 0f), (sp, 0f, 0f), (0f, -sp, 0f), 32, 32, 0.02f, 0.02f);
        var held = new HashSet<int>();
        for (var a = 0; a < 20; a++)
        {
            var i = a == 0 ? 0 : a == 1 ? 31 : a * 37 % 1024;
            if (held.Add(i)) world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[i], point: (i % 32 * sp, 2f - i / 32 * sp, 0f), tether: true);
        }
        var before = Snapshot(world.View);
        Assert.True(before.Count > 1900, $"{before.Count} tethers");
        Box3DSoaParticleHandle[]? previous = null;
        var emit = new TimeSpan();
        for (var tick = 0; tick < 60; tick++)
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();
            if (previous is not null) world.DestroyParticles(previous);
            var batch = new Box3DSoaParticleDesc[64];
            for (var k = 0; k < 64; k++) batch[k] = Fluid((5f + 0.05f * (k % 8), 3f + 0.05f * (k / 8), 0.05f * (tick % 4)));
            var made = new Box3DSoaParticleHandle[64];
            world.CreateParticles(batch, made);
            emit += watch.Elapsed;
            var view = world.View;
            var now = Snapshot(view);
            SameTethers(before, now);
            for (var k = 0; k < 1024; k++) Assert.Equal(before.Component[k], now.Component[k]); // the sheet's components
            foreach (var h in made) Assert.Equal(h.Lane, view.ParticleComponent[h.Lane]);
            if (previous is not null) foreach (var h in previous) Assert.Equal(0, view.ParticleComponent[h.Lane]);
            world.Step(Dt, Sub);
            previous = made;
        }
        world.DestroyParticles(previous!);
        var last = world.View;
        SameTethers(before, Snapshot(last));
        for (var t = 0; t < last.TetherCount; t++) // every tether's parent is present
            Assert.True(last.TetherParent[t] < t);
        var line = $"FLUID_EMIT 64 fluid particles created and 64 destroyed per tick on the 20-pin 32 x 32 sheet: {emit.TotalMilliseconds / 60:F3} ms per tick of commands";
        Console.Error.WriteLine(line);
        Readings.Add(line);
    }
}
