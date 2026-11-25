using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Fluid as a phase bit of the one particle solver: docs/soa-world.md, particles section, Fluid.</summary>
public sealed partial class Box3DSoaFluidTests
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

    private const float R = 0.0125f;
    private const float FluidMass = 0.015625f; // (2 r)^3 of water: 15.6 mL

    private static Box3DSoaWorld FluidWorld(int particles, int bodies = 8, float gravityY = -10f, int materials = 2, int wets = 0, int sensors = 0) =>
        new(bodies, gravityY: gravityY, particles: new Box3DSoaParticleCapacities(particles, 16, 16, 16, 64, materials, particles, wets, sensors));

    /// <summary>A floor and four walls: an open well of interior half-width <paramref name="half"/> centred on the y axis.</summary>
    private static void Well(Box3DSoaWorld world, float half)
    {
        world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
        world.AddStaticBox(2, (half + 0.25f, 1f, 0f), (0.25f, 1f, 1f), friction: 0.3f);
        world.AddStaticBox(3, (-half - 0.25f, 1f, 0f), (0.25f, 1f, 1f), friction: 0.3f);
        world.AddStaticBox(4, (0f, 1f, half + 0.25f), (1f, 1f, 0.25f), friction: 0.3f);
        world.AddStaticBox(5, (0f, 1f, -half - 0.25f), (1f, 1f, 0.25f), friction: 0.3f);
    }

    [Fact]
    public void Fluid_capacity_refuses_a_batch_whole()
    {
        // The fluid capacity sizes the neighbour arena: fluid past it is refused before any lane is taken, solids still go in, and a
        // destroyed fluid particle frees its place at once.
        using var world = Box3DSoaParticleTests.World(particles: 16, distances: 16, bends: 16, attachments: 4, tethers: 64, fluid: 3);
        var ps = new Box3DSoaParticleHandle[2];
        world.CreateParticles([Fluid((0f, 0f, 0f)), Fluid((0.05f, 0f, 0f))], ps);
        Assert.Throws<InvalidOperationException>(() => world.CreateParticles([Fluid((0.1f, 0f, 0f)), Fluid((0.15f, 0f, 0f))]));
        Assert.Equal(2, world.View.ParticleCount);
        Assert.Equal(14, world.FreeParticleSlots);
        world.CreateParticles([new((0f, 0.1f, 0f), 0.02f, 0.0125f), Fluid((0.1f, 0f, 0f))]);
        Assert.Equal(4, world.View.ParticleCount);
        Assert.Throws<InvalidOperationException>(() => world.CreateParticles([Fluid((0.15f, 0f, 0f))]));
        world.DestroyParticles([ps[0]]);
        world.CreateParticles([Fluid((0.15f, 0f, 0f))]);
    }

    [Fact]
    public void Fluid_refusals()
    {
        using var world = Box3DSoaParticleTests.World(particles: 16, distances: 16, bends: 16, attachments: 4, tethers: 64, fluid: 16);
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
        // a viscosity below 0 or not finite is refused, the material as it was
        world.SetMaterial(1, 0.3f, 7, 0.25f);
        Assert.Throws<InvalidOperationException>(() => world.SetMaterial(1, 0.3f, 8, -0.01f));
        Assert.Throws<InvalidOperationException>(() => world.SetMaterial(1, 0.3f, 8, float.NaN));
        Assert.Throws<InvalidOperationException>(() => world.SetMaterial(1, 0.3f, 8, float.PositiveInfinity));
        Assert.Equal(0.25f, world.View.MaterialViscosity[1]);
        Assert.Equal(7ul, world.View.MaterialTableId[1]);
    }

    [Fact]
    public void Fluid_emission_keeps_the_graph()
    {
        // A 32 x 32 sheet at 20 pins, then 64 fluid particles created every tick for 60 ticks and destroyed the tick after: creating a
        // particle changes no constraint and destroying fluid touches none, so the tether lanes never change and the full derivation
        // (3.6 to 4.0 ms on the 20th pin) is never run for them.
        using var world = Box3DSoaParticleTests.World(particles: 1024 + 256, distances: 2048, bends: 2048, attachments: 32, tethers: 4096, fluid: 1024);
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

    [Fact]
    public void Destroying_a_lone_attached_particle_leaves_the_tether_lanes_as_before_the_attach()
    {
        // A particle with a tether attachment and no distance: does it make a tether lane, and does destroying it leave one behind?
        using var world = Box3DSoaParticleTests.World(particles: 16, distances: 16, bends: 16, attachments: 4, tethers: 64, fluid: 16);
        var ps = new Box3DSoaParticleHandle[2];
        world.CreateParticles([new((0f, 0f, 0f), 0.02f, 0.0125f), new((1f, 0f, 0f), 0.02f, 0.0125f)], ps);
        var before = Snapshot(world.View);
        world.Attach(Box3DSoaAttachKind.Static, ps[0], point: (0f, 0f, 0f), tether: true);
        var held = world.View.TetherCount;
        world.DestroyParticles([ps[0]]);
        var line = $"HELD_ATTACHMENT tether lanes {before.Count} before the attach, {held} with a lone tethered particle, {world.View.TetherCount} after destroying it";
        Readings.Add(line);
        Console.Error.WriteLine(line);
        SameTethers(before, Snapshot(world.View));
    }

    // ---- scenes ---------------------------------------------------------------------------------------------

    private static float Sq(float x) => x * x;

    /// <summary>
    /// A fluid block thrown at a free 16 x 16 sheet and a 48-rope in zero gravity, nothing pinned: every particle is free, so
    /// momentum is the whole of it. Masses are per particle.
    /// </summary>
    private static Box3DSoaWorld Collision(float fluidMass, float solidMass)
    {
        var world = new Box3DSoaWorld(4, gravityY: 0f, particles: new Box3DSoaParticleCapacities(1024, 1024, 1024, 8, 64, 2, 1024));
        world.AddStaticBox(1, (0f, -100f, 0f), (1f, 1f, 1f));
        Graph.Sheet(world, (0f, 0f, 0f), (0.04f, 0f, 0f), (0f, 0.04f, 0f), 16, 16, solidMass, 0.02f, Box3DSoaPhase.Of(7, true, true));
        Graph.Rope(world, Graph.Line((-0.3f, 0.25f, 0.3f), (0.03f, 0f, 0f), 48), solidMass, 0.0125f, Box3DSoaPhase.Of(8, true, true), bend: 1e-5f);
        Graph.Block(world, (0.2f, 0.17f, 0.45f), 5, 5, 5, 0.02f, fluidMass, Box3DSoaPhase.Of(9, fluid: true), velocity: (0f, 0f, -4f));
        return world;
    }

    /// <summary>Drift of the total momentum over the run as a fraction of its initial size, and the highest kinetic energy over the initial.</summary>
    private static (double Drift, double EnergyRatio) Momentum(Box3DSoaWorld world, int ticks)
    {
        (double X, double Y, double Z, double E) Total()
        {
            var v = world.View;
            double x = 0, y = 0, z = 0, e = 0;
            for (var i = 0; i < v.ParticleCount; i++)
            {
                if (v.ParticleLive[i] == 0 || v.ParticleInvMass[i] == 0f) continue;
                double m = 1.0 / v.ParticleInvMass[i];
                x += m * v.ParticleVx[i]; y += m * v.ParticleVy[i]; z += m * v.ParticleVz[i];
                e += 0.5 * m * (Sq(v.ParticleVx[i]) + Sq(v.ParticleVy[i]) + Sq(v.ParticleVz[i]));
            }
            return (x, y, z, e);
        }
        var p0 = Total();
        var emax = p0.E;
        var worst = 0.0;
        var mag = Math.Sqrt(p0.X * p0.X + p0.Y * p0.Y + p0.Z * p0.Z);
        for (var t = 0; t < ticks; t++)
        {
            world.Step(1f / 60f, 4);
            var p = Total();
            emax = Math.Max(emax, p.E);
            worst = Math.Max(worst, Math.Sqrt(Sq((float)(p.X - p0.X)) + Sq((float)(p.Y - p0.Y)) + Sq((float)(p.Z - p0.Z))) / mag);
        }
        return (worst, emax / p0.E);
    }

    /// <summary>
    /// Probe scene 3: a 7 x 7 x 7 block above a 16 x 16 sheet tilted down toward +x (pinned at its corners) and a sloped 24-rope. The block is fluid, or
    /// with <paramref name="solid"/> the same particles flagged solid (group 6, self-collide) as the control.
    /// </summary>
    private static (Box3DSoaWorld World, Made Sheet, Made Rope, Box3DSoaParticleHandle[] Fluid) Pour(bool nudge = false, bool solid = false)
    {
        var world = new Box3DSoaWorld(4, gravityY: -9.81f, particles: new Box3DSoaParticleCapacities(640, 560, 520, 8, 1024, 2, 640)); // the probe's g
        world.AddStaticBox(1, (0f, -50f, 0f), (1f, 1f, 1f));
        world.SetMaterial(1, 0.3f, 0, 0.01f); // the probe's XSPH coefficient
        const float sp = 0.04f, r = 0.02f;
        var sheet = Graph.Sheet(world, (0f, 0.6f, 0f), (sp, -0.012f, 0f), (0f, 0f, sp), 16, 16, 0.02f, r, Box3DSoaPhase.Of(4, true, true), bend: 1e-3f);
        foreach (var i in new[] { 0, 15, 240, 255 })
        {
            var v = world.View;
            world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[i], point: (v.ParticleX[i], v.ParticleY[i], v.ParticleZ[i])); // plain pins, as the probe's
        }
        var line = Graph.Line((0.55f, 0.25f, 0.30f), ((1.40f - 0.55f) / 23f, -0.40f / 23f, 0f), 24);
        var rope = Graph.Rope(world, line, 0.02f, r, Box3DSoaPhase.Of(5), bend: 1e-5f);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: line[0], tether: true);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[23], point: line[23], tether: true);
        var desc = new Box3DSoaParticleDesc[343];
        for (var a = 0; a < 7; a++) for (var b = 0; b < 7; b++) for (var c = 0; c < 7; c++)
            desc[(a * 7 + b) * 7 + c] = new Box3DSoaParticleDesc((0.10f + a * sp, 0.75f + b * sp, 0.18f + c * sp), 1000f * sp * sp * sp, r, solid ? Box3DSoaPhase.Of(6, true, false) : Box3DSoaPhase.Of(6, fluid: true), 1);
        if (nudge) desc[100].X = BitConverter.UInt32BitsToSingle(BitConverter.SingleToUInt32Bits(desc[100].X) + 1);
        var fluid = new Box3DSoaParticleHandle[desc.Length];
        world.CreateParticles(desc, fluid);
        return (world, sheet, rope, fluid);
    }

    // ---- behaviour ------------------------------------------------------------------------------------------

    // Bounds follow lodur:ruling:deterministic-bound-margin: the geometric mean of the shipped worst and the least claimed mutant, separation >= 1.2.
    // |mean - 1| at 3 s, by lodur:ruling:ymir-column-mean-bound-order-noise: summation order is not behaviour, so the shipped worst is taken over twelve orders
    // of the fluid pass's bucket offsets (the six axis permutations, each ascending and descending): 0.0178 worst (0.0027 to 0.0178). Md (fluid-fluid pairs done
    // from both rows) reads 0.4625 to 0.4865: bound sqrt(0.0178 x 0.4625) = 0.0907, separation 5.1 each side (it was 0.0137 against one order's reading, and one order broke it).
    // The mean no longer separates the others: Mk (fluid cell 2 rmax, 1 ring) 0.0142 to 0.0182, Mx (fluid pairs also contact; the column sets SELF_COLLIDE) 0.0038 to 0.0154,
    // Mu (bucket dedupe deleted) 0.0013 to 0.0058, Mf (fluid-solid contact deleted; the column holds no solid particle) unchanged: Mk is claimed by KERNEL_REACH, Mf by the floor
    // penetration and the through count, Mu and Mx are unclaimed (Mu only moves the FluidHashes pin). The max density reads 1.2992 to 1.3795 shipped over the twelve orders and
    // 1.7133 to 1.7550 under Md: Md is 1.14 away, the midpoint 1.537 would rise, so the sanity ceiling 1.5 stays and also claims Md (the
    // column compressing under its weight is the one-pass solver, not a defect: probe pours peaked at 1.07). Mo (capacity 8) overflows the lists
    // 377700 times in the column against 0 shipped (half a unit by lodur:ruling:deterministic-bound-zero-reading): the overflow assert is exact
    // zero, below the rule's midpoint, because the count is exact and any overflow means pairs were lost.
    // Unclaimed since the fix cut, readings of its build, a follow-up: Mc1 (lambda halved) max 1.2116, Mc2 (pass A without the self term) max 1.3340.
    private const float ColumnMax = 1.5f;
    private const float ColumnMean = 0.0907f;
    private const float FloorPenetration = 0.233f;
    private const int FloorThrough = 18;

    [Fact]
    public void Fluid_column_rests_at_rest_density()
    {
        // A 8 x 8 x 16 block (r 1.25 cm, water mass) resting in a static box well, 3 s at 60 Hz x 4 substeps. The density constraint is
        // one pass a substep, so the column compresses under its own weight and its walls read low (a rigid box adds no density
        // to the estimate, docs/soa-world.md Not done); what is judged is that the max stays under the bound and the mean near 1.
        using var world = FluidWorld(1024);
        const float half = 0.105f;
        Well(world, half);
        // Alternate groups and set both self flags: two fluid particles share one density field whatever their group (Flex sec. 7), so
        // neither a group boundary nor SELF_COLLIDE makes a contact between them.
        var desc = new Box3DSoaParticleDesc[8 * 16 * 8];
        for (var k = 0; k < 8; k++) for (var j = 0; j < 16; j++) for (var i = 0; i < 8; i++)
            desc[(k * 16 + j) * 8 + i] = new Box3DSoaParticleDesc((-0.0875f + 2 * R * i, R + 2 * R * j, -0.0875f + 2 * R * k), FluidMass, R,
                Box3DSoaPhase.Of((uint)((i + j + k) % 2 + 1), true, true, true));
        var block = new Box3DSoaParticleHandle[desc.Length];
        world.CreateParticles(desc, block);
        var worst = 0f;
        var overflow = 0;
        for (var tick = 0; tick < 180; tick++)
        {
            world.Step(1f / 60f, 4);
            overflow = Math.Max(overflow, world.View.FluidNeighbourOverflow);
            if (tick < 120) continue;
            var d = world.View.ParticleDensity;
            foreach (var h in block) worst = MathF.Max(worst, d[h.Lane]);
        }
        var v = world.View;
        double mean = 0;
        var outside = 0;
        foreach (var h in block)
        {
            mean += v.ParticleDensity[h.Lane];
            if (MathF.Abs(v.ParticleX[h.Lane]) > half || MathF.Abs(v.ParticleZ[h.Lane]) > half || v.ParticleY[h.Lane] < -0.01f) outside++;
        }
        mean /= block.Length;
        var line = $"COLUMN max density over the last second {worst:F4}, mean at 3 s {mean:F4}, outside the well {outside}, neighbour overflow {overflow}";
        Readings.Add(line);
        Assert.Equal(0, outside);
        Assert.Equal(0, overflow);
        Assert.True(worst <= ColumnMax, $"{line}: max over {ColumnMax}");
        Assert.True(Math.Abs(mean - 1.0) <= ColumnMean, $"{line}: mean off 1 by over {ColumnMean}");
    }

    [Fact]
    public void Fluid_density_counts_a_larger_solid_beyond_two_contact_radii()
    {
        // The kernel reach of a pair is 2 (ri + rj), so a fluid particle sees a solid of radius 5 cm 12.5 cm away while contact reaches
        // 6.25 cm: the fluid grid cell is 4 r_fluid (5 cm) and the fluid pass walks 3 rings of them. Placed three cells apart (10.15 cm), the solid is in
        // the fluid particle's estimate only if the rings are derived from the kernel reach (Mk forces the cell to 2 r_max and 1 ring: the solid is then two cells away).
        double Density(bool solid)
        {
            using var world = FluidWorld(256, gravityY: 0f); // a table of 512 buckets: a bucket two cells share could hide the miss
            world.CreateParticles([Fluid((0.099f, 0f, 0f))]);
            if (solid) world.CreateParticles([new Box3DSoaParticleDesc((0.2005f, 0f, 0f), 1f, 0.05f, Box3DSoaPhase.Of(1))]);
            world.Step(1f / 60f, 4);
            return world.View.ParticleDensity[0];
        }
        var alone = Density(false);
        var near = Density(true);
        var rise = near / alone - 1.0;
        var line = $"KERNEL_REACH fluid density {alone:F6} alone, {near:F6} with a 5 cm solid 10.15 cm away: rise {rise:E3}";
        Readings.Add(line);
        Assert.True(rise >= KernelReachBound, line);
    }

    // Relative rise of the fluid particle's density: shipped 2.530e-3, Mk (cell 2 r_max, 1 ring) 0 (a table of 512 buckets, as 16 would hide the miss; Mu, dedupe deleted, 5.06e-2), which counts one unit (1e-6) by
    // lodur:ruling:deterministic-bound-zero-reading: bound sqrt(2.530e-3 x 1e-6) = 5.0e-5, separation 50.
    private const double KernelReachBound = 5.0e-5;

    // Drift as a fraction of the initial momentum, worst of the four mass ratios: shipped 7.71e-4 (fluid : solid 1 : 30; 6e-6 to 1e-5, the float32
    // floor as Contact_conserves_momentum's, at the other three). Mutants, worst row / least row: M1 (solid end of pass B not moved) 0.20 / 2.5e-2;
    // M3 (each end takes the whole correction) 0.148 / 2.06e-3. The midpoint sqrt(5.72e-4 x 2.5e-2) = 3.8e-3 would rise, so the bound keeps
    // its 2.0e-3 (never loosened); shipped is 3.5 under it and M1 and M3 fail it on every row but M3's 100 : 1.
    // Kinetic energy rise (max / initial - 1) on every row: shipped reads 1.000000, no rise at the print's 1e-6 (half a unit, 5e-7, by
    // lodur:ruling:deterministic-bound-zero-reading); M9 (the pair split 2 w / (wi + wj) over a unit-mass lambda) rises 0.108 at 30 : 1 and 0.130 at
    // 100 : 1. The midpoint sqrt(5e-7 x 0.108) = 2.3e-4 would loosen the standing 1e-5, which stays: separation 1.1e4.
    private const double MomentumBound = 2.0e-3;
    private const double EnergyBound = 1e-5;

    [Theory]
    [InlineData(0.064f, 0.02f)]       // fluid water, solids as the pour
    [InlineData(0.002f, 0.06f)]       // fluid : solid 1 : 30
    [InlineData(0.06f, 0.002f)]       // fluid : solid 30 : 1
    [InlineData(0.1f, 0.001f)]        // fluid : solid 100 : 1
    public void Fluid_solid_momentum_zero_g(float fluidMass, float solidMass)
    {
        // A fluid block thrown into a free sheet and a free rope in zero gravity for 2 s: every pair (fluid and fluid, fluid and solid, solid
        // and solid) moves its two ends by equal and opposite momentum, so the total may not drift and the kinetic energy may not rise,
        // at any mass ratio. The bound is the float32 floor of the scene, as Contact_conserves_momentum's.
        using var world = Collision(fluidMass, solidMass);
        var (drift, energy) = Momentum(world, 120);
        var line = $"MOMENTUM_FLUID m_f {fluidMass} m_s {solidMass}: drift {drift:E3} of the initial momentum, kinetic energy max/initial {energy:F6}";
        Readings.Add(line);
        Assert.True(drift <= MomentumBound, line);
        Assert.True(energy <= 1 + EnergyBound, line);
    }

    // The pour is read, not judged: at the scene's own 4 substeps its sheet stretches under the block's weight until a particle the sheet's own size
    // passes between the pins (the sieve, lodur:question:ymir-sheet-sieve-at-budget-substeps), and the same block flagged solid tunnels worse, so no count of it
    // claims a mutant of the fluid. What fluid owns is pinned by Fluid_holds_on_a_solid_floor; this test runs the scene and prints both readings.
    private static (string Line, int Overflow) PourReading(bool solid)
    {
        var (world, sheet, rope, fluid) = Pour(solid: solid);
        using (world)
        {
            var touchedSheet = new bool[fluid.Length];
            var touchedRope = new bool[fluid.Length];
            var deep = 0; var minRatio = 9f; var maxStretch = 0f; var overflow = 0;
            var solids = sheet.Particles.Select(h => (h.Lane, Sheet: true)).Concat(rope.Particles.Select(h => (h.Lane, Sheet: false))).ToArray();
            for (var tick = 0; tick < 240; tick++)
            {
                world.Step(1f / 60f, Sub);
                var v = world.View;
                overflow = Math.Max(overflow, v.FluidNeighbourOverflow);
                foreach (var e in sheet.Edges)
                    maxStretch = MathF.Max(maxStretch, Graph.Dist(v, v.DistanceA[e.Lane], v.DistanceB[e.Lane]) / v.DistanceRest[e.Lane] - 1f);
                for (var k = 0; k < fluid.Length; k++)
                {
                    var f = fluid[k].Lane;
                    foreach (var (s, onSheet) in solids)
                    {
                        var reach = v.ParticleRadius[f] + v.ParticleRadius[s];
                        var d = Graph.Dist(v, f, s);
                        if (d < 1.25f * reach) { if (onSheet) touchedSheet[k] = true; else touchedRope[k] = true; }
                        if (d < 0.5f * reach) deep++;
                        minRatio = MathF.Min(minRatio, d / reach);
                    }
                }
            }
            var view = world.View;
            var below = fluid.Count(h => view.ParticleY[h.Lane] < -0.2f);
            return ($"POUR_{(solid ? "SOLID" : "FLUID")} at {Sub} substeps: touched the sheet {touchedSheet.Count(b => b)} of {fluid.Length}, the rope {touchedRope.Count(b => b)}, below y = -0.2 m at 4 s {below}, " +
                   $"within half a contact distance of a solid (tick x particle pairs) {deep} (closest {minRatio:F3} of a contact distance), worst sheet edge stretch {maxStretch:F3}, neighbour overflow {overflow}", overflow);
        }
    }

    [Fact]
    public void Fluid_pours_on_a_sheet_and_off_a_rope()
    {
        foreach (var solid in new[] { false, true })
        {
            var (line, overflow) = PourReading(solid);
            Console.Error.WriteLine(line);
            Readings.Add(line);
            Assert.Equal(0, overflow);
        }
    }

    // A fluid block dropped 15 cm onto a floor of immovable solid particles (mass 0). Two readings, both over the 4 s at 4 substeps:
    // penetration = 1 - closest fluid-floor distance / contact distance (sampled at tick ends), and the fluid particles that went below the floor plane
    // while over the floor's interior (a pitch in from every edge: a flat floor has no walls, so the rest spill off its edge, 210 of 343 by 4 s).
    // Penetration: shipped 0.186, Mt1 (collides returns 0 for any fluid-solid pair) 0.292, Mt2 (density_pair counts fluid-fluid only) 0.291: bound
    // sqrt(0.187 x 0.291) = 0.233, separation 1.56 (set at the fix cut's 0.187; the midpoint with 0.186 is 0.233 too). Mr (rings forced to 1) reads 0.258 and
    // Md (fluid-fluid pairs done from both rows) 0.401 with 22 through: both fail the bounds. Mo (capacity 8) reads 0.629 and 25823 overflows. Through: shipped 1, Mt1 343, Mt2 1: bound sqrt(1 x 343) = 18, separation 343. The shipped 1 is the
    // block's centre column, exactly over a hole of the floor lattice: by the lattice symmetry the contacts push it sideways at the hole's plane and the
    // column's weight takes it through. Nowhere near half a contact distance deep, so not the fluid defect the spec stops on; reported.
    // At the kernel-cell cut (lodur:cut_spec:cut-ymir-fluid-cost.r2): shipped 0.134 and 0 through; Mk (fluid cell 2 r_max, 1 ring) 0.171 and 0 (unclaimed here, KERNEL_REACH claims it); Md 0.435 and 10 through; Mf (fluid-solid contact deleted) 0.945 and 259 through, claimed by both; Mo (arena 8 a fluid lane) 0.843 and 13650 overflows.
    private static (float Penetration, int Below, int Overflow) Floor(out string line)
    {
        using var world = FluidWorld(640, gravityY: -9.81f);
        world.SetMaterial(1, 0.3f, 0, 0.01f);
        const float pitch = 0.04f, r = 0.02f;
        var floor = Graph.Block(world, (0f, 0f, 0f), 16, 1, 16, r, 0f, Box3DSoaPhase.Of(4), pitch: pitch);
        var fluid = Graph.Block(world, (0.18f, 0.15f, 0.18f), 7, 7, 7, r, 1000f * pitch * pitch * pitch, Box3DSoaPhase.Of(6, fluid: true), material: 1, pitch: pitch);
        var closest = 9f;
        var overflow = 0;
        var through = new bool[fluid.Length];
        for (var tick = 0; tick < 240; tick++)
        {
            world.Step(1f / 60f, Sub);
            var v = world.View;
            overflow = Math.Max(overflow, v.FluidNeighbourOverflow);
            for (var k = 0; k < fluid.Length; k++) // a particle that spills off the floor's edge is not below it: only one over the floor's interior (a pitch in from every edge) counts
            {
                var l = fluid[k].Lane;
                if (v.ParticleY[l] < 0f && v.ParticleX[l] >= pitch && v.ParticleX[l] <= 14f * pitch && v.ParticleZ[l] >= pitch && v.ParticleZ[l] <= 14f * pitch) through[k] = true;
            }
            foreach (var f in fluid)
                foreach (var s in floor)
                    closest = MathF.Min(closest, Graph.Dist(v, f.Lane, s.Lane) / (v.ParticleRadius[f.Lane] + v.ParticleRadius[s.Lane]));
        }
        var view = world.View;
        var below = through.Count(b => b);
        var spilled = fluid.Count(h => view.ParticleY[h.Lane] < 0f);
        var penetration = MathF.Max(0f, 1f - closest);
        line = $"FLOOR a 7 x 7 x 7 fluid block dropped 15 cm onto a 16 x 16 immovable floor at {Sub} substeps: penetration {penetration:F3} of a contact distance, fluid that went below the floor plane over its interior {below} of {fluid.Length} ({spilled} below it at 4 s, the rest of them over the edge), neighbour overflow {overflow}";
        Readings.Add(line);
        return (penetration, below, overflow);
    }

    [Fact]
    public void Fluid_holds_on_a_solid_floor()
    {
        var (penetration, below, overflow) = Floor(out var line);
        Assert.Equal(0, overflow);
        Assert.True(penetration <= FloorPenetration, line + $": penetration over {FloorPenetration}");
        Assert.True(below <= FloorThrough, line + $": more than {FloorThrough} through");
    }

    // The support lanes name contact in the last substep, not rest: the step writes them and nothing reads them, so these tests read them.
    private static void ReadSupport(string scene, Box3DSoaView v, Box3DSoaParticleHandle[] fluid)
    {
        var body = fluid.Count(h => v.SupportBody[h.Lane] >= 0);
        var particle = fluid.Count(h => v.SupportParticle[h.Lane] >= 0);
        Readings.Add($"SUPPORT {scene}: {fluid.Length} fluid particles, {body} read a body, {particle} read a solid particle");
    }

    [Fact]
    public void Support_lanes_name_what_fluid_rests_on()
    {
        // Static floor and walls: a 4 x 4 x 4 pool in a well. Whoever touches the floor or a wall (the highest lane wins) reads that body; a particle
        // resting only on other fluid reads -1 on both lanes, as does a drop in flight. A drop pinched between the floor (lane 0) and a slab 1.9 r above
        // it (lane 5) touches both: the slab, the higher lane.
        {
            using var world = FluidWorld(256, gravityY: -9.81f);
            const float half = 0.1f;
            Well(world, half);
            var slab = world.AddStaticBox(6, (1f, 1.9f * R + 0.05f, 0f), (0.1f, 0.05f, 0.1f));
            var pool = Graph.Block(world, (-0.0625f, R + 0.005f, -0.0625f), 5, 8, 5, R, FluidMass, FluidPhase);
            var pinched = new Box3DSoaParticleHandle[1];
            world.CreateParticles([Fluid((1f, R, 0f))], pinched);
            var flight = new Box3DSoaParticleHandle[1];
            world.CreateParticles([Fluid((-1f, 40f, 0f))], flight);
            var thrown = new Box3DSoaParticleHandle[1];
            world.CreateParticles([Fluid((-1f, 0.5f * R, 1f), velocity: (0f, 1f, 0f))], thrown); // inside the floor, moving up
            Assert.Equal(5, slab);
            var view = world.View;
            Assert.Equal(-1, view.SupportBody[pool[0].Lane]);
            world.Step(1f / 240f, 1); // one substep: the thrown drop touches the floor in it
            Assert.Equal(0, world.View.SupportBody[thrown[0].Lane]);
            world.Step(1f / 60f, Sub); // four more: it is in flight in each, so a lane the step did not clear would still read the floor
            Assert.Equal(-1, world.View.SupportBody[thrown[0].Lane]);
            for (var tick = 0; tick < 119; tick++) world.Step(1f / 60f, Sub);
            view = world.View;
            // Every pool particle falls in one class by its distance to the nearest face. Gravity presses the floor layer every substep, so it must read;
            // a particle clear of every face by 5% must read -1 (resting on fluid); one in between may read either, but only a face it is within 5% of
            // (a contact pushes a particle out to distance r exactly, where it may or may not touch again).
            int floor = 0, resting = 0, band = 0;
            foreach (var h in pool)
            {
                var (x, y, z) = (view.ParticleX[h.Lane], view.ParticleY[h.Lane], view.ParticleZ[h.Lane]);
                var near = new List<int>();
                if (y <= 1.05f * R) near.Add(0);
                if (half - x <= 1.05f * R) near.Add(1);
                if (half + x <= 1.05f * R) near.Add(2);
                if (half - z <= 1.05f * R) near.Add(3);
                if (half + z <= 1.05f * R) near.Add(4);
                var body = view.SupportBody[h.Lane];
                var where = $"x {x:F5} y {y:F5} z {z:F5} body {body}";
                if (y <= 1.001f * R) { floor++; Assert.True(body >= 0 && body <= 4, where); }
                else if (near.Count == 0) { resting++; Assert.Equal(-1, body); }
                else { band++; Assert.True(body == -1 || near.Contains(body), where); }
                Assert.Equal(-1, view.SupportParticle[h.Lane]);
            }
            Assert.Equal(pool.Length, floor + resting + band);
            Assert.True(floor >= 100 && resting >= 10, $"{floor} on the floor, {resting} resting on fluid, {band} in the band");
            Assert.Equal(5, view.SupportBody[pinched[0].Lane]);
            Assert.Equal(-1, view.SupportParticle[pinched[0].Lane]);
            Assert.Equal(-1, view.SupportBody[flight[0].Lane]);
            Assert.Equal(-1, view.SupportParticle[flight[0].Lane]);
            // Destroy clears both lanes at once: a destroyed floor particle reads -1 before any step, not its last support.
            var floorPick = pool.First(h => view.ParticleY[h.Lane] <= 1.001f * R);
            Assert.True(view.SupportBody[floorPick.Lane] >= 0);
            world.DestroyParticles([floorPick]);
            Assert.Equal(-1, world.View.SupportBody[floorPick.Lane]);
            Assert.Equal(-1, world.View.SupportParticle[floorPick.Lane]);
            Readings.Add($"SUPPORT well classes: {floor} floor, {resting} resting on fluid, {band} band of {pool.Length}");
            ReadSupport("well", view, pool);
        }
        // A free box on the floor, a pool on the box: the box's lane for the layer that touches it, -1 above, never the floor's.
        {
            using var world = FluidWorld(128, gravityY: -9.81f);
            world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
            var box = world.AddBox(2, (0f, 0.1f, 0f), (0.1f, 0.1f, 0.1f), 5000f, friction: 0.3f);
            var pool = Graph.Block(world, (-0.025f, 0.2f + R + 0.005f, -0.025f), 3, 7, 3, R, FluidMass, FluidPhase);
            for (var tick = 0; tick < 12; tick++) world.Step(1f / 60f, Sub);
            var view = world.View;
            var top = view.PositionY[box] + 0.1f;
            var touching = 0; var resting = 0;
            foreach (var h in pool)
            {
                var y = view.ParticleY[h.Lane];
                if (MathF.Abs(view.ParticleX[h.Lane] - view.PositionX[box]) > 0.1f - R || MathF.Abs(view.ParticleZ[h.Lane] - view.PositionZ[box]) > 0.1f - R) continue; // spilled over the edge
                if (y <= top + 1.001f * R) { touching++; Assert.Equal(box, view.SupportBody[h.Lane]); }
                else if (y > top + 1.5f * R) { resting++; Assert.Equal(-1, view.SupportBody[h.Lane]); }
                Assert.Equal(-1, view.SupportParticle[h.Lane]);
            }
            Assert.True(touching >= 4 && resting >= 4, $"{touching} touch the box, {resting} above them");
            ReadSupport("free box", view, pool);
        }
        // A sheet pinned along two opposite edges (one edge alone swings down like a curtain; a corner-pinned tarp sieves, lodur:ruling:ymir-sheet-sieve-at-budget-substeps) under a few drops: each reads a
        // sheet particle that touches it; a drop high above reads -1 on both.
        {
            using var world = new Box3DSoaWorld(4, gravityY: -9.81f, particles: new Box3DSoaParticleCapacities(256, 160, 140, 16, 1024, 2, 16));
            world.AddStaticBox(1, (0f, -50f, 0f), (1f, 1f, 1f));
            var sheet = Graph.Sheet(world, (0f, 0.6f, 0f), (0.04f, 0f, 0f), (0f, 0f, 0.04f), 8, 8, 0.02f, 0.02f, Box3DSoaPhase.Of(4, true, true));
            for (var j = 0; j < 8; j++)
                for (var i = 0; i < 8; i += 7)
                    world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[j * 8 + i], point: (0.04f * i, 0.6f, 0.04f * j));
            var drops = new Box3DSoaParticleHandle[4];
            world.CreateParticles([Fluid((0.12f, 0.64f, 0.12f)), Fluid((0.16f, 0.64f, 0.16f)), Fluid((0.2f, 0.64f, 0.08f)), Fluid((0.15f, 60f, 0.14f))], drops);
            world.Step(1f / 60f, Sub);
            // a drop 4 cm above a sheet particle sits inside the kernel reach and so in its row's list, and does not touch (4 cm against 3.25): no support
            Assert.Equal(-1, world.View.SupportParticle[drops[0].Lane]);
            Assert.Equal(-1, world.View.SupportParticle[drops[1].Lane]);
            for (var tick = 1; tick < 20; tick++) world.Step(1f / 60f, Sub);
            var view = world.View;
            var onSheet = new HashSet<int>(sheet.Particles.Select(h => h.Lane));
            var read = 0;
            foreach (var h in drops.Take(3))
            {
                var s = view.SupportParticle[h.Lane];
                Assert.Equal(-1, view.SupportBody[h.Lane]);
                Assert.True(s >= 0, $"drop at lane {h.Lane} reads no sheet particle");
                read++;
                Assert.Contains(s, onSheet);
                Assert.True(Graph.Dist(view, h.Lane, s) <= 1.05f * (view.ParticleRadius[h.Lane] + view.ParticleRadius[s]), "the named particle does not touch");
            }
            Assert.Equal(3, read);
            Assert.Equal(-1, view.SupportParticle[drops[3].Lane]);
            Assert.Equal(-1, view.SupportBody[drops[3].Lane]);
            ReadSupport("sheet", view, drops);
            // A solid destroyed under a drop leaves no name behind: the drop is alive and reads -1 before any step.
            var under = view.SupportParticle[drops[0].Lane];
            world.DestroyParticles([sheet.Particles.Single(h => h.Lane == under)]);
            Assert.Equal(-1, world.View.SupportParticle[drops[0].Lane]);
            Assert.Equal(1, Math.Sign(world.View.ParticleRadius[drops[0].Lane]));
        }
    }

    [Fact]
    public void Fluid_pushes_a_body()
    {
        // A fluid block thrown down on a free 1 kg box in zero gravity: the contact and gravity terms are 0, so the box's M dv in a tick is
        // the impulse the particles gave it, which the lane reads. Fluid is booked on a body as any particle is.
        using var world = FluidWorld(256, gravityY: 0f);
        const float mass = 1f;
        var box = world.AddBox(2, (0f, 0f, 0f), (0.1f, 0.1f, 0.1f), mass / (8f * 0.1f * 0.1f * 0.1f), friction: 0.3f);
        Graph.Block(world, (-0.0375f, 0.13f, -0.0375f), 4, 4, 4, R, FluidMass, FluidPhase, velocity: (0f, -3f, 0f));
        var view = world.View;
        var (vx, vy, vz) = (view.VelocityX[box], view.VelocityY[box], view.VelocityZ[box]);
        float worst = 0f, peak = 0f;
        for (var tick = 0; tick < 120; tick++)
        {
            world.Step(1f / 60f, 4);
            var dx = mass * (view.VelocityX[box] - vx) - view.ParticleImpulseX[box];
            var dy = mass * (view.VelocityY[box] - vy) - view.ParticleImpulseY[box];
            var dz = mass * (view.VelocityZ[box] - vz) - view.ParticleImpulseZ[box];
            worst = MathF.Max(worst, MathF.Max(MathF.Abs(dx), MathF.Max(MathF.Abs(dy), MathF.Abs(dz))));
            peak = MathF.Max(peak, MathF.Abs(view.ParticleImpulseY[box]));
            (vx, vy, vz) = (view.VelocityX[box], view.VelocityY[box], view.VelocityZ[box]);
        }
        var line = $"PUSH peak impulse {peak:F5} N s a tick, worst |M dv - lane| {worst:E3} N s ({worst / peak:P3} of the peak)";
        Readings.Add(line);
        Assert.True(peak > 0.01f, line);
        Assert.True(worst <= 0.005f * peak, line);
    }

    // Drift of the mean nearest-neighbour distance in micrometres: shipped 0 (reads 0.000 at 1e-3, so half a unit, 5e-4), M4 (bilateral C) 1.529e7:
    // bound sqrt(5e-4 x 1.529e7) = 87 micrometres by lodur:ruling:deterministic-bound-zero-reading, separation 1.8e5.
    private const double ClumpBound = 87.0;

    [Fact]
    public void Fluid_does_not_clump()
    {
        // Particles on a lattice at 3 r spacing are under-dense: a unilateral constraint (C = max(n / n0 - 1, 0)) leaves them where they
        // are. A bilateral one pulls them together to rest density.
        using var world = FluidWorld(512, gravityY: 0f);
        world.AddStaticBox(1, (0f, -100f, 0f), (1f, 1f, 1f));
        var block = Graph.Block(world, (0f, 0f, 0f), 6, 6, 6, R, FluidMass, FluidPhase, pitch: 3f * R);
        double Mean()
        {
            var v = world.View;
            double sum = 0;
            foreach (var a in block)
            {
                var nearest = float.MaxValue;
                foreach (var b in block) if (a.Lane != b.Lane) nearest = MathF.Min(nearest, Graph.Dist(v, a.Lane, b.Lane));
                sum += nearest;
            }
            return sum / block.Length;
        }
        var before = Mean();
        for (var tick = 0; tick < 60; tick++) world.Step(1f / 60f, 4);
        var after = Mean();
        var line = $"CLUMP mean nearest-neighbour distance {before * 1e3:F6} mm before, {after * 1e3:F6} mm after 1 s (pitch {3f * R * 1e3:F4} mm), drift {Math.Abs(after - before) * 1e6:F3} micrometres";
        Readings.Add(line);
        Assert.True(Math.Abs(after - before) * 1e6 <= ClumpBound, line);
    }

    // Variance ratio, viscosity 0.1 over 0: shipped 0.7799, M5 (XSPH skipped) and Mv (the viscous-material scan finds none, so the skip flag ignores the materials) 1.0000: bound sqrt(0.7799 x 1.0000) = 0.883, separation 1.28.
    private const double ShearBound = 0.883;

    [Fact]
    public void Fluid_viscosity_damps_shear()
    {
        // The same block with a shear velocity field (vx by layer) for 0.5 s in zero gravity: XSPH at 0.1 takes the velocity variance
        // under the bound, at 0 it stays.
        static double Variance(float viscosity)
        {
            using var world = FluidWorld(512, gravityY: 0f);
            world.AddStaticBox(1, (0f, -100f, 0f), (1f, 1f, 1f));
            world.SetMaterial(0, 0.3f, 0, viscosity);
            var desc = new Box3DSoaParticleDesc[216];
            for (var k = 0; k < 6; k++) for (var j = 0; j < 6; j++) for (var i = 0; i < 6; i++)
                desc[(k * 6 + j) * 6 + i] = new Box3DSoaParticleDesc((2 * R * i, 2 * R * j, 2 * R * k), FluidMass, R, FluidPhase, 0, null, (0.5f * (j - 2.5f), 0f, 0f));
            var made = new Box3DSoaParticleHandle[216];
            world.CreateParticles(desc, made);
            for (var tick = 0; tick < 30; tick++) world.Step(1f / 60f, 4);
            var v = world.View;
            double mean = 0, sq = 0;
            foreach (var h in made) { mean += v.ParticleVx[h.Lane]; sq += Sq(v.ParticleVx[h.Lane]); }
            mean /= made.Length;
            return sq / made.Length - mean * mean;
        }
        var free = Variance(0f);
        var damped = Variance(0.1f);
        var line = $"SHEAR velocity variance after 0.5 s: viscosity 0 {free:F5}, viscosity 0.1 {damped:F5} (ratio {damped / free:F4})";
        Readings.Add(line);
        Assert.True(damped <= ShearBound * free, line);
    }

    // ---- determinism ----------------------------------------------------------------------------------------

    private static ulong Fnv(ulong h, ReadOnlySpan<byte> bytes)
    {
        foreach (var b in bytes) { h ^= b; h *= 1099511628211ul; }
        return h;
    }

    private static ulong HashFluid(Box3DSoaView v)
    {
        var h = 1469598103934665603ul;
        void F(ReadOnlySpan<float> lane) => h = Fnv(h, MemoryMarshal.AsBytes(lane));
        F(v.ParticleX); F(v.ParticleY); F(v.ParticleZ); F(v.ParticleVx); F(v.ParticleVy); F(v.ParticleVz); F(v.ParticleDensity);
        F(v.DistanceTension); F(v.AttachmentForceX); F(v.AttachmentForceY); F(v.AttachmentForceZ);
        F(v.PositionX); F(v.PositionY); F(v.PositionZ); F(v.VelocityX); F(v.VelocityY); F(v.VelocityZ);
        F(v.ParticleImpulseX); F(v.ParticleImpulseY); F(v.ParticleImpulseZ);
        return h;
    }

    private static ulong[] Checkpoints(bool nudge = false)
    {
        var (world, _, _, _) = Pour(nudge);
        using (world)
        {
            var hashes = new ulong[6];
            for (var c = 0; c < 6; c++)
            {
                for (var t = 0; t < 40; t++) world.Step(1f / 60f, Sub);
                hashes[c] = HashFluid(world.View);
            }
            return hashes;
        }
    }

    // Re-pinned once at the kernel-cell cut (fluid rows in sorted order over 4 rfmax cells, fluid-solid contact in the fluid pass: every pair sum has a new order). Recorded on this commit's build (gcc 13.2, -ffp-contract=off): the pour scene's (4 substeps, fluid block) particle, density and body lanes at ticks 40, 80, ... 240.
    private static readonly ulong[] FluidHashes = [5735058435646750614ul, 17157245750823511088ul, 9310017083272243290ul, 13504503053832530307ul, 15203508087677495529ul, 12413386253327045810ul];

    [Fact]
    public void Fluid_world_is_byte_deterministic()
    {
        var a = Checkpoints();
        var b = Checkpoints();
        Assert.True(a.AsSpan().SequenceEqual(b), "two runs diverged: " + string.Join(",", a) + " vs " + string.Join(",", b));
        Assert.True(a.Distinct().Count() == 6, "the scene moves");
        Assert.False(a.AsSpan().SequenceEqual(Checkpoints(nudge: true)), "a one-ulp input difference must diverge");
        Assert.True(FluidHashes.AsSpan().SequenceEqual(a), string.Join(",", a.Select(g => g + "ul")));
    }

    [Fact]
    public async Task Fluid_world_is_the_same_bytes_on_any_thread_under_load()
    {
        var serial = Checkpoints();
        var results = Enumerable.Range(0, 8).Select(_ => Task.Factory.StartNew(() => Checkpoints(), TaskCreationOptions.LongRunning)).ToArray();
        await Task.WhenAll(results);
        foreach (var r in results) Assert.True(Enumerable.SequenceEqual(serial, await r), "a concurrent run diverged from the serial one");
    }
}
