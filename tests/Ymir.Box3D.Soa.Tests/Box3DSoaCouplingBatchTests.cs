using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Attach as a batch ending in one derivation, and the ONE_WAY flag (docs/soa-world.md, attachment lanes).</summary>
public sealed partial class Box3DSoaCouplingTests
{
    private static ulong FnvOf(ulong h, ReadOnlySpan<byte> bytes)
    {
        foreach (var b in bytes) { h ^= b; h *= 1099511628211ul; }
        return h;
    }

    /// <summary>Every attachment lane, the tether count and the particle position and velocity lanes, as one hash.</summary>
    private static ulong StateHash(Box3DSoaView v)
    {
        var h = 1469598103934665603ul;
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentKind));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentFlags));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentGeneration));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentParticle));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentBody));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentPointX));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentPointY));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentPointZ));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.AttachmentCompliance));
        h = FnvOf(h, BitConverter.GetBytes(v.AttachmentCount));
        h = FnvOf(h, BitConverter.GetBytes(v.TetherCount));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.ParticleX)); h = FnvOf(h, MemoryMarshal.AsBytes(v.ParticleY)); h = FnvOf(h, MemoryMarshal.AsBytes(v.ParticleZ));
        h = FnvOf(h, MemoryMarshal.AsBytes(v.ParticleVx)); h = FnvOf(h, MemoryMarshal.AsBytes(v.ParticleVy)); h = FnvOf(h, MemoryMarshal.AsBytes(v.ParticleVz));
        return h;
    }

    private static Box3DSoaConstraintHandle AttachOne(Box3DSoaWorld world, in Box3DSoaAttachmentDesc d) =>
        world.Attach((Box3DSoaAttachKind)d.Kind, d.Particle, d.Body, (d.PointX, d.PointY, d.PointZ), d.Compliance,
            tether: (d.Flags & 1u) != 0, limb: (d.Flags & 2u) != 0, oneWay: (d.Flags & 4u) != 0);

    /// <summary>
    /// A 16 x 16 sheet (256 particles, distances and bends) and four capsules; attachment k ties particle k to the capsule of its quadrant at the
    /// particle's own place. Every 7th is ONE_WAY, every 11th LIMB, so the batch carries each flag.
    /// </summary>
    private static (Box3DSoaWorld World, Box3DSoaAttachmentDesc[] Descs) SheetOnCapsules()
    {
        var world = Box3DSoaParticleTests.World(bodies: 8, particles: 264, distances: 600, bends: 600, attachments: 256, tethers: 8, gravityY: -G);
        Pos origin = (-0.225f, 0.5f, -0.225f);
        var sheet = Graph.Sheet(world, origin, (Spacing, 0f, 0f), (0f, 0f, Spacing), 16, 16, ParticleMass, ParticleRadius);
        var bodies = new Pos[] { (-0.11f, 0.3f, -0.11f), (0.11f, 0.3f, -0.11f), (-0.11f, 0.3f, 0.11f), (0.11f, 0.3f, 0.11f) };
        var lanes = new int[4];
        for (var q = 0; q < 4; q++) lanes[q] = world.AddCapsule((ulong)(q + 1), bodies[q], 0.05f, 0.1f, 1000f);
        var descs = new Box3DSoaAttachmentDesc[256];
        for (var k = 0; k < 256; k++)
        {
            int i = k % 16, j = k / 16, q = (i / 8) + 2 * (j / 8);
            Pos at = (origin.X + Spacing * i, origin.Y, origin.Z + Spacing * j);
            descs[k] = new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, sheet.Particles[k], lanes[q], (at.X - bodies[q].X, at.Y - bodies[q].Y, at.Z - bodies[q].Z),
                compliance: 1e-5f, limb: k % 11 == 0, oneWay: k % 7 == 0);
        }
        return (world, descs);
    }

    [Fact]
    public void Attach_batch_equals_the_same_attaches_one_by_one()
    {
        var (singles, descs) = SheetOnCapsules();
        var (batch, _) = SheetOnCapsules();
        var one = new Box3DSoaConstraintHandle[256];
        var many = new Box3DSoaConstraintHandle[256];
        using (singles)
        using (batch)
        {
            for (var k = 0; k < 256; k++) one[k] = AttachOne(singles, descs[k]);
            batch.Attach(descs, many);
            Assert.Equal(one, many);
            Assert.Equal(256, batch.View.AttachmentCount);
            Assert.Equal(StateHash(singles.View), StateHash(batch.View));
            Box3DSoaParticleTests.Run(singles, 120);
            Box3DSoaParticleTests.Run(batch, 120);
            Assert.Equal(StateHash(singles.View), StateHash(batch.View));
            var line = $"ATTACH_BATCH 256 attachments, 120 steps: lanes and generations equal, state hash {StateHash(batch.View):x16}";
            Console.Error.WriteLine(line);
            Reading(line);
        }
    }

    [Fact]
    public void Attach_batch_reuses_lanes_lowest_first_as_singles_would()
    {
        var (singles, descs) = SheetOnCapsules();
        var (batch, _) = SheetOnCapsules();
        using (singles)
        using (batch)
        {
            var first = new Box3DSoaConstraintHandle[256];
            batch.Attach(descs, first);
            foreach (var d in descs) AttachOne(singles, d);
            // remove every third lane in both worlds, step, then attach 40 again: the same lowest free lanes with new generations
            var gone = first.Where((_, k) => k % 3 == 0).ToArray();
            singles.Remove(gone);
            batch.Remove(gone);
            Box3DSoaParticleTests.Run(singles, 1);
            Box3DSoaParticleTests.Run(batch, 1);
            var again = descs.Where((_, k) => k % 3 == 0).Take(40).ToArray();
            var viaSingles = again.Select(d => AttachOne(singles, d)).ToArray();
            var viaBatch = new Box3DSoaConstraintHandle[40];
            batch.Attach(again, viaBatch);
            Assert.Equal(viaSingles, viaBatch);
            Assert.True(viaBatch.Zip(gone, (n, o) => n.Lane == o.Lane && n.Generation == o.Generation + 1).Take(40).All(x => x));
            Assert.Equal(StateHash(singles.View), StateHash(batch.View));
        }
    }

    private static string Snapshot(Box3DSoaWorld world)
    {
        var v = world.View;
        return $"{v.AttachmentCount}|{v.TetherCount}|{StateHash(v):x16}|{string.Join(",", v.AttachmentKind.ToArray())}|{string.Join(",", v.AttachmentGeneration.ToArray())}";
    }

    // A batch is atomic and N single Attach calls are not: on a refusal the batch leaves the world as it was, while a loop of singles keeps the
    // attachments accepted before the refused one. So "a batch equals N singles" holds for an accepted batch (the tests above), and a refused batch
    // is pinned on its own below. The status can also differ when a bad desc and too few free lanes meet: the batch validates first (INVALID_ARGUMENT),
    // singles reach FULL at the overflow. Callers that tolerated a late refusal in a loop lose the whole batch.
    private static void AssertRefusedWhole(Box3DSoaWorld world, Box3DSoaAttachmentDesc[] batch, string status, string what)
    {
        var before = Snapshot(world);
        var e = Assert.Throws<InvalidOperationException>(() => world.Attach(batch));
        Assert.Contains($"status {status}", e.Message);
        Assert.True(before == Snapshot(world), $"{what}: the refusal changed a lane");
    }

    [Fact]
    public void A_batch_with_one_bad_desc_changes_nothing()
    {
        using var world = Box3DSoaParticleTests.World(bodies: 4, particles: 64, distances: 64, bends: 8, attachments: 16, tethers: 600, gravityY: 0f, fluid: 4);
        var ps = new Box3DSoaParticleHandle[16];
        world.CreateParticles(Graph.Line((0f, 1f, 0f), (Spacing, 0f, 0f), 16).Select(q => new Box3DSoaParticleDesc(q, ParticleMass, ParticleRadius)).ToArray(), ps);
        world.CreateDistances(Enumerable.Range(0, 15).Select(i => new Box3DSoaDistanceDesc(ps[i], ps[i + 1])).ToArray());
        var fluid = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0f, 3f, 0f), ParticleMass, ParticleRadius, Box3DSoaPhase.Of(2, fluid: true))], fluid);
        var live = world.AddCapsule(1, (0f, 0f, 0f), 0.05f, 0.1f, 1000f);
        var dead = world.AddCapsule(2, (1f, 0f, 0f), 0.05f, 0.1f, 1000f);
        world.RemoveBody(dead);
        world.Attach(Box3DSoaAttachKind.Static, ps[0], point: (0f, 1f, 0f), tether: true); // a held tether lane to keep unchanged
        world.Attach(Box3DSoaAttachKind.Body, ps[15], live, (0f, 0f, 0f));
        Assert.True(world.View.TetherCount > 0);

        Box3DSoaAttachmentDesc[] Batch(Box3DSoaAttachmentDesc bad)
        {
            var b = new Box3DSoaAttachmentDesc[8];
            for (var k = 0; k < 8; k++) b[k] = new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[1 + k], live, (0.01f * k, 0.05f, 0f), tether: k % 2 == 0);
            b[5] = bad;
            return b;
        }
        AssertRefusedWhole(world, Batch(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, fluid[0], live)), "1", "a fluid particle");
        AssertRefusedWhole(world, Batch(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[9], dead.Lane)), "1", "a removed body");
        AssertRefusedWhole(world, Batch(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Static, ps[9], point: (0f, 1f, 0f), oneWay: true)), "1", "STATIC with ONE_WAY");
        AssertRefusedWhole(world, Batch(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Static, ps[9], point: (0f, 1f, 0f), limb: true)), "1", "STATIC with LIMB");
        AssertRefusedWhole(world, Batch(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[9], live, compliance: -1f)), "1", "a negative compliance");
        // a batch that fits is taken whole afterwards
        world.Attach(Batch(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[9], live, (0f, 0.05f, 0f))));
        Assert.Equal(10, world.View.AttachmentCount);
    }

    [Fact]
    public void A_batch_past_the_free_lanes_or_the_tether_capacity_changes_nothing()
    {
        using var world = Box3DSoaParticleTests.World(bodies: 4, particles: 32, distances: 32, bends: 8, attachments: 8, tethers: 5, gravityY: 0f);
        var ps = new Box3DSoaParticleHandle[10];
        world.CreateParticles(Graph.Line((0f, 1f, 0f), (Spacing, 0f, 0f), 10).Select(q => new Box3DSoaParticleDesc(q, ParticleMass, ParticleRadius)).ToArray(), ps);
        world.CreateDistances(Enumerable.Range(0, 9).Select(i => new Box3DSoaDistanceDesc(ps[i], ps[i + 1])).ToArray()); // no tether attachment yet: no tethers
        var body = world.AddCapsule(1, (0f, 0f, 0f), 0.05f, 0.1f, 1000f);
        world.Attach(Box3DSoaAttachKind.Body, ps[9], body, (0f, 0f, 0f));
        var spare = new Box3DSoaConstraintHandle[2]; // two tombstones below the count, which the tether batch below would reuse
        world.Attach([new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[7], body), new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[8], body)], spare);
        world.Remove(spare);
        Box3DSoaParticleTests.Run(world, 1);
        var free = new Box3DSoaAttachmentDesc[8]; // one lane is taken: 8 do not fit in 7
        for (var k = 0; k < 8; k++) free[k] = new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[k], body);
        AssertRefusedWhole(world, free, "3", "more descs than free lanes");
        // two tether attachments over the 10-rope need more tethers than the capacity of 5: the derivation refuses and every lane of the batch backs out
        var tether = new[]
        {
            new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Static, ps[0], point: (0f, 1f, 0f), tether: true),
            new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Static, ps[8], point: (8 * Spacing, 1f, 0f), tether: true),
        };
        var before = Snapshot(world);
        Assert.Throws<InvalidOperationException>(() => world.Attach(tether));
        Assert.True(before == Snapshot(world), "a derivation that does not fit left a lane of the batch");
        Assert.Equal(3, world.View.AttachmentCount);
        Assert.Equal(0, world.View.TetherCount);
        Assert.Equal(2u, world.View.AttachmentKind[1]); // the tombstones read as they did
        Assert.Equal(2u, world.View.AttachmentKind[2]);
    }

    private static (Box3DSoaWorld World, int Box, Box3DSoaParticleHandle Particle) OneWayScene(bool oneWay)
    {
        var world = Box3DSoaParticleTests.World(bodies: 4, particles: 8, distances: 1, bends: 1, attachments: 4, tethers: 1, gravityY: 0f);
        var box = world.AddBox(1, (0f, 0f, 0f), (0.1f, 0.1f, 0.1f), 125f); // 1 kg
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0.2f, 0f, 0f), 0.5f, ParticleRadius, velocity: (0f, 1f, 0f))], p);
        world.Attach(Box3DSoaAttachKind.Body, p[0], box, (0.2f, 0f, 0f), oneWay: oneWay);
        return (world, box, p[0]);
    }

    /// <summary>The box's top speed and largest impulse lane magnitude, and the particle's worst distance from its point, over 60 ticks.</summary>
    private static (float Speed, float Impulse, float Deviation) OneWayRun(bool oneWay)
    {
        var (world, box, particle) = OneWayScene(oneWay);
        using (world)
        {
            var speed = 0f; var impulse = 0f; var deviation = 0f;
            for (var i = 0; i < 60; i++)
            {
                world.Step(Dt, Sub);
                var v = world.View;
                speed = MathF.Max(speed, MathF.Sqrt(Sq(v.VelocityX[box]) + Sq(v.VelocityY[box]) + Sq(v.VelocityZ[box])));
                impulse = MathF.Max(impulse, MathF.Sqrt(Sq(v.ParticleImpulseX[box]) + Sq(v.ParticleImpulseY[box]) + Sq(v.ParticleImpulseZ[box])));
                // the point is the box's local (0.2, 0, 0); a one-way box does not move or turn, and the plain run is not judged on this
                deviation = MathF.Max(deviation, MathF.Sqrt(Sq(v.ParticleX[particle.Lane] - (v.PositionX[box] + 0.2f)) + Sq(v.ParticleY[particle.Lane] - v.PositionY[box]) + Sq(v.ParticleZ[particle.Lane] - v.PositionZ[box])));
            }
            return (speed, impulse, deviation);
        }
    }

    [Fact]
    public void A_one_way_attachment_holds_its_particle_and_leaves_its_body_alone()
    {
        var plain = OneWayRun(false);
        var one = OneWayRun(true);
        var line = $"ONE_WAY box speed / impulse lane / particle deviation: plain {F(plain.Speed)} / {F(plain.Impulse)} / {F(plain.Deviation)}; one-way {F(one.Speed)} / {F(one.Impulse)} / {F(one.Deviation)}";
        Console.Error.WriteLine(line);
        Reading(line);
        Assert.True(plain.Impulse > 0f && plain.Speed > 0.05f, "without the flag the particle did not move the box: " + line);
        Assert.Equal(0f, one.Speed);
        Assert.Equal(0f, one.Impulse);
        Assert.True(one.Deviation < OneWayDeviation, "the one-way particle left its point: " + line);
    }

    /// <summary>
    /// A 10-particle rope sliding at 3 m/s, its first particle tethered to box <c>a</c>, with the tether solved on its own (the particle-tether branch)
    /// or against a plain second tether on box <c>b</c> (the pair-tether branch). Returns the worst of box <c>a</c>'s impulse lane magnitude plus its
    /// speed over 60 ticks: what the rope books on the box.
    /// </summary>
    private static float TetherLoad(bool oneWay, bool pair)
    {
        using var world = Box3DSoaParticleTests.World(bodies: 4, particles: 32, distances: 32, bends: 8, attachments: 8, tethers: 600, gravityY: 0f);
        var ps = new Box3DSoaParticleHandle[10];
        world.CreateParticles(Graph.Line((0f, 0f, 0f), (Spacing, 0f, 0f), 10).Select(q => new Box3DSoaParticleDesc(q, ParticleMass, ParticleRadius, velocity: (3f, 0f, 0f))).ToArray(), ps);
        world.CreateDistances(Enumerable.Range(0, 9).Select(i => new Box3DSoaDistanceDesc(ps[i], ps[i + 1])).ToArray());
        var a = world.AddBox(1, (-0.3f, 0f, 0f), (0.1f, 0.1f, 0.1f), 125f);
        var b = world.AddBox(2, (9 * Spacing + 0.3f, 0f, 0f), (0.1f, 0.1f, 0.1f), 125f);
        world.Attach([new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[0], a, (0.3f, 0f, 0f), tether: true, oneWay: oneWay)]);
        if (pair) world.Attach([new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, ps[9], b, (-0.3f, 0f, 0f), tether: true)]);
        Assert.True(world.View.TetherCount > 0);
        var worst = 0f; var v = world.View;
        for (var i = 0; i < 60; i++)
        {
            world.Step(Dt, Sub);
            worst = MathF.Max(worst, MathF.Sqrt(Sq(v.ParticleImpulseX[a]) + Sq(v.ParticleImpulseY[a]) + Sq(v.ParticleImpulseZ[a])) + MathF.Abs(v.VelocityX[a]));
        }
        return worst;
    }

    [Fact]
    public void A_one_way_tether_books_nothing_on_its_body_in_either_tether_branch()
    {
        // end_w and end_push carry the ONE_WAY guard for solve_attachments and for both tether branches. Pinned only through solve_attachments, the
        // guard could move out of them and the tethers would load the box again.
        var plain = TetherLoad(false, false); var one = TetherLoad(true, false);
        var plainPair = TetherLoad(false, true); var onePair = TetherLoad(true, true);
        var line = $"ONE_WAY_TETHER box load, particle-tether: plain {F(plain)} one-way {F(one)}; pair-tether: plain {F(plainPair)} one-way {F(onePair)}";
        Console.Error.WriteLine(line);
        Reading(line);
        Assert.True(plain > OneWayTetherLoad && plainPair > OneWayTetherLoad, "without the flag the tether did not load the box: " + line);
        Assert.True(one < OneWayTetherLoad, "a one-way particle tether loaded its body: " + line);
        Assert.True(onePair < OneWayTetherLoad, "a one-way pair tether loaded its body: " + line);
    }

    [Fact]
    public void A_one_way_attachment_is_refused_on_a_world_point()
    {
        using var world = Box3DSoaParticleTests.World(gravityY: 0f);
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0f, 1f, 0f), ParticleMass, ParticleRadius)], p);
        Assert.Throws<InvalidOperationException>(() => world.Attach(Box3DSoaAttachKind.Static, p[0], point: (0f, 1f, 0f), oneWay: true));
        Assert.Equal(0, world.View.AttachmentCount);
    }

    [Fact]
    public void A_flag_bit_outside_the_known_mask_is_refused()
    {
        // The world is valid in every other respect (a live body, a live particle, a legal point and compliance), so only the unknown bit can refuse it.
        // The managed single Attach cannot send a bit, the descriptor can. Bit 8 is the next bit after ONE_WAY (lodur:ruling:ymir-new-flag-value-no-abi-bump).
        using var world = Box3DSoaParticleTests.World(bodies: 2, particles: 4, distances: 1, bends: 1, attachments: 4, tethers: 4, gravityY: 0f);
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0f, 1f, 0f), ParticleMass, ParticleRadius)], p);
        var body = world.AddCapsule(1, (0f, 0f, 0f), 0.05f, 0.1f, 1000f);
        // the same desc with a known flag is taken, so the refusal below is the bit and nothing else
        var valid = new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, p[0], body, (0f, 1f, 0f), oneWay: true);
        var taken = new Box3DSoaConstraintHandle[1];
        world.Attach([valid], taken);
        Assert.Equal(1, world.View.AttachmentCount);
        world.Remove(taken);
        Box3DSoaParticleTests.Run(world, 1);
        var unknown = valid;
        unknown.Flags = 8u;
        AssertRefusedWhole(world, [unknown], "1", "an unknown flag bit"); // status INVALID_ARGUMENT, no lane changed
    }

    /// <summary>One Fella's flesh in the probe's counts: 661 particles (a 22 x 30 sheet and one more), a distance per sheet link, every particle attached to one of 19 bones.</summary>
    private static double Spawn(int fellas, bool batch)
    {
        const int per = 661, sheet = 660;
        using var world = Box3DSoaParticleTests.World(bodies: 19 * fellas + 1, particles: per * fellas + 8, distances: 1300 * fellas, bends: 8,
            attachments: per * fellas, tethers: 8, gravityY: 0f);
        var bones = new int[19 * fellas];
        for (var b = 0; b < bones.Length; b++) bones[b] = world.AddCapsule((ulong)(b + 1), (0.12f * (b % 19), 0f, 0.5f * (b / 19)), 0.05f, 0.1f, 100f);
        var all = new List<Box3DSoaAttachmentDesc>();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        for (var f = 0; f < fellas; f++)
        {
            var desc = new Box3DSoaParticleDesc[per];
            for (var k = 0; k < per; k++)
                desc[k] = new Box3DSoaParticleDesc((0.04f * (k % 22), 0.04f * (k / 22), 0.5f * f), ParticleMass, ParticleRadius, Box3DSoaPhase.Of((uint)(f + 1), noParticleContact: true));
            var p = new Box3DSoaParticleHandle[per];
            world.CreateParticles(desc, p);
            var d = new List<Box3DSoaDistanceDesc>();
            for (var k = 0; k < sheet; k++)
            {
                if (k % 22 != 21 && k + 1 < sheet) d.Add(new(p[k], p[k + 1]));
                if (k + 22 < sheet) d.Add(new(p[k], p[k + 22]));
            }
            world.CreateDistances(d.ToArray());
            for (var k = 0; k < per; k++)
            {
                var bone = bones[19 * f + k % 19];
                all.Add(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, p[k], bone, (desc[k].X - 0.12f * (k % 19), desc[k].Y, desc[k].Z - 0.5f * f), compliance: 1e-4f, limb: true, oneWay: k % 2 == 0));
            }
        }
        if (batch) world.Attach(all.ToArray());
        else foreach (var a in all) AttachOne(world, a);
        return watch.Elapsed.TotalMilliseconds;
    }

    [Fact]
    public void Attach_spawn_cost_line()
    {
        // The 256-attachment sheet as 256 single attaches and as one batch, alternately 5 times; then Fella-scale spawns (661 attachments each, one and
        // four) the same way, 3 times, the whole spawn timed. Printed, no bound: a batch ending in one derivation against one per attachment.
        static double Median(double[] x) { Array.Sort(x); return x[x.Length / 2]; }
        var single = new double[5]; var batch = new double[5];
        for (var r = 0; r < 5; r++)
        {
            foreach (var asBatch in r % 2 == 0 ? new[] { false, true } : new[] { true, false })
            {
                var (world, descs) = SheetOnCapsules();
                using (world)
                {
                    var watch = System.Diagnostics.Stopwatch.StartNew();
                    if (asBatch) world.Attach(descs);
                    else foreach (var d in descs) AttachOne(world, d);
                    (asBatch ? batch : single)[r] = watch.Elapsed.TotalMilliseconds;
                }
            }
        }
        var line = $"ATTACH_SPAWN 256 attachments: singles median {F(Median(single))} ms, one batch {F(Median(batch))} ms";
        Console.Error.WriteLine(line);
        Reading(line);
        foreach (var fellas in new[] { 1, 4 })
        {
            var s = new double[3]; var b = new double[3];
            for (var r = 0; r < 3; r++)
            {
                s[r] = Spawn(fellas, false);
                b[r] = Spawn(fellas, true);
            }
            var fella = $"FELLA_SPAWN {fellas} Fella(s), {661 * fellas} attachments, whole spawn: singles median {F(Median(s))} ms, batched attach median {F(Median(b))} ms";
            Console.Error.WriteLine(fella);
            Reading(fella);
        }
        Assert.True(Median(batch) > 0);
    }

    // Bounds by lodur:ruling:deterministic-bound-margin and deterministic-bound-zero-reading (the unit is 1 um; a shipped zero counts as half a unit).
    // The mutants, each a one-line change to ymir_particles.c or ymir_box3d_soa.c, and what separates them:
    //   Mo  end_push books on a ONE_WAY body        the box's speed reads 400 m/s: Assert.Equal(0f, speed) and the impulse lane 310
    //   Mw  end_w weighs a ONE_WAY body             particle deviation below
    //   Mb  the batch's back-out skips the lanes    A_batch_past_the_free_lanes_or_the_tether_capacity fails (a reused tombstone reads live)
    //   Mv  only the first desc validated           A_batch_with_one_bad_desc fails;  Mq  ONE_WAY accepted on STATIC: both refusal tests fail
    //   Ms  the removed-body check on the first desc only: A_batch_with_one_bad_desc fails
    //   Mc  the cursor not advanced: reads identical (lane k is live once written, so the scan skips it): a cost-only equivalent mutant, unclaimed.
    //   Mr  a rederive per desc: cost only (the cost line), unclaimed.
    // OneWayDeviation: shipped worst 0 (-> 5e-7), least claimed mutant Mw 2.17828e-3, separation 4357; bound sqrt(5e-7 x 2.17828e-3).
    private const float OneWayDeviation = 3.3e-5f;
    // OneWayTetherLoad: shipped worst 0 (-> 5e-7), least claimed mutant Mtether 0.77 (pair branch; 0.93 particle branch), separation 1.5e6; bound sqrt(5e-7 x 0.77).
    //   Mtether  the ONE_WAY guard moved out of end_w/end_push into solve_attachments alone: only A_one_way_tether_books_nothing_* fails.
    //   Mmask    the unknown-bit clause deleted: only A_flag_bit_outside_the_known_mask_is_refused fails.
    private const float OneWayTetherLoad = 6e-4f;
}
