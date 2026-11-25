using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Settled liquid as data: docs/soa-world.md, Settle. The step writes no wet lane; settle, remove and destroy do.</summary>
public sealed partial class Box3DSoaFluidTests
{
    private const float DropVolume = (2f * R) * (2f * R) * (2f * R); // (2 r)^3, in float as the native computes it

    private static Box3DSoaParticleHandle Handle(Box3DSoaWorld world, int lane) => new(lane, world.View.ParticleGeneration[lane]);

    /// <summary>A floor (body lane 0, top at y 0) and a world with room for patches; the drops rest on it for 20 ticks so each reads its support.</summary>
    private static Box3DSoaWorld WetFloor(int particles, int wets, Pos[] at, uint[]? material = null, float[]? radius = null, int materials = 2)
    {
        var world = FluidWorld(particles, gravityY: -9.81f, materials: materials, wets: wets);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
        var desc = new Box3DSoaParticleDesc[at.Length];
        for (var i = 0; i < at.Length; i++)
        {
            var r = radius?[i] ?? R;
            desc[i] = new Box3DSoaParticleDesc((at[i].X, r, at[i].Z), FluidMass * (r / R) * (r / R) * (r / R), r, FluidPhase, material?[i] ?? 0u, null, default);
        }
        world.CreateParticles(desc);
        for (var tick = 0; tick < 20; tick++) world.Step(1f / 60f, Sub);
        return world;
    }

    private static Box3DSoaParticleHandle[] Handles(Box3DSoaWorld world, int count) =>
        Enumerable.Range(0, count).Select(i => Handle(world, i)).ToArray();

    private static byte[] WetBytes(Box3DSoaView v)
    {
        var o = new MemoryStream();
        void B<T>(ReadOnlySpan<T> lane) where T : unmanaged => o.Write(MemoryMarshal.AsBytes(lane));
        B(v.WetBody); B(v.WetParticle); B(v.WetX); B(v.WetY); B(v.WetZ); B(v.WetRadius); B(v.WetPatchVolume); B(v.WetMaterial);
        B(v.WetLive); B(v.WetGeneration); B(v.WetRemovedAt); B(v.WetVolume);
        return o.ToArray();
    }

    private static byte[] WorldBytes(Box3DSoaView v)
    {
        var o = new MemoryStream();
        void B<T>(ReadOnlySpan<T> lane) where T : unmanaged => o.Write(MemoryMarshal.AsBytes(lane));
        o.Write(WetBytes(v));
        B(v.ParticleLive); B(v.ParticleGeneration); B(v.ParticleX); B(v.ParticleY); B(v.ParticleZ); B(v.ParticleVx); B(v.ParticleVy); B(v.ParticleVz);
        B(v.ParticleRemovedAt); B(v.SupportBody); B(v.SupportParticle);
        B(v.TetherAttachment); B(v.TetherParticle); B(v.TetherParent); B(v.TetherEdge); B(v.TetherOther); B(v.TetherBound); B(v.ParticleComponent);
        return o.ToArray();
    }

    private static int LivePatches(Box3DSoaView v) => v.WetLive.ToArray().Count(l => l != 0);

    // Relative error of the patch volumes' sum against n (2 r)^3, over three settle orders of the same 100 drops (ascending, descending, interleaved): shipped
    // worst 1.909e-7 (a float sum, so the bound takes the worst across orders by lodur:ruling:ymir-column-mean-bound-order-noise), M1 (a merge forgets the volume
    // already in the patch) 0.84 in each order: bound sqrt(1.909e-7 x 0.84) = 4.0e-4, separation 4.4e6. M2 (no merge) and M2b (merge across materials) conserve the sum and are
    // claimed by Settle_merges_inside_a_patch.
    private const double ConserveBound = 4.0e-4;

    [Fact]
    public void Settle_conserves_volume()
    {
        var worst = 0.0;
        foreach (var order in new[] { "ascending", "descending", "interleaved" })
        {
            var at = new Pos[100];
            for (var k = 0; k < 10; k++) for (var i = 0; i < 10; i++) at[k * 10 + i] = (-0.14f + 2.5f * R * i, 0f, -0.14f + 2.5f * R * k);
            using var world = WetFloor(128, 16, at);
            var view = world.View;
            Assert.All(Enumerable.Range(0, 100), i => Assert.True(view.SupportBody[i] >= 0, $"drop {i} reads no support"));
            var tethers = Snapshot(view);
            var drops = Handles(world, 100);
            if (order == "descending") Array.Reverse(drops);
            if (order == "interleaved") drops = drops.Where((_, i) => i % 2 == 0).Concat(drops.Where((_, i) => i % 2 == 1).Reverse()).ToArray();
            world.Settle(drops);
            view = world.View;
            SameTethers(tethers, Snapshot(view));
            Assert.All(Enumerable.Range(0, 100), i => Assert.Equal(0u, view.ParticleLive[i]));
            var sum = 0.0;
            for (var w = 0; w < view.WetCount; w++) if (view.WetLive[w] != 0) sum += view.WetPatchVolume[w];
            var error = Math.Abs(sum - 100.0 * DropVolume) / (100.0 * DropVolume);
            Readings.Add($"WET conserve {order}: {LivePatches(view)} patches, relative error {error:E3}");
            worst = Math.Max(worst, error);
        }
        Assert.True(worst <= ConserveBound, $"relative error {worst:E3}");
    }

    // Centre of a merged patch against the volume-weighted mean of its two contact points, in metres: shipped worst 5.96e-8, M3 (an unweighted mean) 2.333e-2:
    // bound sqrt(5.96e-8 x 2.333e-2) = 3.7e-5 m, separation 3.9e5.
    private const double CentreBound = 3.7e-5;

    [Fact]
    public void Settle_merges_inside_a_patch()
    {
        // A drop's patch is a disc of radius sqrt((2 r)^3 / (pi film)) = 70.5 mm. Two drops 30 mm apart: one patch.
        using (var world = WetFloor(16, 4, [(-0.015f, 0f, 0f), (0.015f, 0f, 0f)]))
        {
            world.Settle(Handles(world, 2));
            Assert.Equal(1, LivePatches(world.View));
            Assert.Equal(2f * DropVolume, world.View.WetPatchVolume[0], 1e-9f);
        }
        // Two spots 200 mm apart: two.
        using (var world = WetFloor(16, 4, [(-0.1f, 0f, 0f), (0.1f, 0f, 0f)]))
        {
            world.Settle(Handles(world, 2));
            Assert.Equal(2, LivePatches(world.View));
        }
        // The same spot, two materials: two (liquids of different material do not mix).
        using (var world = WetFloor(16, 4, [(-0.015f, 0f, 0f), (0.015f, 0f, 0f)], material: [0u, 1u]))
        {
            world.Settle(Handles(world, 2));
            var view = world.View;
            Assert.Equal(2, LivePatches(view));
            Assert.Equal(0u, view.WetMaterial[0]);
            Assert.Equal(1u, view.WetMaterial[1]);
        }
        // Unequal drops (volume 1 : 8) merge to the volume-weighted centre, in the body's frame (the floor body is at y -0.5: the contact point is 0.5 above its origin).
        using (var world = WetFloor(16, 4, [(-0.03f, 0f, 0f), (0.03f, 0f, 0f)], radius: [R, 2f * R]))
        {
            var view = world.View;
            var (x0, x1) = ((double)view.ParticleX[0], (double)view.ParticleX[1]);
            var (v0, v1) = ((double)DropVolume, 8.0 * DropVolume);
            var (z0, z1) = ((double)view.ParticleZ[0], (double)view.ParticleZ[1]);
            world.Settle(Handles(world, 2));
            view = world.View;
            Assert.Equal(1, LivePatches(view));
            var cx = (x0 * v0 + x1 * v1) / (v0 + v1);
            var cz = (z0 * v0 + z1 * v1) / (v0 + v1);
            var error = Math.Max(Math.Abs(view.WetX[0] - cx), Math.Max(Math.Abs(view.WetZ[0] - cz), Math.Abs(view.WetY[0] - 0.5)));
            Readings.Add($"WET centre error {error:E3} m, weighted x {cx:F5}, unweighted {(x0 + x1) / 2:F5}");
            Assert.Equal(0, view.WetBody[0]);
            Assert.Equal(-1, view.WetParticle[0]);
            Assert.True(error <= CentreBound, $"centre error {error:E3} m");
        }
    }

    [Fact]
    public void Settle_refusals_change_nothing()
    {
        // One patch lane. A pedestal (body 1) beside the floor (body 0); a drop on each, a second on the floor, a solid particle, a drop in flight.
        using var world = FluidWorld(16, gravityY: -9.81f, wets: 1);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
        world.AddStaticBox(2, (0.5f, 0.1f, 0f), (0.1f, 0.1f, 0.1f), friction: 0.3f);
        var made = new Box3DSoaParticleHandle[4];
        world.CreateParticles([Fluid((-0.5f, R, 0f)), Fluid((-0.45f, R, 0f)), Fluid((0.5f, 0.2f + R, 0f)), Fluid((0.9f, 30f, 0f))], made);
        var solid = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0.2f, 0.5f, 0f), 0.02f, R, 0u, 0, null, default)], solid);
        for (var tick = 0; tick < 20; tick++) world.Step(1f / 60f, Sub);
        var (floorA, floorB, pedestal, flight) = (made[0], made[1], made[2], made[3]);
        Assert.Equal(0, world.View.SupportBody[floorA.Lane]);
        Assert.Equal(1, world.View.SupportBody[pedestal.Lane]);
        world.Settle([floorA]); // the only lane is taken, on the floor
        Assert.Equal(1, LivePatches(world.View));
        var gone = new Box3DSoaParticleHandle[1];
        world.CreateParticles([Fluid((0.9f, 30f, 0.5f))], gone);
        world.DestroyParticles(gone);
        var before = WorldBytes(world.View);
        void Refused(params Box3DSoaParticleHandle[] batch)
        {
            Assert.Throws<InvalidOperationException>(() => world.Settle(batch));
            Assert.True(before.AsSpan().SequenceEqual(WorldBytes(world.View)), "a refused batch changed a lane");
        }
        Refused(floorB, solid[0]);           // a solid particle
        Refused(floorB, flight);             // no support
        Refused(floorB, gone[0]);            // a stale handle
        Refused(floorB, floorB);             // named twice
        Refused(floorB, pedestal);           // the floor drop would merge, the pedestal drop finds no lane and no patch on its support: FULL, all undone
        world.Settle([floorB]);              // and without the pedestal drop it merges
        Assert.Equal(1, LivePatches(world.View));
        Assert.Equal(2f * DropVolume, world.View.WetPatchVolume[0], 1e-9f);
    }

    [Fact]
    public void Patches_follow_their_body()
    {
        // A patch on a free box keeps its point in the box's frame while the box is shoved along the floor.
        {
            using var world = FluidWorld(64, gravityY: -9.81f, wets: 4);
            world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
            var box = world.AddBox(2, (0f, 0.1f, 0f), (0.1f, 0.1f, 0.1f), 5000f, friction: 0.3f);
            var drop = new Box3DSoaParticleHandle[1];
            world.CreateParticles([Fluid((0.03f, 0.2f + R, 0.02f))], drop);
            for (var tick = 0; tick < 20; tick++) world.Step(1f / 60f, Sub);
            Assert.Equal(box, world.View.SupportBody[drop[0].Lane]);
            world.Settle(drop);
            var view = world.View;
            var (x, y, z) = (view.WetX[0], view.WetY[0], view.WetZ[0]);
            Assert.Equal(box, view.WetBody[0]);
            var rest = view.PositionX[box];
            world.ApplyImpulse(box, (1500f, 0f, 0f), (view.PositionX[box], view.PositionY[box], view.PositionZ[box]));
            for (var tick = 0; tick < 30; tick++) world.Step(1f / 60f, Sub);
            view = world.View;
            Assert.True(view.PositionX[box] - rest > 0.1f, "the box moved");
            Assert.Equal((x, y, z), (view.WetX[0], view.WetY[0], view.WetZ[0]));
            Assert.Equal(box, view.WetBody[0]);
        }
        // A patch on a solid particle goes when that particle is destroyed; a patch on a body does not.
        {
            using var world = new Box3DSoaWorld(4, gravityY: -9.81f, particles: new Box3DSoaParticleCapacities(256, 160, 140, 16, 1024, 2, 16, 4));
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
            var view = world.View;
            Assert.Equal(1, LivePatches(view));
            Assert.Equal(under, view.WetParticle[0]);
            Assert.Equal(-1, view.WetBody[0]);
            Assert.Equal((0f, 0f, 0f), (view.WetX[0], view.WetY[0], view.WetZ[0]));
            world.DestroyParticles([sheet.Particles.Single(h => h.Lane == under)]);
            view = world.View;
            Assert.Equal(0, LivePatches(view));
            Assert.Equal(0f, view.WetPatchVolume[0]);
            Assert.Equal(-1, view.WetParticle[0]);
            Assert.Equal(1u, view.WetGeneration[0]);
        }
    }

    [Fact]
    public void Wet_volume_reads_patches()
    {
        using var world = WetFloor(16, 4, [(-0.1f, 0f, 0f), (0.1f, 0f, 0f), (0.11f, 0f, 0.01f)]);
        void Reads()
        {
            var v = world.View;
            var sum = new float[v.Count];
            for (var w = 0; w < v.WetCount; w++) if (v.WetLive[w] != 0) sum[v.WetBody[w]] += v.WetPatchVolume[w];
            Assert.True(sum.AsSpan().SequenceEqual(v.WetVolume), "wet volume is not the patch sum");
        }
        Assert.Equal(0f, world.View.WetVolume[0]);
        var patches = new Box3DSoaConstraintHandle[3];
        world.Settle(Handles(world, 3), patches);
        Reads();
        Assert.Equal(3f * DropVolume, world.View.WetVolume[0], 1e-9f);
        Assert.Equal(patches[1].Lane, patches[2].Lane);
        var before = WetBytes(world.View);
        for (var tick = 0; tick < 30; tick++) world.Step(1f / 60f, Sub); // a step writes no wet lane and no wet volume
        Assert.True(before.AsSpan().SequenceEqual(WetBytes(world.View)), "a step changed a wet lane");
        Assert.Equal(2, world.FreeWetSlots); // two of four lanes taken
        world.Remove(patches[0]);
        Reads();
        Assert.Equal(2f * DropVolume, world.View.WetVolume[0], 1e-9f);
        Assert.Equal(2, world.FreeWetSlots); // a freed lane waits for a step to pass
        world.Step(1f / 60f, Sub);
        Assert.Equal(3, world.FreeWetSlots);
        Assert.Throws<InvalidOperationException>(() => world.Remove(patches[0])); // a stale handle
    }

    [Fact]
    public void Settle_joins_the_lowest_patch_that_holds_the_point()
    {
        // Patches at x -0.05 and 0.05 (100 mm apart, radius 70.5 mm: separate); a drop at 0 lies in both discs and joins the lower lane.
        using var world = WetFloor(16, 4, [(-0.05f, 0f, 0f), (0.05f, 0f, 0f), (0f, 0f, 0f)]);
        var v = world.View;
        Assert.True(v.ParticleX[1] - v.ParticleX[0] > 0.0705f && MathF.Abs(v.ParticleX[2] - v.ParticleX[0]) < 0.07f && MathF.Abs(v.ParticleX[2] - v.ParticleX[1]) < 0.07f, "the scene did not hold its geometry");
        world.Settle([Handle(world, 0)]);
        world.Settle([Handle(world, 1)]);
        Assert.Equal(2, LivePatches(world.View));
        var landed = new Box3DSoaConstraintHandle[1];
        world.Settle([Handle(world, 2)], landed);
        v = world.View;
        Assert.Equal(0, landed[0].Lane);
        Assert.Equal(2f * DropVolume, v.WetPatchVolume[0], 1e-9f);
        Assert.Equal(DropVolume, v.WetPatchVolume[1], 1e-9f);
    }

    [Fact]
    public void Settle_with_no_lane_joins_the_nearest_patch_of_its_material()
    {
        // Three lanes, all taken: lane 0 material 0 at x -0.12, lane 1 material 0 at -0.4, lane 2 material 1 at 0.09. A material 0 drop at 0 lies in no disc and no lane
        // is free: it joins lane 0, the nearest of its own material (lane 2 is nearer and lane 1 farther, and neither may take it).
        using var world = WetFloor(16, 3, [(-0.12f, 0f, 0f), (-0.4f, 0f, 0f), (0.09f, 0f, 0f), (0f, 0f, 0f)], material: [0u, 0u, 1u, 0u]);
        var v = world.View;
        Assert.True(v.ParticleX[3] - v.ParticleX[0] > 0.0705f && v.ParticleX[2] - v.ParticleX[3] > 0.0705f && v.ParticleX[3] - v.ParticleX[0] > v.ParticleX[2] - v.ParticleX[3], "the scene did not hold its geometry");
        world.Settle([Handle(world, 0), Handle(world, 1), Handle(world, 2)]);
        Assert.Equal(0, world.FreeWetSlots);
        var landed = new Box3DSoaConstraintHandle[1];
        world.Settle([Handle(world, 3)], landed);
        v = world.View;
        Assert.Equal(0, landed[0].Lane);
        Assert.Equal(3, LivePatches(v));
        Assert.Equal(2f * DropVolume, v.WetPatchVolume[0], 1e-9f);
        Assert.Equal(DropVolume, v.WetPatchVolume[1], 1e-9f);
        Assert.Equal(DropVolume, v.WetPatchVolume[2], 1e-9f);
    }

    [Fact]
    public void Settle_refusal_unwinds_a_patch_it_created()
    {
        // One lane; the floor drop creates the patch, then the pedestal drop finds no lane and no patch on its support: FULL. The created patch and the count go back.
        using var world = FluidWorld(16, gravityY: -9.81f, wets: 1);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
        world.AddStaticBox(2, (0.5f, 0.1f, 0f), (0.1f, 0.1f, 0.1f), friction: 0.3f);
        world.CreateParticles([Fluid((-0.5f, R, 0f)), Fluid((0.5f, 0.2f + R, 0f)), Fluid((-0.45f, R, 0f))]);
        for (var tick = 0; tick < 20; tick++) world.Step(1f / 60f, Sub);
        Assert.Equal(0, world.View.WetCount);
        var before = WorldBytes(world.View);
        Assert.Throws<InvalidOperationException>(() => world.Settle([Handle(world, 0), Handle(world, 1)]));
        Assert.Equal(0, world.View.WetCount);
        Assert.Equal(0, LivePatches(world.View));
        Assert.True(before.AsSpan().SequenceEqual(WorldBytes(world.View)), "a refused batch changed a lane");
        var landed = new Box3DSoaConstraintHandle[1];
        world.Settle([Handle(world, 0)], landed); // the lane is still free and its generation unmoved
        Assert.Equal(0u, world.View.WetGeneration[0]);
        // A lane below the count, dead and a step old, is taken again by the batch and must read dead again after the refusal.
        world.Remove(landed[0]);
        world.Step(1f / 60f, Sub);
        before = WorldBytes(world.View);
        Assert.Equal(1, world.View.WetCount);
        Assert.Equal(0, LivePatches(world.View));
        Assert.Throws<InvalidOperationException>(() => world.Settle([Handle(world, 2), Handle(world, 1)]));
        Assert.Equal(0, LivePatches(world.View));
        Assert.True(before.AsSpan().SequenceEqual(WorldBytes(world.View)), "a refused batch changed a lane");
    }

    private static string NativeSource(string file)
    {
        for (var dir = new DirectoryInfo(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            var path = Path.Combine(dir.FullName, "native", "Ymir.Box3D", file);
            if (File.Exists(path)) return File.ReadAllText(path);
        }
        throw new FileNotFoundException(file);
    }

    [Fact]
    public void Remove_wet_never_reaches_the_topology_derivation()
    {
        // Rederiving an unchanged graph writes the same bytes, so no lane tells the call from its absence; the rule is pinned on the source (the authority map: remove(WET)
        // ends in rederive_wet, never in rederive_topology; rederive_wet's callers are settle, remove(WET), release_body and destroy).
        var source = NativeSource("ymir_particles.c").Replace("\r\n", "\n");
        var at = source.IndexOf("if (type == YMIR_LANES_WET)", StringComparison.Ordinal);
        Assert.True(at > 0);
        var branch = source.Substring(at, source.IndexOf("return YMIR_PARTICLES_OK;", at, StringComparison.Ordinal) - at);
        branch = System.Text.RegularExpressions.Regex.Replace(branch, "//[^\n]*", ""); // the comments name what the branch avoids
        Assert.Contains("rederive_wet(p);", branch);
        Assert.DoesNotContain("rederive_topology", branch);
        Assert.DoesNotContain("TOPO_", branch);
        Assert.Equal(4, System.Text.RegularExpressions.Regex.Matches(source, @"[^_]rederive_wet\(p\);").Count); // settle, remove(WET), release_body (a removed body takes its patches), and destroy when a destroyed solid takes its patches (particle_wet_mu goes dry again)
    }

    // ---- determinism ----------------------------------------------------------------------------------------

    private static ulong HashWet(Box3DSoaView v)
    {
        var h = 1469598103934665603ul;
        h = Fnv(h, WorldBytes(v));
        void F(ReadOnlySpan<float> lane) => h = Fnv(h, MemoryMarshal.AsBytes(lane));
        F(v.PositionX); F(v.PositionY); F(v.PositionZ);
        return h;
    }

    private static ulong[] WetCheckpoints(bool nudge = false)
    {
        // A pool poured into a well that holds a free box; every 10 ticks every drop that reads a support settles.
        using var world = FluidWorld(256, gravityY: -9.81f, wets: 32);
        Well(world, 0.1f);
        world.AddBox(6, (0f, 0.04f, 0f), (0.04f, 0.04f, 0.04f), 2000f, friction: 0.3f);
        var desc = new Box3DSoaParticleDesc[100];
        for (var k = 0; k < 5; k++) for (var j = 0; j < 4; j++) for (var i = 0; i < 5; i++)
            desc[(k * 4 + j) * 5 + i] = new Box3DSoaParticleDesc((-0.05f + 0.03f * i, 0.12f + 0.03f * j, -0.05f + 0.03f * k), FluidMass, R, FluidPhase, 0, null, default);
        if (nudge) desc[40].X = BitConverter.UInt32BitsToSingle(BitConverter.SingleToUInt32Bits(desc[40].X) + 1);
        world.CreateParticles(desc);
        var hashes = new ulong[6];
        for (var c = 0; c < 6; c++)
        {
            for (var t = 0; t < 40; t++)
            {
                world.Step(1f / 60f, Sub);
                if (t % 10 != 9) continue;
                var v = world.View;
                var supported = new List<Box3DSoaParticleHandle>();
                for (var lane = 0; lane < v.ParticleCount; lane++)
                    if (v.ParticleLive[lane] != 0 && (v.SupportBody[lane] >= 0 || v.SupportParticle[lane] >= 0)) supported.Add(new Box3DSoaParticleHandle(lane, v.ParticleGeneration[lane]));
                world.Settle(supported.ToArray());
            }
            hashes[c] = HashWet(world.View);
        }
        Assert.True(LivePatches(world.View) > 0, "nothing settled");
        return hashes;
    }

    // Recorded on this commit's build (gcc 13.2, -ffp-contract=off): the wet pour at ticks 40, 80, ... 240. Re-pinned when the Box3D fork ran the friction hook on recycled contacts
    // (lodur:ruling:wet-friction-on-recycled-contacts): this world settles liquid under a resting box, whose friction now follows it; the fork at 2a43fae reproduces the old values.
    private static readonly ulong[] WetHashes = [6720483314439625191ul, 15807674163676588943ul, 8969239209631959286ul, 3159003609627402407ul, 16856749248747581653ul, 3168406661706779043ul];

    [Fact]
    public void Wet_world_is_byte_deterministic()
    {
        var a = WetCheckpoints();
        var b = WetCheckpoints();
        Assert.True(a.AsSpan().SequenceEqual(b), "two runs diverged: " + string.Join(",", a) + " vs " + string.Join(",", b));
        Assert.True(a.Distinct().Count() > 1, "the scene moves");
        Assert.False(a.AsSpan().SequenceEqual(WetCheckpoints(nudge: true)), "a one-ulp input difference must diverge");
        Assert.True(WetHashes.AsSpan().SequenceEqual(a), string.Join(",", a.Select(g => g + "ul")));
    }

    [Fact]
    public async Task Wet_world_is_the_same_bytes_on_any_thread_under_load()
    {
        var serial = WetCheckpoints();
        var results = Enumerable.Range(0, 8).Select(_ => Task.Factory.StartNew(() => WetCheckpoints(), TaskCreationOptions.LongRunning)).ToArray();
        await Task.WhenAll(results);
        foreach (var r in results) Assert.True(Enumerable.SequenceEqual(serial, await r), "a concurrent run diverged from the serial one");
    }
}
