using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Liquid sensors: docs/soa-world.md, Sensors. A sphere fixed in a body reads the liquid volume inside it at the end of every step; nothing reads it back.</summary>
public sealed partial class Box3DSoaFluidTests
{
    /// <summary>The volume the sensor must read: (2 r)^3 summed in lane order over the live fluid lanes whose centre is within <paramref name="radius"/> of <paramref name="centre"/>, the count, and the closest any centre comes to the sphere's surface.</summary>
    private static (float Volume, int Count, double Margin) Inside(Box3DSoaView v, Pos centre, float radius)
    {
        var volume = 0f;
        var count = 0;
        var margin = double.MaxValue;
        for (var i = 0; i < v.ParticleCount; i++)
        {
            if (v.ParticleLive[i] == 0 || (v.ParticlePhase[i] & Box3DSoaPhase.Fluid) == 0) continue;
            double dx = v.ParticleX[i] - centre.X, dy = v.ParticleY[i] - centre.Y, dz = v.ParticleZ[i] - centre.Z;
            var d = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            margin = Math.Min(margin, Math.Abs(d - radius));
            if (d <= radius)
            {
                var side = 2f * v.ParticleRadius[i];
                volume += side * side * side;
                count++;
            }
        }
        return (volume, count, margin);
    }

    /// <summary>A well of half-width 0.15 (floor is body lane 0, top at y 0, frame origin at y -0.5) with a 6 x 6 x 6 block of water in it.</summary>
    private static Box3DSoaWorld Pool(int sensors, int bodies = 8)
    {
        var world = FluidWorld(256, bodies, gravityY: -9.81f, sensors: sensors);
        Well(world, 0.15f);
        Graph.Block(world, (-0.0625f, 0.0125f, -0.0625f), 6, 6, 6, R, FluidMass, FluidPhase);
        return world;
    }

    private static void Run(Box3DSoaWorld world, int ticks)
    {
        for (var t = 0; t < ticks; t++) world.Step(Dt, Sub);
    }

    [Fact]
    public void Sensor_reads_the_liquid_inside()
    {
        using var world = Pool(sensors: 4);
        var handles = new Box3DSoaConstraintHandle[3];
        // The floor's frame origin is (0, -0.5, 0): a body-frame y of 0.5 + h is world height h over the floor.
        world.CreateSensors([(0, (0f, 0.54f, 0f), 0.06f), (0, (0f, 1.5f, 0f), 0.1f), (0, (0f, 0.55f, 0f), 1f)], handles);
        Run(world, 90);
        var v = world.View;
        var part = Inside(v, (0f, 0.04f, 0f), 0.06f);
        var above = Inside(v, (0f, 1f, 0f), 0.1f);
        var all = Inside(v, (0f, 0.05f, 0f), 1f);
        Assert.True(part.Count > 0 && part.Count < all.Count, $"the scene must split the pool: {part.Count} of {all.Count}");
        Assert.True(part.Margin > 1e-4 && all.Margin > 1e-4, $"a centre within 0.1 mm of a sensor's surface: {part.Margin} {all.Margin}");
        Assert.Equal(216, all.Count);
        Assert.Equal(part.Volume, v.SensorLiquidVolume[handles[0]]);
        Assert.Equal(0f, above.Volume);
        Assert.Equal(0f, v.SensorLiquidVolume[handles[1]]);
        Assert.Equal(all.Volume, v.SensorLiquidVolume[handles[2]]);
        Assert.Equal(3, v.SensorCount);
        Readings.Add($"SENSOR pool 216 drops: partial sensor {part.Count} drops = {part.Volume:E4} m^3, all {all.Count} = {all.Volume:E4} m^3, margin {part.Margin:E2} m");
    }

    [Fact]
    public void Sensor_on_a_dunked_and_lifted_body_reads_up_then_0()
    {
        using var world = Pool(sensors: 2);
        var anchor = world.AddAnchor(9, (0f, 0.6f, 0f));
        var handle = new Box3DSoaConstraintHandle[1];
        world.CreateSensors([(anchor, (0.02f, 0f, 0f), 0.05f)], handle);
        Run(world, 60);
        var v = world.View;
        Assert.Equal(0f, v.SensorLiquidVolume[handle[0]]);
        var peak = 0f;
        var reads = new List<float>();
        for (var t = 0; t < 40; t++)
        {
            var y = t < 20 ? 0.6f - 0.03f * (t + 1) : 0.03f * (t - 19); // down to y 0, then back up to 0.6
            world.SetAnchorTarget(anchor, (0f, y, 0f), default, Dt);
            world.Step(Dt, Sub);
            v = world.View;
            var want = Inside(v, (v.PositionX[anchor] + 0.02f, v.PositionY[anchor], v.PositionZ[anchor]), 0.05f);
            Assert.True(want.Margin > 1e-4, $"tick {t}: a centre within 0.1 mm of the surface ({want.Margin})");
            Assert.Equal(want.Volume, v.SensorLiquidVolume[handle[0]]);
            reads.Add(v.SensorLiquidVolume[handle[0]]);
            peak = MathF.Max(peak, reads[^1]);
        }
        Assert.True(peak >= 3f * DropVolume, $"dunked: peak {peak}");
        Assert.True(reads.Take(3).All(r => r == 0f), "above the surface it reads 0 on the way down");
        Assert.Equal(0f, reads[^1]);
        Readings.Add($"SENSOR dunk and lift: peak {peak:E4} m^3 ({peak / DropVolume:F1} drops), first {reads[0]}, last {reads[^1]}");
    }

    [Fact]
    public void Sensor_follows_its_body_frame_and_the_pose_the_step_ends_in()
    {
        // Zero gravity, drops 0.5 m apart so none moves. The anchor is turned a quarter about z, so its body-frame point (0.3, 0, 0) lies at world (x, y + 0.3, z) over it,
        // and it runs 0.5 m a tick along x: at the end of tick s it is over the drop at x 0.5 s. Decoys sit where an unrotated frame (or the pose a substep earlier) would look.
        using var world = FluidWorld(32, 4, gravityY: 0f, sensors: 2);
        var descs = new List<Box3DSoaParticleDesc>();
        for (var k = 0; k < 8; k++)
        {
            descs.Add(Fluid((0.5f * k, 0.3f, 0f), FluidMass, R));
            descs.Add(Fluid((0.5f * k + 0.3f, 0f, 0f), FluidMass, R));
        }
        world.CreateParticles(descs.ToArray());
        var q = (0f, 0f, 0.70710678f, 0.70710678f);
        var anchor = world.AddAnchor(9, (0f, 0f, 0f), q);
        var handle = new Box3DSoaConstraintHandle[1];
        world.CreateSensors([(anchor, (0.3f, 0f, 0f), 0.2f)], handle);
        var reads = new float[10];
        for (var s = 1; s <= 9; s++)
        {
            world.SetAnchorTarget(anchor, (0.5f * s, 0f, 0f), q, Dt);
            world.Step(Dt, Sub);
            reads[s] = world.View.SensorLiquidVolume[handle[0]];
        }
        for (var s = 1; s <= 7; s++) Assert.Equal(DropVolume, reads[s]);
        Assert.Equal(0f, reads[8]);
        Assert.Equal(0f, reads[9]);
    }

    [Fact]
    public void Sensor_reads_0_once_every_drop_is_gone()
    {
        // The step's end returns early with no live particle; the sensor pass must run before that, or the sensor keeps its last reading.
        foreach (var how in new[] { "destroyed", "destroyed with a solid left", "settled" })
        {
            using var world = FluidWorld(64, gravityY: -9.81f, wets: 8, sensors: 2);
            world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
            var drops = Graph.Block(world, (0f, 0.0125f, 0f), 3, 1, 3, R, FluidMass, FluidPhase);
            if (how == "destroyed with a solid left") world.CreateParticles([new Box3DSoaParticleDesc((1f, 1f, 1f), 1f, R)]);
            var handle = new Box3DSoaConstraintHandle[1];
            world.CreateSensors([(0, (0f, 0.5f, 0f), 0.5f)], handle);
            Run(world, 20);
            var nine = 0f;
            for (var k = 0; k < 9; k++) nine += DropVolume; // the lane-order float sum the native pass makes
            Assert.Equal(nine, world.View.SensorLiquidVolume[handle[0]]);
            if (how == "settled") world.Settle(drops);
            else world.DestroyParticles(drops);
            world.Step(Dt, Sub);
            Assert.Equal(0f, world.View.SensorLiquidVolume[handle[0]]);
            Assert.Equal(how == "destroyed with a solid left" ? 1 : 0, world.View.ParticleLive.ToArray().Count(l => l != 0));
        }
    }

    private static byte[] SensorBytes(Box3DSoaView v)
    {
        var o = new MemoryStream();
        void B<T>(ReadOnlySpan<T> lane) where T : unmanaged => o.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes(lane));
        B(v.SensorBody); B(v.SensorX); B(v.SensorY); B(v.SensorZ); B(v.SensorRadius); B(v.SensorLiquidVolume); B(v.SensorLive); B(v.SensorGeneration); B(v.SensorRemovedAt);
        o.Write(BitConverter.GetBytes(v.SensorCount));
        return o.ToArray();
    }

    [Fact]
    public void Sensor_refusals_and_reuse()
    {
        using var world = FluidWorld(16, 3, gravityY: 0f, sensors: 3);
        world.AddStaticBox(1, (0f, 0f, 0f), (1f, 1f, 1f));
        world.AddAnchor(2, (0f, 3f, 0f));
        var before = SensorBytes(world.View);
        var bad = new (int Body, Pos Point, float Radius)[]
        {
            (-1, (0f, 0f, 0f), 0.1f), (2, (0f, 0f, 0f), 0.1f), (0, (0f, 0f, 0f), 0f), (0, (0f, 0f, 0f), -0.1f), (0, (0f, 0f, 0f), float.NaN),
            (0, (0f, 0f, 0f), float.PositiveInfinity), (0, (float.NaN, 0f, 0f), 0.1f), (0, (0f, float.PositiveInfinity, 0f), 0.1f),
        };
        foreach (var b in bad)
        {
            Assert.Throws<InvalidOperationException>(() => world.CreateSensors([(0, (0f, 0f, 0f), 0.1f), b]));
            Assert.True(before.AsSpan().SequenceEqual(SensorBytes(world.View)), "a refused batch changed a lane");
            Assert.Equal(3, world.FreeSensorSlots);
        }
        Assert.Throws<InvalidOperationException>(() => world.CreateSensors([(0, default, 0.1f), (0, default, 0.1f), (0, default, 0.1f), (0, default, 0.1f)]));
        Assert.True(before.AsSpan().SequenceEqual(SensorBytes(world.View)), "a batch past the capacity changed a lane");
        var h = new Box3DSoaConstraintHandle[3];
        world.CreateSensors([(0, (0.1f, 0.2f, 0.3f), 0.4f), (1, (0f, 0f, 0f), 0.5f), (0, (1f, 1f, 1f), 0.6f)], h);
        Assert.Equal([0, 1, 2], h.Select(x => x.Lane));
        Assert.All(h, x => Assert.Equal(Box3DSoaConstraintType.Sensor, x.Type));
        var view = world.View;
        Assert.Equal(0, world.FreeSensorSlots);
        Assert.Equal([0, 1, 0], view.SensorBody.ToArray());
        Assert.Equal(0.2f, view.SensorY[0]);
        Assert.Equal(0.5f, view.SensorRadius[1]);
        // Remove: a tombstone reading 0, its lane back after a step, the lowest first, the old handle refused.
        world.Remove([h[0], h[2]]);
        view = world.View;
        Assert.Equal([0u, 1u, 0u], view.SensorLive.ToArray());
        Assert.Equal([-1, 1, -1], view.SensorBody.ToArray());
        Assert.Equal(0f, view.SensorRadius[0]);
        Assert.Equal(0, world.FreeSensorSlots);
        Assert.Throws<InvalidOperationException>(() => world.Remove(h[0]));
        Assert.Throws<InvalidOperationException>(() => world.CreateSensors([(0, default, 0.1f)]));
        world.Step(Dt, Sub);
        Assert.Equal(2, world.FreeSensorSlots);
        var again = new Box3DSoaConstraintHandle[2];
        world.CreateSensors([(1, (0f, 0f, 0f), 0.7f), (0, (0f, 0f, 0f), 0.8f)], again);
        Assert.Equal([0, 2], again.Select(x => x.Lane));
        Assert.Equal([1u, 1u], again.Select(x => x.Generation));
        Assert.Throws<InvalidOperationException>(() => world.Remove(h[0])); // the old occupant's handle
    }

    [Fact]
    public void Sensors_move_nothing()
    {
        // The same pool with and without four sensors steps to the same bytes in every other lane.
        static byte[] Bytes(bool sensors)
        {
            using var world = Pool(sensors ? 4 : 0);
            if (sensors) world.CreateSensors([(0, (0f, 0.54f, 0f), 0.06f), (0, (0.06f, 0.55f, 0f), 0.07f), (0, (0f, 0.6f, 0f), 0.2f), (0, (0f, 1.5f, 0f), 0.1f)]);
            Run(world, 120);
            var v = world.View;
            var o = new MemoryStream();
            void B<T>(ReadOnlySpan<T> lane) where T : unmanaged => o.Write(System.Runtime.InteropServices.MemoryMarshal.AsBytes(lane));
            B(v.ParticleX); B(v.ParticleY); B(v.ParticleZ); B(v.ParticleVx); B(v.ParticleVy); B(v.ParticleVz); B(v.ParticleDensity); B(v.SupportBody); B(v.SupportParticle);
            B(v.PositionX); B(v.PositionY); B(v.PositionZ); B(v.ParticleImpulseX); B(v.ParticleImpulseY); B(v.ParticleImpulseZ); B(v.ParticleLive); B(v.ParticleGeneration);
            return o.ToArray();
        }
        Assert.True(Bytes(true).AsSpan().SequenceEqual(Bytes(false)), "a sensor moved something");
    }

    [Fact]
    public void Remove_sensor_never_reaches_the_topology_derivation_and_Ymir_names_no_organ()
    {
        var source = NativeSource("ymir_particles.c").Replace("\r\n", "\n");
        var at = source.IndexOf("if (type == YMIR_LANES_SENSOR)", StringComparison.Ordinal);
        Assert.True(at > 0);
        var branch = source.Substring(at, source.IndexOf("return YMIR_PARTICLES_OK;", at, StringComparison.Ordinal) - at);
        branch = System.Text.RegularExpressions.Regex.Replace(branch, "//[^\n]*", "");
        Assert.DoesNotContain("rederive_", branch);
        Assert.DoesNotContain("TOPO_", branch);
        // Ymir exposes a volume; what a head or a lung does with it is the caller's.
        var organ = new System.Text.RegularExpressions.Regex("airway|lung|throat", System.Text.RegularExpressions.RegexOptions.IgnoreCase);
        for (var d = new DirectoryInfo(AppContext.BaseDirectory); d is not null; d = d.Parent)
        {
            var native = Path.Combine(d.FullName, "native", "Ymir.Box3D");
            var src = Path.Combine(d.FullName, "src", "Ymir.Box3D");
            if (!Directory.Exists(native) || !Directory.Exists(src)) continue;
            foreach (var file in Directory.EnumerateFiles(native, "*.c").Concat(Directory.EnumerateFiles(native, "*.h")).Concat(Directory.EnumerateFiles(src, "*.cs")))
                Assert.False(organ.IsMatch(File.ReadAllText(file)), file + " names an organ");
            return;
        }
        throw new DirectoryNotFoundException("repo root");
    }
}
