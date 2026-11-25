using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Liquid sensors read settled liquid too (lodur:ruling:liquid-sensor-reads-pooled-liquid): the wet patches whose centre is inside the sphere, on any body or solid particle, in a lane of their own.</summary>
public sealed partial class Box3DSoaFluidTests
{
    /// <summary>The volume the pooled lane must read, by a scan of every wet lane (what the sensor's culled lookup must equal): the patches' volumes summed in lane order over the live patches whose centre, taken to the world, is within <paramref name="radius"/> of <paramref name="centre"/>; with the closest any centre comes to the surface.</summary>
    private static (float Volume, int Count, double Margin) PooledInside(Box3DSoaView v, Pos centre, float radius)
    {
        var volume = 0f;
        var count = 0;
        var margin = double.MaxValue;
        for (var w = 0; w < v.WetCount; w++)
        {
            if (v.WetLive[w] == 0) continue;
            Pos at;
            if (v.WetBody[w] >= 0)
            {
                var b = v.WetBody[w];
                var (qx, qy, qz, qw) = (v.RotationX[b], v.RotationY[b], v.RotationZ[b], v.RotationW[b]);
                var (px, py, pz) = (v.WetX[w], v.WetY[w], v.WetZ[w]);
                var (tx, ty, tz) = (qy * pz - qz * py + qw * px, qz * px - qx * pz + qw * py, qx * py - qy * px + qw * pz);
                at = (v.PositionX[b] + px + 2f * (qy * tz - qz * ty), v.PositionY[b] + py + 2f * (qz * tx - qx * tz), v.PositionZ[b] + pz + 2f * (qx * ty - qy * tx));
            }
            else
            {
                var i = v.WetParticle[w];
                at = (v.ParticleX[i], v.ParticleY[i], v.ParticleZ[i]);
            }
            double dx = at.X - centre.X, dy = at.Y - centre.Y, dz = at.Z - centre.Z;
            var d = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            margin = Math.Min(margin, Math.Abs(d - radius));
            if (d <= radius)
            {
                volume += v.WetPatchVolume[w];
                count++;
            }
        }
        return (volume, count, margin);
    }

    /// <summary>A floor (body lane 0, frame origin (0, -0.5, 0)) with one drop on it at each of <paramref name="at"/> (x, z), rested 20 ticks so each reads its support.</summary>
    private static (Box3DSoaWorld World, Box3DSoaParticleHandle[] Drops) DropsOnFloor(Pos[] at, int wets, int sensors)
    {
        var world = FluidWorld(at.Length + 8, 8, gravityY: -9.81f, wets: wets, sensors: sensors);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (2f, 0.5f, 2f), friction: 0.3f);
        var desc = at.Select(p => new Box3DSoaParticleDesc((p.X, R, p.Z), FluidMass, R, FluidPhase, 0u, null, default)).ToArray();
        world.CreateParticles(desc);
        Run(world, 20);
        return (world, Enumerable.Range(0, at.Length).Select(i => Handle(world, i)).ToArray());
    }

    [Fact]
    public void Settled_liquid_in_a_sphere_reads_in_the_pooled_lane_and_0_live()
    {
        var (world, drops) = DropsOnFloor([(-0.4f, 0f, 0f), (-0.2f, 0f, 0f), (0.2f, 0f, 0f), (0.4f, 0f, 0f)], 8, 4);
        using var _ = world;
        var anchor = world.AddAnchor(9, (0.2f, 0.3f, 0f));
        var h = new Box3DSoaConstraintHandle[3];
        // floor frame: body-frame y 0.5 is the floor's top. A holds the drops at x -0.4 and -0.2; B (r 5 cm) none; C is on the anchor, so the patch at x 0.2 is on another body.
        world.CreateSensors([(0, (-0.3f, 0.5f, 0f), 0.15f), (0, (0.3f, 0.5f, 0f), 0.05f), (anchor, (0f, 0f, 0f), 0.31f)], h);
        world.Step(Dt, Sub);
        var live = world.View.SensorLiquidVolume.ToArray();
        Assert.True(live[h[0]] >= 2f * DropVolume && world.View.SensorWetVolume.ToArray().All(x => x == 0f), "unsettled drops read live, none pooled");
        world.Settle(drops);
        Assert.Equal(live[h[0]], world.View.SensorLiquidVolume[h[0]]); // end-of-step values: a Settle between steps shows at the next step
        Assert.Equal(0f, world.View.SensorWetVolume[h[0]]);
        world.Step(Dt, Sub);
        var v = world.View;
        var a = PooledInside(v, (-0.3f, 0f, 0f), 0.15f);
        var b = PooledInside(v, (0.3f, 0f, 0f), 0.05f);
        var c = PooledInside(v, (0.2f, 0.3f, 0f), 0.31f);
        Assert.True(Math.Min(a.Margin, Math.Min(b.Margin, c.Margin)) > 1e-4, $"a patch centre within 0.1 mm of a surface: {a.Margin} {b.Margin} {c.Margin}");
        Assert.Equal(2, a.Count);
        Assert.Equal(0, b.Count);
        Assert.Equal(1, c.Count);
        Assert.Equal(2f * DropVolume, a.Volume);
        Assert.Equal(a.Volume, v.SensorWetVolume[h[0]]);
        Assert.Equal(0f, v.SensorWetVolume[h[1]]); // a patch outside reads 0
        Assert.Equal(c.Volume, v.SensorWetVolume[h[2]]); // a patch on another body counts
        for (var s = 0; s < 3; s++) Assert.Equal(0f, v.SensorLiquidVolume[h[s]]);
        Readings.Add($"SENSOR pooled: A {v.SensorWetVolume[h[0]]:E4} m^3 ({a.Count} patches), B {v.SensorWetVolume[h[1]]}, C (anchor, other body) {v.SensorWetVolume[h[2]]:E4}, live 0 after the step");
        // The lane is a reading, not a record: removing a patch shows at the next step too.
        world.Step(Dt, Sub);
        Assert.Equal(a.Volume, world.View.SensorWetVolume[h[0]]);
    }

    [Fact]
    public void Pooled_lane_equals_a_scan_of_every_patch_on_a_dense_floor()
    {
        // 64 patches on a 0.15 m grid (a patch disc is 7 cm, so none merge), every one the same volume, so the sum is the same in any order; sensors cut the grid at assorted places.
        var at = new List<Pos>();
        for (var i = 0; i < 8; i++) for (var j = 0; j < 8; j++) at.Add((-0.525f + 0.15f * i, 0f, -0.525f + 0.15f * j));
        var (world, drops) = DropsOnFloor(at.ToArray(), 64, 8);
        using var _ = world;
        var spec = new (Pos Point, float Radius)[]
        {
            ((0.075f, 0.5f, 0.075f), 0.12f), ((0.0f, 0.5f, 0.0f), 0.08f), ((-0.3f, 0.5f, 0.2f), 0.33f), ((0.45f, 0.5f, -0.45f), 0.2f),
            ((0.0f, 0.5f, 0.0f), 1.5f), ((0.0f, 0.9f, 0.0f), 0.3f), ((3f, 0.5f, 0f), 0.5f), ((-0.12f, 0.5f, 0.3f), 0.17f),
        };
        var h = new Box3DSoaConstraintHandle[spec.Length];
        world.CreateSensors(spec.Select(x => (0, x.Point, x.Radius)).ToArray(), h);
        world.Settle(drops);
        world.Step(Dt, Sub);
        var v = world.View;
        Assert.Equal(64, v.WetLive.ToArray().Count(l => l != 0));
        var seen = new List<int>();
        for (var s = 0; s < spec.Length; s++)
        {
            var want = PooledInside(v, (spec[s].Point.X, spec[s].Point.Y - 0.5f, spec[s].Point.Z), spec[s].Radius);
            Assert.True(want.Margin > 1e-4, $"sensor {s}: a patch centre within 0.1 mm of the surface ({want.Margin})");
            Assert.Equal(want.Volume, v.SensorWetVolume[h[s]]);
            seen.Add(want.Count);
        }
        Assert.Contains(seen, n => n > 0 && n < 64);
        Assert.Contains(seen, n => n == 64);
        Assert.Contains(seen, n => n == 0);
        Readings.Add($"SENSOR pooled dense: patches inside per sensor {string.Join(",", seen)} of 64, each equal to a scan of every lane");
    }

    [Fact]
    public void Settled_liquid_on_a_solid_particle_reads_pooled()
    {
        using var world = FluidWorld(64, 8, gravityY: -9.81f, wets: 4, sensors: 2);
        world.AddStaticBox(1, (0f, -50f, 0f), (1f, 1f, 1f));
        var plate = Graph.Block(world, (-0.05f, 0.2f, -0.05f), 5, 1, 5, R, 0f, Box3DSoaPhase.Of(4));
        var drop = Graph.Block(world, (0.0f, 0.2f + 2f * R + 0.001f, 0.0f), 1, 1, 1, R, FluidMass, FluidPhase);
        var h = new Box3DSoaConstraintHandle[2];
        world.CreateSensors([(0, (0f, 50.2f, 0f), 0.3f), (0, (0f, 52f, 0f), 0.3f)], h);
        Run(world, 20);
        Assert.True(world.View.SupportParticle[drop[0].Lane] >= 0, "the drop rests on the plate");
        world.Settle(drop);
        world.Step(Dt, Sub);
        var v = world.View;
        var want = PooledInside(v, (0f, 0.2f, 0f), 0.3f);
        Assert.Equal(1, want.Count);
        Assert.Equal(DropVolume, v.SensorWetVolume[h[0]]);
        Assert.Equal(0f, v.SensorWetVolume[h[1]]);
        Assert.Equal(0f, v.SensorLiquidVolume[h[0]]);
    }
}
