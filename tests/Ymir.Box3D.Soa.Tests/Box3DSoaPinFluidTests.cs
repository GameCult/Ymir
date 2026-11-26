using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Pinned flesh and liquid (docs/soa-world.md, pinned, dormant, kinematic, anchor): settled liquid on dormant flesh rides the flesh's bone, and poured liquid stops at the bone under dormant skin.</summary>
public sealed partial class Box3DSoaFluidTests
{
    /// <summary>
    /// A kinematic bone with a 5 x 5 plate of flesh on it (one-way, limb), one drop settled on the plate, a sensor on the bone around the drop. With <paramref name="pin"/> the plate is
    /// pinned after the settle. Returns the sensor's wet reading after the bone has moved <paramref name="travel"/> m along x, and the reading before.
    /// </summary>
    private static (float Before, float After) WetOnMovingBone(bool pin, float travel)
    {
        using var world = new Box3DSoaWorld(8, gravityY: -10f, particles: new Box3DSoaParticleCapacities(64, 16, 16, 32, 64, 2, 64, 4, 2));
        var bone = world.AddAnchor(1, (0f, 0.3f, 0f));
        var plate = Graph.Block(world, (-0.05f, 0.3f, -0.05f), 5, 1, 5, R, 0.01f, Box3DSoaPhase.Of(4));
        var drop = Graph.Block(world, (0f, 0.3f + 2f * R + 0.001f, 0f), 1, 1, 1, R, FluidMass, FluidPhase);
        for (var i = 0; i < plate.Length; i++)
        {
            var q = world.View;
            world.Attach(Box3DSoaAttachKind.Body, plate[i], bone, (q.ParticleX[plate[i]], 0f, q.ParticleZ[plate[i]]), limb: true, oneWay: true);
        }
        var sensor = new Box3DSoaConstraintHandle[1];
        world.CreateSensors([(bone, (0f, 0f, 0f), 0.3f)], sensor);
        Run(world, 20);
        Assert.True(world.View.SupportParticle[drop[0].Lane] >= 0, "the drop rests on the plate");
        world.Settle(drop);
        if (pin) world.SetPinned(plate, true);
        world.Step(Dt, Sub);
        var before = world.View.SensorWetVolume[sensor[0]];
        var steps = 30;
        for (var s = 1; s <= steps; s++)
        {
            world.SetAnchorTarget(bone, (travel * s / steps, 0.3f, 0f), default, Dt);
            world.Step(Dt, Sub);
        }
        world.Step(Dt, Sub);
        return (before, world.View.SensorWetVolume[sensor[0]]);
    }

    [Fact]
    public void Settled_liquid_on_pinned_flesh_rides_its_bone()
    {
        // The drop settled on flesh that is then pinned: the sensor on the bone reads it after the bone has moved 2 m, as it does for live flesh. A patch read at the
        // dormant particle's stale position would be left behind and read 0.
        var live = WetOnMovingBone(false, 2f);
        var pinned = WetOnMovingBone(true, 2f);
        Readings.Add($"PIN_WET sensor on the bone before and after it moved 2 m: live {live.Before:E4} {live.After:E4}, pinned {pinned.Before:E4} {pinned.After:E4}");
        Assert.Equal(DropVolume, live.Before);
        Assert.Equal(live.Before, live.After);
        Assert.Equal(DropVolume, pinned.Before);
        Assert.Equal(pinned.Before, pinned.After);
    }

    private const float CapR = 0.07f, CapHalf = 0.18f, SkinR = 0.036f;

    /// <summary>Per fluid particle, the closest its centre came to the capsule's axis over a 343-particle pour on a horizontal capsule under a 36-particle skin ring of flesh, live or pinned.</summary>
    private static (double[] Closest, int Inside) PourOnSkin(bool pinned)
    {
        const int Ring = 36;
        using var world = new Box3DSoaWorld(8, gravityY: -10f, jointCapacity: 4,
            particles: new Box3DSoaParticleCapacities(Ring + 343 + 8, 16, 16, Ring + 8, 16, 2, 343));
        var wall = world.AddStaticBox(1, (2f, 0.5f, 0f), (0.05f, 0.05f, 0.05f));
        var sq = MathF.Sqrt(0.5f);
        var bone = world.AddCapsule(2, (0f, 0.5f, 0f), CapR, CapHalf, 500f, rotation: (0f, 0f, sq, sq)); // local Y lies along world x
        world.AddJoint(Box3DSoaJointKind.Weld, wall, bone, (0f, 0.5f, 0f), (0f, 0f, 1f));
        var skin = new Box3DSoaParticleHandle[Ring];
        var local = new Pos[Ring];
        var desc = new Box3DSoaParticleDesc[Ring];
        for (var k = 0; k < Ring; k++)
        {
            var a = 2f * MathF.PI * k / Ring;
            local[k] = (CapR * MathF.Cos(a), 0f, CapR * MathF.Sin(a)); // a ring in the bone's xz plane, around its axis y
        }
        // rotation (0, 0, s, s) is +90 degrees about z: local (x, y, z) -> world (-y, x, z), plus the bone's centre (0, 0.5, 0)
        for (var k = 0; k < Ring; k++) desc[k] = new Box3DSoaParticleDesc((-local[k].Y, local[k].X + 0.5f, local[k].Z), 0.02f, SkinR, Box3DSoaPhase.Of(4));
        world.CreateParticles(desc, skin);
        for (var k = 0; k < Ring; k++) world.Attach(Box3DSoaAttachKind.Body, skin[k], bone, local[k], limb: true, oneWay: true);
        if (pinned) world.SetPinned(skin, true);
        var pour = Graph.Block(world, (-0.15f + 0.0125f, 0.5f + 0.45f, -0.15f + 0.0125f), 7, 7, 7, R, FluidMass, FluidPhase, pitch: 2f * R);
        var closest = Enumerable.Repeat(double.MaxValue, pour.Length).ToArray();
        var inside = 0;
        for (var s = 0; s < 240; s++)
        {
            world.Step(Dt, Sub);
            var v = world.View;
            for (var i = 0; i < pour.Length; i++)
            {
                var l = pour[i].Lane;
                double y = v.ParticleY[l] - 0.5, z = v.ParticleZ[l];
                var d = Math.Sqrt(y * y + z * z);
                if (Math.Abs(v.ParticleX[l]) < CapHalf) closest[i] = Math.Min(closest[i], d);
                if (Math.Abs(v.ParticleX[l]) < CapHalf && d < CapR) inside++;
            }
        }
        return (closest, inside);
    }

    [Fact]
    public void Blood_poured_on_dormant_skin_stops_at_the_bone()
    {
        // Dormant flesh meets no fluid, so liquid on pinned skin stops at the bone's capsule (centre at R + r), not at the skin's surface (R + skin radius + r) where live flesh stops it.
        static double Median(double[] x) { var c = x.Where(d => d != double.MaxValue).OrderBy(d => d).ToArray(); return c[c.Length / 2]; }
        var live = PourOnSkin(false);
        var dormant = PourOnSkin(true);
        double stop = CapR + R;
        var (lm, dm) = (Median(live.Closest), Median(dormant.Closest));
        Readings.Add($"PIN_BLOOD median closest approach to the axis: live skin {lm:F4}, dormant skin {dm:F4}, capsule surface plus fluid radius {stop:F4}; centres inside the capsule live {live.Inside}, dormant {dormant.Inside}");
        Assert.Equal(0, dormant.Inside);
        Assert.True(dm >= stop - 0.002 && dm <= stop + 0.008, $"dormant blood lies {dm} from the axis, the capsule stops it at {stop}");
        Assert.True(lm > dm + 0.008, $"live skin holds the blood off at {lm}, dormant at {dm}");
    }
}
