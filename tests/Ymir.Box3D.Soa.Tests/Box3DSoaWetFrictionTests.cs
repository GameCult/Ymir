using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>Settled liquid lowers friction where it lands: docs/soa-world.md, Wet friction. Body-body contact through the world's friction hook, particle-body through wet_mu.</summary>
public sealed partial class Box3DSoaFluidTests
{
    private const float BoxMass = 5f;
    private const float PushForce = 20f;      // below mu m g = 29.4 N dry, above the wet 0.1 m g = 4.9 N
    private const float WetMu = 0.1f;
    private const float DropRadius = 0.03f;   // a patch of radius sqrt((2 r)^3 / (pi film)) = 0.262 m
    private const int PushTicks = 40;
    private const double WetAcceleration = (PushForce - WetMu * BoxMass * 9.81) / BoxMass;

    /// <summary>A floor of friction 0.6 (body lane 0, top at y 0) with one drop of liquid (wet friction 0.1) settled at <paramref name="patchAt"/> (or destroyed when there is no patch), then 0.2 m boxes of 5 kg on it.</summary>
    private static (Box3DSoaWorld World, int[] Boxes) WetBoxes(Pos[] boxes, bool patch, Pos patchAt)
    {
        var world = FluidWorld(8, wets: 4, gravityY: -9.81f);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (4f, 0.5f, 2f), friction: 0.6f);
        world.SetMaterial(0, 0.6f);
        world.SetMaterial(1, 0.6f, wetFriction: WetMu);
        var scale = DropRadius / R;
        world.CreateParticles([Fluid((patchAt.X, DropRadius, patchAt.Z), FluidMass * scale * scale * scale, DropRadius, material: 1u)]);
        for (var tick = 0; tick < 20; tick++) world.Step(Dt, Sub);
        if (patch) world.Settle([Handle(world, 0)]); else world.DestroyParticles([Handle(world, 0)]);
        var lanes = boxes.Select((at, i) => world.AddBox(10ul + (ulong)i, (at.X, 0.1f, at.Z), (0.1f, 0.1f, 0.1f), BoxMass / 0.008f, friction: 0.6f)).ToArray();
        return (world, lanes);
    }

    /// <summary>Pushes every box along x with a constant force for <paramref name="ticks"/> ticks.</summary>
    private static void Push(Box3DSoaWorld world, int[] boxes, int ticks)
    {
        var view = world.View;
        for (var tick = 0; tick < ticks; tick++)
        {
            foreach (var b in boxes) view.ForceX[b] = PushForce;
            world.Step(Dt, Sub);
        }
    }

    /// <summary>The distance a box moves over <see cref="PushTicks"/> pushed ticks from rest.</summary>
    private static double Moved(Box3DSoaWorld world, int[] boxes, int box)
    {
        var x0 = world.View.PositionX[boxes[box]];
        Push(world, boxes, PushTicks);
        return world.View.PositionX[boxes[box]] - x0;
    }

    [Fact]
    public void Box_slides_on_a_wet_floor()
    {
        double dry, wet;
        {
            var (world, boxes) = WetBoxes([(0f, 0f, 0f)], patch: false, (0f, 0f, 0f));
            using (world) dry = Math.Abs(Moved(world, boxes, 0));
        }
        {
            var (world, boxes) = WetBoxes([(0f, 0f, 0f)], patch: true, (0f, 0f, 0f));
            using (world) wet = 2.0 * Moved(world, boxes, 0) / ((double)PushTicks * Dt * PushTicks * Dt);
        }
        Readings.Add($"WETFRICTION box: dry moved {dry:E3} m; wet acceleration {wet:F4} against {WetAcceleration:F4}, relative error {Math.Abs(wet / WetAcceleration - 1.0):E3}");
        Assert.True(dry < DryBound, $"the dry box moved {dry} m");
        Assert.True(Math.Abs(wet / WetAcceleration - 1.0) <= SlideBound, $"wet acceleration {wet} against {WetAcceleration}");
    }

    // Bounds (lodur:ruling:deterministic-bound-margin, bound = sqrt(shipped worst x least claimed mutant)). Wet acceleration, relative error against (F - mu m g) / m: shipped
    // 6.284e-3; M1 (the patch test in the world frame) and M2 (the hook takes max) leave the box held, 0.9998: bound sqrt(6.284e-3 x 0.9998) = 0.079, separation 12.6. Dry box off
    // the patch, metres moved in 40 ticks: shipped worst 3.123e-5; M4 (no disc: the patch is the whole floor) 0.1688, B slides with A: bound sqrt(3.123e-5 x 0.1688) = 2.3e-3,
    // separation 74. The box under the patch moves 0.1688 m shipped; M1 and M2 read 0.0000, counted one unit (1e-6 m) by lodur:ruling:deterministic-bound-zero-reading:
    // bound sqrt(0.1688 x 1e-6) = 4.1e-4 m, separation 410.
    // The late-settled patch under a resting box reads 6.276e-3 shipped (rest 100 ticks); M5 (the fork does not run the hook on recycled contacts) holds the box, 1.0: the same 0.079.
    // The dry readings across the tests: shipped worst 3.839e-5 m (the patch removed from under a resting box); least claimed mutant M5 (no hook on recycled contacts) 0.1128 m there: bound sqrt(3.839e-5 x 0.1128) = 2.1e-3, tightened from 2.3e-3, separation 54.
    private const double SlideBound = 0.079;
    private const double DryBound = 2.1e-3;
    private const double MovedBound = 4.1e-4;

    [Fact]
    public void Dry_box_beside_a_wet_patch_holds()
    {
        // The patch is under box A; B, 1 m away on the same floor under the same push, stays: friction is lowered where the liquid is, not on the floor.
        var (world, boxes) = WetBoxes([(0f, 0f, 0f), (1f, 0f, 0f)], patch: true, (0f, 0f, 0f));
        using (world)
        {
            var (a0, b0) = (world.View.PositionX[boxes[0]], world.View.PositionX[boxes[1]]);
            Push(world, boxes, PushTicks);
            var (a, b) = (world.View.PositionX[boxes[0]] - a0, Math.Abs(world.View.PositionX[boxes[1]] - b0));
            Readings.Add($"WETFRICTION beside: A moved {a:F4} m, B moved {b:E3} m");
            Assert.True(a > MovedBound, $"A moved {a} m");
            Assert.True(b < DryBound, $"B, off the patch, moved {b} m");
        }
    }

    [Fact]
    public void Patch_elsewhere_changes_nothing()
    {
        // A patch 2 m from the boxes against no patch (the drop destroyed instead of settled): the bodies' bytes are equal.
        static byte[] Bytes((Box3DSoaWorld World, int[] Boxes) w)
        {
            using var world = w.World;
            Push(world, w.Boxes, 60);
            var v = world.View;
            var o = new MemoryStream();
            foreach (var b in w.Boxes)
                foreach (var f in new[] { v.PositionX[b], v.PositionY[b], v.PositionZ[b], v.VelocityX[b], v.VelocityY[b], v.VelocityZ[b] }) o.Write(BitConverter.GetBytes(f));
            return o.ToArray();
        }
        var with = Bytes(WetBoxes([(0f, 0f, 0f), (1f, 0f, 0f)], patch: true, (-2f, 0f, 0f)));
        var without = Bytes(WetBoxes([(0f, 0f, 0f), (1f, 0f, 0f)], patch: false, (-2f, 0f, 0f)));
        Assert.True(with.AsSpan().SequenceEqual(without), "a patch away from every contact moved a body");
    }

    [Fact]
    public void Fluid_keeps_its_own_friction_on_a_wet_floor()
    {
        // A fluid particle sliding over a patch moves as it does over a bare floor (idea ymir-fluid-reval.rejected-fluid-reads-patches).
        static float Slide(bool patch)
        {
            var (world, _) = WetBoxes([], patch, (0f, 0f, 0f));
            using (world)
            {
                world.CreateParticles([Fluid((-0.1f, R, 0f), velocity: (1.5f, 0f, 0f))]);
                for (var tick = 0; tick < 60; tick++) world.Step(Dt, Sub);
                return world.View.ParticleX[1];
            }
        }
        var (with, without) = (Slide(true), Slide(false));
        Readings.Add($"WETFRICTION fluid slide: patch {with:F6}, none {without:F6}");
        Assert.Equal(BitConverter.SingleToInt32Bits(without), BitConverter.SingleToInt32Bits(with));
    }

    // The wet rope's slide in 2 s, metres: shipped 0.535; M1 (the patch test in the world frame) reads 0.000, counted one unit (1e-6 m) by lodur:ruling:deterministic-bound-zero-reading:
    // bound sqrt(0.535 x 1e-6) = 7.3e-4, separation 730. The dry rope's 0.01 holds as in Material_friction_holds_on_a_slope (shipped 0).
    private const float SlopeBound = 7.3e-4f;

    [Fact]
    public void Rope_slides_on_a_wet_slope()
    {
        // Material_friction_holds_on_a_slope's scene (Box3DSoaCouplingTests): the rope holds dry at 0.6 and slides with a patch under it.
        static float Slid(bool patch)
        {
            var tilt = 20f * MathF.PI / 180f;
            using var world = new Box3DSoaWorld(8, gravityX: 10f * MathF.Sin(tilt), gravityY: -10f, jointCapacity: 2,
                particles: new Box3DSoaParticleCapacities(256, 600, 600, 8, 600, 2, 16, 4));
            world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f), friction: 0.6f);
            world.SetMaterial(0, 0.6f);
            world.SetMaterial(1, 0.6f, wetFriction: 0.05f);
            var rope = Graph.Rope(world, Graph.Line((0f, Box3DSoaParticleTests.ParticleRadius, 0f), (0f, 0f, Box3DSoaParticleTests.Spacing), 12),
                Box3DSoaParticleTests.ParticleMass, Box3DSoaParticleTests.ParticleRadius);
            if (patch)
            {
                var scale = 0.05f / R;
                world.CreateParticles([Fluid((-0.3f, 0.05f, 0.15f), FluidMass * scale * scale * scale, 0.05f, material: 1u)]);
                world.Step(Dt, Sub);
                world.Settle([Handle(world, 12)]);
            }
            for (var tick = 0; tick < 2 * 120; tick++) world.Step(Dt, Sub);
            return MathF.Abs(world.View.ParticleX[rope.Particles[6]]);
        }
        var (dry, wet) = (Slid(false), Slid(true));
        Readings.Add($"WETFRICTION slope: dry slid {dry:E3} m, wet slid {wet:F3} m in 2 s");
        Assert.True(dry < 0.01f, $"dry slid {dry} m");
        Assert.True(wet > SlopeBound, $"wet slid only {wet} m");
    }

    [Fact]
    public void A_reused_lane_is_not_slippery()
    {
        // Liquid settled on a solid particle makes that particle slip (particle_wet_mu); when the solid is destroyed the lane goes dry, and a solid created on the reused lane slides
        // as one that never met the liquid (the control destroys the drop instead of settling it).
        static (float X, int Lane, int Reused) Slide(bool patch)
        {
            var world = FluidWorld(16, wets: 4, gravityY: -9.81f);
            using (world)
            {
                world.AddStaticBox(1, (0f, -0.5f, 0f), (4f, 0.5f, 2f), friction: 0.6f);
                world.SetMaterial(0, 0.6f);
                world.SetMaterial(1, 0.6f, wetFriction: 0.05f);
                var solid = Box3DSoaPhase.Of(1);
                world.CreateParticles([new Box3DSoaParticleDesc((0f, R, 0f), 0.02f, R, solid, 0u, null, default)]);
                world.CreateParticles([Fluid((0f, 3f * R, 0f), material: 1u)]);
                for (var tick = 0; tick < 20; tick++) world.Step(Dt, Sub);
                var under = world.View.SupportParticle[1];
                if (patch) { Assert.Equal(0, under); world.Settle([Handle(world, 1)]); } else world.DestroyParticles([Handle(world, 1)]);
                world.DestroyParticles([Handle(world, 0)]);
                world.Step(Dt, Sub);
                var made = new Box3DSoaParticleHandle[1];
                world.CreateParticles([new Box3DSoaParticleDesc((0f, R, 0f), 0.02f, R, solid, 0u, null, (1.5f, 0f, 0f))], made);
                for (var tick = 0; tick < 60; tick++) world.Step(Dt, Sub);
                return (world.View.ParticleX[made[0].Lane], made[0].Lane, 0);
            }
        }
        var (with, without) = (Slide(true), Slide(false));
        Readings.Add($"WETFRICTION reused lane: lane {with.Lane}, x {with.X:F6} after a settled patch, {without.X:F6} without");
        Assert.Equal(0, with.Lane);
        Assert.Equal(BitConverter.SingleToInt32Bits(without.X), BitConverter.SingleToInt32Bits(with.X));
    }

    /// <summary>A floor, a drop of liquid 0.4 m off settled into a patch of radius 0.56 m (wet friction 0.1) that holds the point 0, a box of 5 kg at 0 that rests <paramref name="rest"/> ticks, then <paramref name="between"/> and <paramref name="settleLate"/>.</summary>
    private static double RestedThenPushed(int rest, bool settleLate, bool removeAfter)
    {
        var world = FluidWorld(8, wets: 4, gravityY: -9.81f);
        using (world)
        {
            world.AddStaticBox(1, (0f, -0.5f, 0f), (4f, 0.5f, 2f), friction: 0.6f);
            world.SetMaterial(0, 0.6f);
            world.SetMaterial(1, 0.6f, wetFriction: WetMu);
            var scale = 0.05f / R;
            world.CreateParticles([Fluid((0.4f, 0.05f, 0f), FluidMass * scale * scale * scale, 0.05f, material: 1u)]);
            for (var tick = 0; tick < 20; tick++) world.Step(Dt, Sub);
            var patch = new Box3DSoaConstraintHandle[1];
            if (!settleLate) world.Settle([Handle(world, 0)], patch);
            var box = world.AddBox(10ul, (0f, 0.1f, 0f), (0.1f, 0.1f, 0.1f), BoxMass / 0.008f, friction: 0.6f);
            for (var tick = 0; tick < rest; tick++) world.Step(Dt, Sub);
            if (settleLate) world.Settle([Handle(world, 0)], patch);
            if (removeAfter) world.Remove(patch[0]);
            for (var tick = 0; tick < 5; tick++) world.Step(Dt, Sub);
            return Moved(world, [box], 0);
        }
    }

    [Theory]
    [InlineData(5)]
    [InlineData(100)]
    public void Liquid_settled_under_a_resting_box_makes_it_slide(int rest)
    {
        // The box rests dry (its contact recycled, held by friction), then a patch is settled under it: the hook runs on the recycled contact, so friction drops.
        var moved = RestedThenPushed(rest, settleLate: true, removeAfter: false);
        var acceleration = 2.0 * moved / ((double)PushTicks * Dt * PushTicks * Dt);
        Readings.Add($"WETFRICTION late patch after {rest} ticks: acceleration {acceleration:F4} against {WetAcceleration:F4}, relative error {Math.Abs(acceleration / WetAcceleration - 1.0):E3}");
        Assert.True(Math.Abs(acceleration / WetAcceleration - 1.0) <= SlideBound, $"acceleration {acceleration} against {WetAcceleration}");
    }

    [Fact]
    public void Liquid_removed_from_under_a_resting_box_restores_friction()
    {
        // The box rests on the patch (friction lowered), the patch is removed: the recycled contact takes the floor's friction back and holds the push.
        var moved = Math.Abs(RestedThenPushed(20, settleLate: false, removeAfter: true));
        Readings.Add($"WETFRICTION patch removed from under a resting box: moved {moved:E3} m");
        Assert.True(moved < DryBound, $"the box moved {moved} m");
    }

    // Overlapping patches of two liquids: the lowest lane decides, wherever its centre sorts. Wet acceleration, relative error against (F - 0.3 m g) / m: shipped 2.262e-2; M7 (the
    // first holder in x order wins: the 0.05 patch, acceleration 3.51) 2.32: bound sqrt(2.262e-2 x 2.32) = 0.229, separation 10.1.
    private const double LowestLaneBound = 0.229;

    [Fact]
    public void Lowest_lane_decides_where_patches_overlap()
    {
        var world = FluidWorld(8, wets: 4, gravityY: -9.81f, materials: 3);
        using (world)
        {
            world.AddStaticBox(1, (0f, -0.5f, 0f), (4f, 0.5f, 2f), friction: 0.6f);
            world.SetMaterial(0, 0.6f);
            world.SetMaterial(1, 0.6f, wetFriction: 0.3f);
            world.SetMaterial(2, 0.6f, wetFriction: 0.05f);
            var scale = DropRadius / R;
            // Lane 0 sits at +0.05 m (wet 0.3), lane 1 at -0.05 m (wet 0.05): both hold the point 0, and the sort by x meets lane 1 first.
            world.CreateParticles([Fluid((0.05f, DropRadius, 0f), FluidMass * scale * scale * scale, DropRadius, material: 1u),
                                   Fluid((-0.05f, DropRadius, 0f), FluidMass * scale * scale * scale, DropRadius, material: 2u)]);
            for (var tick = 0; tick < 20; tick++) world.Step(Dt, Sub);
            world.Settle([Handle(world, 0)]);
            world.Settle([Handle(world, 1)]);
            var box = world.AddBox(10ul, (0f, 0.1f, 0f), (0.1f, 0.1f, 0.1f), BoxMass / 0.008f, friction: 0.6f);
            var moved = Moved(world, [box], 0);
            var expected = (PushForce - 0.3 * BoxMass * 9.81) / BoxMass;
            var acceleration = 2.0 * moved / ((double)PushTicks * Dt * PushTicks * Dt);
            Readings.Add($"WETFRICTION lowest lane: acceleration {acceleration:F4} against {expected:F4}, relative error {Math.Abs(acceleration / expected - 1.0):E3}");
            Assert.True(Math.Abs(acceleration / expected - 1.0) <= LowestLaneBound, $"acceleration {acceleration} against {expected}");
        }
    }
}
