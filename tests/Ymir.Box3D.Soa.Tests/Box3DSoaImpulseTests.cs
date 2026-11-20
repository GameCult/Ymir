using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

/// <summary>
/// ContactImpulse and the hit fact's Impulse are the normal impulse contacts delivered over the whole step. The
/// oracle is momentum: only contacts and gravity act, so over one step the contact impulse along the contact normal
/// is the change of the system's momentum along it plus the weight's impulse, whatever the joints and sub-steps did.
/// </summary>
public sealed class Box3DSoaImpulseTests
{
    private const float G = 10f;
    private static readonly (float, float, float, float) QuarterTurnZ = (0f, 0f, MathF.Sin(MathF.PI / 4), MathF.Cos(MathF.PI / 4));

    private static float SphereMass(float radius, float density) => 4f / 3f * MathF.PI * radius * radius * radius * density;
    private static float CapsuleMass(float radius, float halfLength, float density) =>
        MathF.PI * radius * radius * (2f * halfLength + 4f / 3f * radius) * density;

    /// <summary>Momentum along Y of the given lanes with their masses.</summary>
    private static double MomentumY(Box3DSoaWorld world, (int Lane, float Mass)[] bodies)
    {
        double p = 0;
        foreach (var (lane, mass) in bodies) p += (double)mass * world.View.VelocityY[lane];
        return p;
    }

    /// <summary>
    /// Torso box 23.8 kg, upper-arm capsule on a spherical shoulder and forearm capsule on a revolute elbow, as in the
    /// probe that measured Box3D. Lanes 0, 1, 2; the arm lies along +X with the forearm's centre at x = 0.78.
    /// </summary>
    private static (Box3DSoaWorld World, (int Lane, float Mass)[] Arm) Arm(float gravityY)
    {
        var world = new Box3DSoaWorld(capacity: 16, gravityY: gravityY, jointCapacity: 2);
        var torso = world.AddBox(1, (0f, 0f, 0f), (0.175f, 0.25f, 0.1f), 680f);
        var upper = world.AddCapsule(2, (0.40f, 0.2f, 0f), 0.045f, 0.15f, 1000f, rotation: QuarterTurnZ);
        var fore = world.AddCapsule(3, (0.78f, 0.2f, 0f), 0.04f, 0.14f, 1000f, rotation: QuarterTurnZ);
        world.AddJoint(Box3DSoaJointKind.Spherical, torso, upper, (0.2f, 0.2f, 0f), (0f, 0f, 1f));
        world.AddJoint(Box3DSoaJointKind.Revolute, upper, fore, (0.6f, 0.2f, 0f), (0f, 0f, 1f));
        var arm = new[] { (torso, 0.35f * 0.5f * 0.2f * 680f), (upper, CapsuleMass(0.045f, 0.15f, 1000f)), (fore, CapsuleMass(0.04f, 0.14f, 1000f)) };
        return (world, arm);
    }

    /// <summary>Fires a ball at the forearm from above; returns the sum over the run of the forearm's ContactImpulse and the ball's -m dv.</summary>
    private static (double Delivered, double Truth, double Peak, int Hits) Strike(float radius, float mass, float speed, float restitution, float gap, int hz, int subSteps, int steps)
    {
        var (world, _) = Arm(0f);
        using var _w = world;
        const int fore = 2;
        // Box3D opens a contact only once shapes overlap and a fast ball would be through the forearm by then (its
        // continuous collision is for static shapes and bullets, which the SoA world has none of). So the ball starts at
        // rest where the contact exists (a negative gap overlaps by that much) and an impulse launches it into the forearm.
        var ball = world.AddSphere(10, (0.78f, 0.2f + 0.04f + radius + gap, 0f), radius, mass / SphereMass(radius, 1f), restitution: restitution);
        var dt = 1f / hz;
        world.Step(dt, subSteps);
        world.Step(dt, subSteps);
        world.ApplyImpulse(ball, (0f, -mass * speed, 0f), (0.78f, 0.2f + 0.04f + radius + gap, 0f));
        var launched = -(double)mass * speed;
        var truths = new double[steps];
        var lanes = new float[steps];
        var hitImpulse = new float[steps];
        var hitSteps = new bool[steps];
        var hits = 0;
        var before = launched;
        for (var i = 0; i < steps; i++)
        {
            world.Step(dt, subSteps);
            var after = (double)mass * world.View.VelocityY[ball];
            truths[i] = after - before;
            before = after;
            lanes[i] = world.View.ContactImpulse[fore];
            foreach (var hit in world.Facts.Hit) { hitImpulse[i] += hit.Impulse; hitSteps[i] = true; hits++; }
        }
        var peak = truths.Max(Math.Abs);
        for (var i = 0; i < steps; i++)
        {
            Assert.True(Math.Abs(lanes[i] - truths[i]) <= 1e-4 * peak, $"step {i}: forearm ContactImpulse {lanes[i]} vs the ball's momentum change {truths[i]}");
            if (hitSteps[i]) Assert.Equal(lanes[i], hitImpulse[i], 1e-4 * peak);
        }
        var delivered = (double)lanes.Sum();
        var truth = truths.Sum();
        return (delivered, truth, peak, hits);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(0.5f)]
    public void A_jointed_forearm_hit_is_exact_per_step_and_the_hit_carries_the_same_impulse(float restitution)
    {
        var (delivered, truth, peak, hits) = Strike(0.1f, 5f, 10f, restitution, gap: 0.05f, hz: 60, subSteps: 4, steps: 9);
        Assert.InRange(peak, 14f, 25f); // a 5 kg ball at 10 m/s onto a forearm: tens of N s, not the tenth the ball's own approach speed implies
        Assert.Equal(truth, delivered, 1e-4 * peak);
        Assert.Equal(1, hits); // the hit fact carried the impulse checked above
    }

    [Fact]
    public void A_BB_is_not_a_cannonball()
    {
        var (bb, bbTruth, _, _) = Strike(0.003f, 0.0003f, 100f, 0f, gap: -0.001f, hz: 60, subSteps: 4, steps: 9);
        Assert.True(bb >= 0.030f * 0.99f && bb <= 0.030f * 1.01f, $"BB delivered {bb} N s");
        Assert.Equal(bbTruth, bb, 1e-6);
        var (ball, _, _, _) = Strike(0.1f, 5f, 10f, 0f, gap: 0.05f, hz: 60, subSteps: 4, steps: 9);
        Assert.True(ball > 400f * bb, $"the ball's {ball} N s dwarfs the BB's {bb} N s");
    }

    [Fact]
    public void A_forearm_falling_onto_a_wall_delivers_the_whole_arms_momentum_change()
    {
        // The forearm lands on a slab while the shoulder and elbow keep the rest of the arm moving: the wall's impulse is
        // the change of the three bodies' momentum plus their weight, not the forearm's own.
        var (world, arm) = Arm(-G);
        using var _w = world;
        world.AddStaticBox(9, (1.0f, 0.2f - 0.04f - 0.8f - 0.1f, 0f), (0.3f, 0.1f, 0.5f)); // under the forearm only: the upper arm never reaches it
        var totalMass = arm.Sum(b => b.Mass);
        const int fore = 2;
        var dt = 1f / 60f;
        double peak = 0;
        var truths = new List<(double Truth, float Lane)>();
        for (var i = 0; i < 90; i++)
        {
            var before = MomentumY(world, arm);
            world.Step(dt, 4);
            var truth = MomentumY(world, arm) - before + totalMass * G * dt;
            truths.Add((truth, world.View.ContactImpulse[fore]));
            peak = Math.Max(peak, Math.Abs(truth));
        }
        Assert.True(peak > 10.0, $"the arm hit the slab (peak {peak} N s)");
        for (var i = 0; i < truths.Count; i++)
        {
            Assert.True(Math.Abs(truths[i].Lane - truths[i].Truth) <= 1e-4 * peak, $"step {i}: {truths[i].Lane} vs {truths[i].Truth}");
        }
    }

    [Fact]
    public void A_swung_arm_into_a_wall_delivers_the_whole_arms_momentum_change()
    {
        // The swing the probe measured: impulses launch the upper arm and the forearm off-centre (spin included) at 6 m/s,
        // no gravity, and the forearm meets a slab. The wall's impulse is the three bodies' momentum change.
        var (world, arm) = Arm(0f);
        using var _w = world;
        world.AddStaticBox(9, (1.0f, 0.2f - 0.04f - 0.5f - 0.1f, 0f), (0.3f, 0.1f, 0.5f), friction: 0f); // under the forearm only; frictionless, as the lane is the normal channel only
        const int fore = 2;
        world.ApplyImpulse(1, (0f, -arm[1].Mass * 6f, 0f), (0.55f, 0.2f, 0f));
        world.ApplyImpulse(fore, (0f, -arm[2].Mass * 6f, 0f), (0.95f, 0.2f, 0f));
        // The swung forearm meets the slab at a tilt, so the impulse is along the contact normal, not Y: the truth is the
        // length of the momentum change vector (every contact point of the one contact shares its normal).
        var before = (X: 0.0, Y: -(double)(arm[1].Mass + arm[2].Mass) * 6f, Z: 0.0);
        var dt = 1f / 60f;
        var truths = new List<double>();
        var lanes = new List<float>();
        for (var i = 0; i < 8; i++) // the impact is step 3; after it the forearm swings into its own torso, an internal contact whose impulses cancel in the sum
        {
            world.Step(dt, 4);
            var v = world.View;
            var after = (X: 0.0, Y: 0.0, Z: 0.0);
            foreach (var (lane, mass) in arm) after = (after.X + (double)mass * v.VelocityX[lane], after.Y + (double)mass * v.VelocityY[lane], after.Z + (double)mass * v.VelocityZ[lane]);
            var dx = after.X - before.X; var dy = after.Y - before.Y; var dz = after.Z - before.Z;
            truths.Add(Math.Sqrt(dx * dx + dy * dy + dz * dz));
            before = after;
            lanes.Add(v.ContactImpulse[fore]);
        }
        var peak = truths.Max(Math.Abs);
        Assert.True(peak > 5.0, $"the arm hit the slab (peak {peak} N s)");
        for (var i = 0; i < truths.Count; i++)
        {
            Assert.True(Math.Abs(lanes[i] - truths[i]) <= 1e-4 * peak, $"step {i}: {lanes[i]} vs {truths[i]}");
        }
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(0.5f)]
    public void A_body_with_more_contacts_than_graph_colours_still_reads_every_one(float restitution)
    {
        // One big sphere with 60 small ones converging on it: more contacts on the big body than Box3D has graph colours,
        // so some are solved by its overflow path. Each small ball has one contact, so its lane is its own momentum
        // change; the big ball's lane is the sum of theirs.
        const int count = 60;
        const float bigRadius = 0.5f, smallRadius = 0.05f, speed = 4f;
        using var world = new Box3DSoaWorld(capacity: count + 2, gravityY: 0f);
        var big = world.AddSphere(1, (0f, 0f, 0f), bigRadius, 500f, restitution: restitution);
        var smallMass = SphereMass(smallRadius, 500f);
        var smalls = new int[count];
        var dirs = new (float X, float Y, float Z)[count];
        for (var i = 0; i < count; i++)
        {
            var y = 1f - 2f * (i + 0.5f) / count;
            var r = MathF.Sqrt(1f - y * y);
            var a = i * MathF.PI * (3f - MathF.Sqrt(5f));
            dirs[i] = (r * MathF.Cos(a), y, r * MathF.Sin(a));
            var d = bigRadius + smallRadius - 0.001f;
            smalls[i] = world.AddSphere((ulong)(10 + i), (dirs[i].X * d, dirs[i].Y * d, dirs[i].Z * d), smallRadius, 500f, restitution: restitution);
        }
        var dt = 1f / 60f;
        world.Step(dt);
        world.Step(dt);
        Assert.True(world.View.ContactCount[big] > 24, $"the big ball has {world.View.ContactCount[big]} contacts, more than Box3D has graph colours");
        for (var i = 0; i < count; i++)
        {
            world.ApplyImpulse(smalls[i], (-dirs[i].X * smallMass * speed, -dirs[i].Y * smallMass * speed, -dirs[i].Z * smallMass * speed),
                (dirs[i].X * bigRadius, dirs[i].Y * bigRadius, dirs[i].Z * bigRadius));
        }
        var launched = new (double X, double Y, double Z)[count];
        for (var i = 0; i < count; i++) launched[i] = (-dirs[i].X * smallMass * speed, -dirs[i].Y * smallMass * speed, -dirs[i].Z * smallMass * speed);
        var peak = 0.0;
        var steps = 12;
        var sumOfSmalls = new double[steps];
        var bigLane = new float[steps];
        var worst = new double[steps];
        for (var s = 0; s < steps; s++)
        {
            world.Step(dt);
            var v = world.View;
            for (var i = 0; i < count; i++)
            {
                var l = smalls[i];
                var now = (X: (double)smallMass * v.VelocityX[l], Y: (double)smallMass * v.VelocityY[l], Z: (double)smallMass * v.VelocityZ[l]);
                var dp = Math.Sqrt((now.X - launched[i].X) * (now.X - launched[i].X) + (now.Y - launched[i].Y) * (now.Y - launched[i].Y) + (now.Z - launched[i].Z) * (now.Z - launched[i].Z));
                launched[i] = now;
                sumOfSmalls[s] += v.ContactImpulse[l];
                worst[s] = Math.Max(worst[s], Math.Abs(v.ContactImpulse[l] - dp));
                peak = Math.Max(peak, dp);
            }
            bigLane[s] = v.ContactImpulse[big];
        }
        Assert.True(peak > 0.5, $"the small balls hit (peak {peak} N s)");
        for (var s = 0; s < steps; s++)
        {
            Assert.True(Math.Abs(bigLane[s] - sumOfSmalls[s]) <= 1e-4 * sumOfSmalls.Max(), $"step {s}: big {bigLane[s]} vs the smalls' sum {sumOfSmalls[s]}");
            Assert.True(worst[s] <= 1e-3 * peak, $"step {s}: a small ball's lane is off its momentum change by {worst[s]}");
        }
    }

    [Theory]
    [InlineData(60, 1, 0f)]
    [InlineData(60, 4, 0f)]
    [InlineData(60, 1, 0.5f)]
    [InlineData(60, 4, 0.5f)]
    [InlineData(240, 1, 0f)]
    [InlineData(240, 4, 0f)]
    [InlineData(240, 1, 0.5f)]
    [InlineData(240, 4, 0.5f)]
    public void Drop_impulse_is_exact_every_step(int hz, int subSteps, float restitution)
    {
        using var world = new Box3DSoaWorld(capacity: 4, gravityY: -G);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        const float radius = 0.1f;
        const float density = 125f; // 0.52 kg
        var ball = world.AddSphere(2, (0f, 1.5f, 0f), radius, density, restitution: restitution);
        var mass = SphereMass(radius, density);
        var dt = 1f / hz;
        var truths = new double[hz];
        var lanes = new float[hz];
        var hits = 0;
        for (var i = 0; i < hz; i++)
        {
            var before = (double)mass * world.View.VelocityY[ball];
            world.Step(dt, subSteps);
            truths[i] = mass * (double)world.View.VelocityY[ball] - before + mass * G * dt;
            lanes[i] = world.View.ContactImpulse[ball];
            foreach (var hit in world.Facts.Hit)
            {
                hits++;
                Assert.Equal(lanes[i], hit.Impulse, 1e-4 * truths.Max(Math.Abs));
            }
        }
        var peak = truths.Max(Math.Abs);
        Assert.InRange(peak, 2.5, 6.0);
        Assert.True(hits >= 1);
        for (var i = 0; i < hz; i++)
        {
            Assert.True(Math.Abs(lanes[i] - truths[i]) <= 1e-4 * peak, $"step {i}: {lanes[i]} vs {truths[i]}");
        }
    }

    [Fact]
    public void Each_hit_reads_its_own_pair_when_the_slab_has_several_contacts()
    {
        // Two balls of different mass land on the slab in the same step: the slab's contact list holds both, and each hit
        // fact is its own ball's impulse, whichever of the slab's contacts Box3D lists first.
        using var world = new Box3DSoaWorld(capacity: 8, gravityY: -G);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        var light = world.AddSphere(2, (-1f, 1.5f, 0f), 0.1f, 125f);
        var heavy = world.AddSphere(3, (1f, 1.5f, 0f), 0.1f, 500f);
        var dt = 1f / 60f;
        var checkedHits = 0;
        for (var i = 0; i < 90; i++)
        {
            world.Step(dt);
            foreach (var hit in world.Facts.Hit)
            {
                var ball = hit.LaneA == light || hit.LaneB == light ? light : heavy;
                Assert.Equal(world.View.ContactImpulse[ball], hit.Impulse, 1e-3f);
                checkedHits++;
            }
        }
        Assert.Equal(2, checkedHits);
        Assert.Equal(world.View.ContactImpulse[heavy] / 4f, world.View.ContactImpulse[light], 1e-3f); // resting: weight scales with mass
    }

    [Fact]
    public void Resting_impulse_is_the_weight_and_a_body_touching_nothing_reads_zero()
    {
        using var world = new Box3DSoaWorld(capacity: 8, gravityY: -G);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        world.AddSphere(2, (0f, 0.6f, 0f), 0.5f, 1f);
        world.AddSphere(3, (10f, 1000f, 10f), 0.5f, 1f); // still falling after the whole run
        var dt = 1f / 60f;
        for (var i = 0; i < 300; i++) world.Step(dt);
        var weight = SphereMass(0.5f, 1f) * G;
        var view = world.View;
        Assert.InRange(view.ContactImpulse[1] / dt, weight * 0.999f, weight * 1.001f);
        Assert.InRange(view.ContactImpulse[0] / dt, weight * 0.999f, weight * 1.001f); // the slab carries the same load
        Assert.Equal(0f, view.ContactImpulse[2]);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(0.5f)]
    public void Two_free_bodies_exchange_exactly_the_momentum_B_gains(float restitution)
    {
        using var world = new Box3DSoaWorld(capacity: 4, gravityY: 0f);
        world.AddSphere(10, (-0.5f, 0f, 0f), 0.1f, 1f / SphereMass(0.1f, 1f), velocity: (5f, 0f, 0f), restitution: restitution);
        var b = world.AddSphere(11, (0f, 0f, 0f), 0.1f, 3f / SphereMass(0.1f, 1f), restitution: restitution);
        const float massB = 3f;
        const float dt = 1f / 60f;
        double delivered = 0, truth = 0, peak = 0;
        for (var i = 0; i < 12; i++)
        {
            var before = (double)massB * world.View.VelocityX[b];
            world.Step(dt);
            var step = massB * (double)world.View.VelocityX[b] - before;
            peak = Math.Max(peak, Math.Abs(step));
            Assert.True(Math.Abs(world.View.ContactImpulse[b] - step) <= 1e-4 * Math.Max(peak, 1e-6), $"step {i}");
            foreach (var hit in world.Facts.Hit) Assert.Equal(world.View.ContactImpulse[b], hit.Impulse, 1e-4 * peak);
            delivered += world.View.ContactImpulse[b];
            truth += step;
        }
        Assert.InRange(peak, 3.5, 6.0);
        Assert.Equal(truth, delivered, 1e-4 * peak);
    }
}
