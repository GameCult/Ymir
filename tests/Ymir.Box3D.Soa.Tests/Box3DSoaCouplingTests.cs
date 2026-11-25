using System.Globalization;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>
/// Particles and bodies coupled inside Box3D's substep (docs/soa-world.md, particle section): a held weight stays held,
/// momentum and energy are kept through a throw and a snap, and a release takes effect in its step. The ropes are built
/// by a test-local builder. Every reading the tests judge is also appended to the file named by YMIR_READINGS when set.
/// </summary>
public sealed class Box3DSoaCouplingTests
{
    private const float Dt = Box3DSoaParticleTests.Dt;
    private const int Sub = Box3DSoaParticleTests.Sub;
    private const float G = 10f;
    private const float ParticleMass = Box3DSoaParticleTests.ParticleMass;
    private const float ParticleRadius = Box3DSoaParticleTests.ParticleRadius;
    private const float Spacing = Box3DSoaParticleTests.Spacing;

    private static void Reading(string line)
    {
        var path = Environment.GetEnvironmentVariable("YMIR_READINGS");
        if (!string.IsNullOrEmpty(path)) File.AppendAllText(path, line + "\n");
    }

    private static string F(double x) => x.ToString("G6", CultureInfo.InvariantCulture);
    private static Pos[] Line(Pos from, Pos step, int count) => Graph.Line(from, step, count);
    private static float Dens(float mass, float radius) => Graph.Dens(mass, radius);
    private static float Sq(float x) => x * x;
    private static Box3DSoaWorld World(float gravityX = 0f) => Box3DSoaParticleTests.World(gravityX: gravityX, gravityY: -G);
    private static void Run(Box3DSoaWorld world, int steps) => Box3DSoaParticleTests.Run(world, steps);

    /// <summary>A rope of 48 particles hanging from a static point with a sphere of <paramref name="mass"/> tied under its end.</summary>
    internal static (int Sphere, Box3DSoaConstraintHandle Top, Box3DSoaConstraintHandle Bottom, Made Rope) Hung(
        Box3DSoaWorld world, float mass, float radius = 0.1f, Pos velocity = default)
    {
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        var top = world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        var end = positions[47];
        var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Dens(mass, radius), velocity: velocity);
        var bottom = world.Attach(Box3DSoaAttachKind.Body, rope.Particles[47], sphere, (0f, radius, 0f), tether: true);
        return (sphere, top, bottom, rope);
    }

    /// <summary>Kinetic and potential energy of the particles and of the listed spheres (translation and spin), gravity along -y.</summary>
    private static double Energy(Box3DSoaView v, params (int Lane, float Mass, float Radius)[] spheres)
    {
        double e = 0;
        for (var i = 0; i < v.ParticleCount; i++)
        {
            if (v.ParticleLive[i] == 0 || v.ParticleInvMass[i] <= 0f) continue;
            double m = 1.0 / v.ParticleInvMass[i];
            e += 0.5 * m * (Sq(v.ParticleVx[i]) + Sq(v.ParticleVy[i]) + Sq(v.ParticleVz[i])) + m * G * v.ParticleY[i];
        }
        foreach (var (lane, m, r) in spheres)
        {
            var inertia = 0.4 * m * r * r;
            e += 0.5 * m * (Sq(v.VelocityX[lane]) + Sq(v.VelocityY[lane]) + Sq(v.VelocityZ[lane]))
               + 0.5 * inertia * (Sq(v.AngularVelocityX[lane]) + Sq(v.AngularVelocityY[lane]) + Sq(v.AngularVelocityZ[lane]))
               + m * G * v.PositionY[lane];
        }
        return e;
    }

    private static double ParticleMomentumX(Box3DSoaView v)
    {
        double p = 0;
        for (var i = 0; i < v.ParticleCount; i++)
            if (v.ParticleLive[i] != 0 && v.ParticleInvMass[i] > 0f) p += v.ParticleVx[i] / v.ParticleInvMass[i];
        return p;
    }

    [Fact]
    public void Attached_end_stays_on_its_point()
    {
        // The end particle is tied to the sphere's top and skips that sphere in contact. A contact that pushed it off its own body would read as a gap of its radius (12 mm).
        using var world = World();
        var (sphere, _, _, _) = Hung(world, 1f);
        var view = world.View;
        const float radius = 0.1f;
        var worst = 0f;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt, Sub);
            if (i < 10) continue;
            float qx = view.RotationX[sphere], qy = view.RotationY[sphere], qz = view.RotationZ[sphere], qw = view.RotationW[sphere];
            var tx = 2f * (qy * 0f - qz * radius); var ty = 2f * (qz * 0f - qx * 0f); var tz = 2f * (qx * radius - qy * 0f);
            var px = view.PositionX[sphere] + qw * tx + (qy * tz - qz * ty);
            var py = view.PositionY[sphere] + radius + qw * ty + (qz * tx - qx * tz);
            var pz = view.PositionZ[sphere] + qw * tz + (qx * ty - qy * tx);
            worst = MathF.Max(worst, MathF.Sqrt(Sq(view.ParticleX[47] - px) + Sq(view.ParticleY[47] - py) + Sq(view.ParticleZ[47] - pz)));
        }
        Reading($"ATTACHED_END worst gap {F(worst)} m");
        Assert.True(worst < 1e-4f, $"the end particle sat {worst} m from its body point");
    }

    [Fact]
    public void Tethers_hold_the_rest_path()
    {
        // A 70 kg particle at the end of a 48-particle rope from a static point: no particle may be farther from the
        // attachment than the rest path to it, however the load is routed.
        using var world = World();
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var masses = new float[48];
        Array.Fill(masses, ParticleMass);
        masses[47] = 70f;
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius, masses: masses);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        var view = world.View;
        var worst = 0f;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt, Sub);
            for (var k = 1; k < 48; k++)
                worst = MathF.Max(worst, MathF.Sqrt(Sq(view.ParticleX[k] - positions[0].X) + Sq(view.ParticleY[k] - positions[0].Y) + Sq(view.ParticleZ[k] - positions[0].Z)) - k * Spacing);
        }
        Reading($"TETHER_BOUND worst overshoot {F(worst)} m");
        Assert.True(worst < 1e-5f, $"a particle sits {worst} m past its rest path");
    }

    // ---- momentum and energy ---------------------------------------------------------------------------------

    [Theory]
    [InlineData(5f)]
    [InlineData(70f)]
    public void Free_throw_keeps_momentum(float mass)
    {
        // A sphere thrown at 5 m/s with a 48 x 20 g rope tied behind it, the rope at rest: x momentum is M x 5 for good.
        const float radius = 0.1f;
        using var world = World();
        var rope = Graph.Rope(world, Line((0f, 10f, 0f), (-Spacing, 0f, 0f), 48), ParticleMass, ParticleRadius);
        var sphere = world.AddSphere(10, (radius, 10f, 0f), radius, Dens(mass, radius), velocity: (5f, 0f, 0f));
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[0], sphere, (-radius, 0f, 0f), tether: true);
        var view = world.View;
        var floor = 0.95 * mass * 5.0 / (mass + 48 * ParticleMass);
        double e0 = 0, worstMomentum = 0, worstEnergy = double.MinValue, firstVx = 0;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt, Sub);
            var momentum = mass * view.VelocityX[sphere] + ParticleMomentumX(view);
            var energy = Energy(view, (sphere, mass, radius));
            if (i == 0) { e0 = energy; firstVx = view.VelocityX[sphere]; }
            worstMomentum = Math.Max(worstMomentum, Math.Abs(momentum - mass * 5.0) / (mass * 5.0));
            worstEnergy = Math.Max(worstEnergy, energy - e0);
        }
        Reading($"FREE_THROW M={mass} vx after tick 0 {F(firstVx)} (floor {F(floor)}) worst momentum error {F(worstMomentum)} worst energy over tick 0 {F(worstEnergy)} J");
        Assert.True(firstVx >= floor, $"the body kept {firstVx} m/s after one tick, at least {floor} expected");
        Assert.True(worstMomentum < 1e-3, $"x momentum drifted by {worstMomentum:P3}");
        Assert.True(worstEnergy < 0.01, $"the system gained {worstEnergy} J");
    }

    [Theory]
    [InlineData(5f)]
    [InlineData(70f)]
    public void Throw_up_apex(float mass)
    {
        const float radius = 0.1f;
        using var world = World();
        var rope = Graph.Rope(world, Line((0f, 10f - radius, 0f), (0f, -Spacing, 0f), 48), ParticleMass, ParticleRadius); // hangs below the body
        var sphere = world.AddSphere(10, (0f, 10f, 0f), radius, Dens(mass, radius), velocity: (0f, 5f, 0f));
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[0], sphere, (0f, -radius, 0f), tether: true);
        var view = world.View;
        var apex = 10f;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt, Sub);
            apex = MathF.Max(apex, view.PositionY[sphere]);
        }
        // The body alone would rise 1.25 m; the rope it picks up from rest takes the momentum it needs.
        var ratio = mass / (mass + 48 * ParticleMass);
        var expected = 10f + 1.25f * ratio * ratio;
        Reading($"THROW_UP M={mass} apex {F(apex)} expected {F(expected)}");
        Assert.True(MathF.Abs(apex - expected) < 0.01f * (expected - 10f), $"apex {apex}, expected {expected}");
    }

    [Fact]
    public void Snap_taut_loses_energy()
    {
        const float radius = 0.05f;
        foreach (var mass in new[] { 5f, 70f })
        {
            using var world = World();
            var (sphere, _, _, _) = Hung(world, mass, radius, velocity: (0f, -6f, 0f));
            var view = world.View;
            var before = 0.5 * mass * 36.0;
            double first = 0, worst = 0, vyAfterOne = 0;
            for (var i = 0; i < 240; i++)
            {
                world.Step(Dt, Sub);
                var ke = 0.5 * mass * (Sq(view.VelocityX[sphere]) + Sq(view.VelocityY[sphere]) + Sq(view.VelocityZ[sphere]));
                if (i == 0) { first = ke; vyAfterOne = view.VelocityY[sphere]; }
                worst = Math.Max(worst, ke);
            }
            Reading($"SNAP M={mass} body KE {F(first)} of {F(before)} J after one tick, vy {F(vyAfterOne)}, worst later {F(worst)} J");
            Assert.True(worst <= before, $"{mass} kg: the body's energy rose to {worst} J from {before} J");
            if (mass == 5f) Assert.True(first <= 0.05 * before, $"5 kg kept {first} J of {before} J after the snap");
            else Assert.True(Math.Abs(vyAfterOne) < 0.1, $"70 kg moves at {vyAfterOne} m/s after the snap");
        }
    }

    [Fact]
    public void Pendulum_gains_no_energy()
    {
        // A 45 degree swing for 12 s, the weight as a tied body and as a heavy particle at the rope's end (the control the
        // solver is known to damp): the body must lose no less than nothing and no more than 1.25 times the control.
        const float radius = 0.05f;
        foreach (var mass in new[] { 5f, 70f })
        {
            var th = 45f * MathF.PI / 180f;
            var step = (MathF.Sin(th) * Spacing, -MathF.Cos(th) * Spacing, 0f);
            double Swing(bool asBody)
            {
                using var world = World();
                var positions = Line((0f, 5f, 0f), step, 48);
                var masses = new float[48];
                Array.Fill(masses, ParticleMass);
                if (!asBody) masses[47] = mass;
                var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius, masses: masses);
                world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
                (int, float, float)[] bodies = [];
                if (asBody)
                {
                    var end = positions[47];
                    var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Dens(mass, radius));
                    world.Attach(Box3DSoaAttachKind.Body, rope.Particles[47], sphere, (0f, radius, 0f), tether: true);
                    bodies = [(sphere, mass, radius)];
                }
                var view = world.View;
                double e0 = 0, over = double.MinValue, last = 0;
                for (var i = 0; i < 12 * 120; i++)
                {
                    world.Step(Dt, Sub);
                    var e = Energy(view, bodies);
                    if (i == 0) e0 = e;
                    over = Math.Max(over, e - e0);
                    last = e - e0;
                }
                Assert.True(over < 0.01, $"{mass} kg {(asBody ? "body" : "control")}: the pendulum gained {over} J");
                return last;
            }
            var body = Swing(true);
            var control = Swing(false);
            Reading($"PENDULUM M={mass} body {F(body)} J control {F(control)} J after 12 s");
            Assert.True(-body <= 1.25 * -control, $"{mass} kg: the body lost {-body} J, the control {-control} J");
        }
    }

    [Fact]
    public void A_hung_body_keeps_its_spin_about_the_vertical()
    {
        const float radius = 0.05f, mass = 5f;
        using var world = World();
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        var end = positions[47];
        var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Dens(mass, radius));
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[47], sphere, (0f, radius, 0f), tether: true);
        var inertia = 0.4f * mass * radius * radius;
        Pos centre = (end.X, end.Y - radius, end.Z);
        world.ApplyImpulse(sphere, (0f, 0f, 30f * inertia / radius), (centre.X + radius, centre.Y, centre.Z)); // a couple about -y: omega_y = -30
        world.ApplyImpulse(sphere, (0f, 0f, -30f * inertia / radius), centre);
        var view = world.View;
        const float start = -30f;
        Run(world, 8 * 120);
        var spin = view.AngularVelocityY[sphere];
        Reading($"SPIN omega_y {F(start)} -> {F(spin)}");
        Assert.True(MathF.Abs(spin - start) < 0.01f * MathF.Abs(start), $"the spin went from {start} to {spin} rad/s");
    }

    // ---- release ---------------------------------------------------------------------------------------------

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Release_takes_effect_in_step(bool destroyParticles)
    {
        using var world = World();
        var (sphere, _, bottom, rope) = Hung(world, 70f);
        var view = world.View;
        Run(world, 3 * 120);
        if (destroyParticles) world.DestroyParticles(rope.Particles); else world.Remove(bottom);
        var worst = 0f;
        var impulse = float.MinValue;
        for (var k = 1; k <= 4; k++)
        {
            world.Step(Dt, Sub);
            worst = MathF.Max(worst, MathF.Abs(view.VelocityY[sphere] - (-G * Dt * k)));
            impulse = MathF.Max(impulse, view.ParticleImpulseY[sphere]);
        }
        Reading($"RELEASE destroyParticles={destroyParticles} worst vy error {F(worst)} m/s, greatest impulse Y {F(impulse)}");
        Assert.True(worst < 0.005f, $"the body is {worst} m/s off free fall in the step after the command");
        Assert.True(impulse <= 0f, $"the released rope still pushed the body up by {impulse} N s");
    }

    [Fact]
    public void Particle_impulse_is_what_Box3D_applied()
    {
        // M (v_after - v_before) - M g dt is, per axis, the impulse the particles gave the body this tick: the lane reads exactly that.
        const float radius = 0.1f;
        foreach (var thrown in new[] { false, true })
        {
            const float mass = 70f;
            using var world = World();
            int sphere;
            if (!thrown) sphere = Hung(world, mass).Sphere;
            else
            {
                var rope = Graph.Rope(world, Line((0f, 10f, 0f), (-Spacing, 0f, 0f), 48), ParticleMass, ParticleRadius);
                sphere = world.AddSphere(10, (radius, 10f, 0f), radius, Dens(mass, radius), velocity: (5f, 0f, 0f));
                world.Attach(Box3DSoaAttachKind.Body, rope.Particles[0], sphere, (-radius, 0f, 0f), tether: true);
            }
            var view = world.View;
            var (vx, vy, vz) = (view.VelocityX[sphere], view.VelocityY[sphere], view.VelocityZ[sphere]);
            var worst = 0f;
            for (var i = 0; i < 360; i++)
            {
                world.Step(Dt, Sub);
                var dx = mass * (view.VelocityX[sphere] - vx) - view.ParticleImpulseX[sphere];
                var dy = mass * (view.VelocityY[sphere] - vy) - mass * -G * Dt - view.ParticleImpulseY[sphere];
                var dz = mass * (view.VelocityZ[sphere] - vz) - view.ParticleImpulseZ[sphere];
                worst = MathF.Max(worst, MathF.Max(MathF.Abs(dx), MathF.Max(MathF.Abs(dy), MathF.Abs(dz))));
                (vx, vy, vz) = (view.VelocityX[sphere], view.VelocityY[sphere], view.VelocityZ[sphere]);
            }
            Reading($"IMPULSE_LANE thrown={thrown} worst |M dv - M g dt - lane| {F(worst)} N s (M g dt = {F(mass * G * Dt)})");
            Assert.True(worst < 1e-3f, $"thrown={thrown}: the lane differs from the body's change by {worst} N s");
        }
    }

    // ---- bodies and particles without the pull -------------------------------------------------------------------

    [Theory]
    [InlineData(0.6f, true)]
    [InlineData(0f, false)]
    public void Material_friction_holds_on_a_slope(float friction, bool holds)
    {
        // Gravity tilted 20 degrees on a rope lying on a static box: friction 0.6 (above tan 20) holds it, friction 0 lets it go.
        var tilt = 20f * MathF.PI / 180f;
        using var world = World(gravityX: G * MathF.Sin(tilt));
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f), friction: 0.6f);
        world.SetMaterial(0, friction);
        var rope = Graph.Rope(world, Line((0f, ParticleRadius, 0f), (0f, 0f, Spacing), 12), ParticleMass, ParticleRadius);
        var view = world.View;
        Run(world, 2 * 120);
        var slid = MathF.Abs(view.ParticleX[rope.Particles[6]]);
        Reading($"SLOPE mu={friction} slid {F(slid)} m in 2 s");
        if (holds) Assert.True(slid < 0.01f, $"friction {friction} let the rope slide {slid} m");
        else Assert.True(slid > 1f, $"friction 0 slid only {slid} m");
    }

    [Fact]
    public void Particles_step_with_no_awake_body()
    {
        // Box3D returns early from a world with nothing awake; the particle substeps run all the same.
        using var world = World();
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f));
        var rope = Graph.Rope(world, Line((0f, 1f, 0f), (0f, 0f, Spacing), 8), ParticleMass, ParticleRadius);
        var view = world.View;
        Run(world, 24); // 0.2 s
        var fell = 1f - view.ParticleY[rope.Particles[3]];
        Reading($"NO_AWAKE fell {F(fell)} m in 0.2 s (g t^2 / 2 = 0.2)");
        Assert.True(MathF.Abs(fell - 0.2f) < 0.02f * 0.2f, $"the rope fell {fell} m, expected 0.2 m");
        Run(world, 4 * 120);
        Assert.Equal(ParticleRadius, view.ParticleY[rope.Particles[3]], 1e-3f);
        Assert.True(MathF.Abs(view.ParticleVy[rope.Particles[3]]) < 1e-3f, "the rope rests on the box");
    }

    // ---- cost ---------------------------------------------------------------------------------------------------

    [Theory]
    [InlineData(1, false)]
    [InlineData(40, false)]
    [InlineData(1, true)]
    [InlineData(40, true)]
    public void Particle_cost_line(int bodies, bool sheet)
    {
        using var world = new Box3DSoaWorld(64, gravityY: -G, particles: new Box3DSoaParticleCapacities(16 * 48 + 256, 16 * 47 + 480, 16 * 46 + 448, 16, 600, 1));
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f));
        for (var b = 1; b < bodies; b++)
            world.AddSphere((ulong)(100 + b), (0.2f * (b % 8), 0.3f + 0.25f * (b / 8), 0.5f), 0.08f, 800f);
        for (var c = 0; c < 16; c++) // a pile: ropes zig-zag so particles meet their neighbours' links
            Graph.Rope(world, Line((0f, 0.5f + 0.03f * c, 0.02f * (c % 4)), (Spacing, 0f, 0f), 48), ParticleMass, ParticleRadius, Box3DSoaPhase.Of((uint)(c + 1), true, true));
        if (sheet) Graph.Sheet(world, (0.2f, 1.2f, 0f), (0.03f, 0f, 0f), (0f, 0f, 0.03f), 16, 16, ParticleMass, ParticleRadius, Box3DSoaPhase.Of(99, true, true));
        Run(world, 60);
        var watch = System.Diagnostics.Stopwatch.StartNew();
        Run(world, 240);
        var line = $"PARTICLE_COST 16 ropes x 48{(sheet ? " and one 16 x 16 sheet" : "")}, 4 substeps, {bodies} bod{(bodies == 1 ? "y" : "ies")}: {watch.Elapsed.TotalMilliseconds / 240:F3} ms per tick";
        Console.Error.WriteLine(line);
        Reading(line);
        Assert.True(watch.Elapsed.TotalMilliseconds > 0);
    }

    /// <summary>The 16 x 16 sheet cost scene with one body, plus <paramref name="fluid"/> fluid particles (r 1.25 cm) of XSPH viscosity <paramref name="viscosity"/> poured on the sheet; ms per tick at 4 substeps.</summary>
    private static double PourCost(int fluid, float viscosity = 0f, int sensors = 0)
    {
        using var world = new Box3DSoaWorld(64, gravityY: -G, particles: new Box3DSoaParticleCapacities(16 * 48 + 256 + fluid, 16 * 47 + 480, 16 * 46 + 448, 16, 600, 2, fluid, 0, sensors));
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f));
        for (var s = 0; s < sensors; s++) world.CreateSensors([(0, (0.35f + 0.1f * s, 1.8f, 0.15f), 0.15f)]); // body-frame y 1.8 is world 1.3: over the block
        world.SetMaterial(0, 0f, 0, viscosity);
        for (var c = 0; c < 16; c++)
            Graph.Rope(world, Line((0f, 0.5f + 0.03f * c, 0.02f * (c % 4)), (Spacing, 0f, 0f), 48), ParticleMass, ParticleRadius, Box3DSoaPhase.Of((uint)(c + 1), true, true));
        Graph.Sheet(world, (0.2f, 1.2f, 0f), (0.03f, 0f, 0f), (0f, 0f, 0.03f), 16, 16, ParticleMass, ParticleRadius, Box3DSoaPhase.Of(99, true, true));
        if (fluid > 0)
        {
            var nx = 10;
            var ny = fluid / (nx * nx);
            if (fluid == 512) { nx = 8; ny = 8; }
            Graph.Block(world, (0.3f, 1.3f, 0.05f), nx, ny, fluid / (nx * ny), ParticleRadius, 0.015625f, Box3DSoaPhase.Of(200, fluid: true));
        }
        Run(world, 60);
        var watch = System.Diagnostics.Stopwatch.StartNew();
        Run(world, 240);
        return watch.Elapsed.TotalMilliseconds / 240;
    }

    /// <summary>Private bytes the world takes for <paramref name="particles"/> particle lanes and a fluid capacity, as the process counts them (the native block is one calloc).</summary>
    private static long WorldBytes(int particles, int fluid)
    {
        GC.Collect();
        var before = System.Diagnostics.Process.GetCurrentProcess().PrivateMemorySize64;
        using var world = new Box3DSoaWorld(4, particles: new Box3DSoaParticleCapacities(particles, 0, 0, 0, 0, 1, fluid));
        GC.Collect();
        return System.Diagnostics.Process.GetCurrentProcess().PrivateMemorySize64 - before;
    }

    [Theory]
    [InlineData(512)]
    [InlineData(2000)]
    public void Fluid_cost_line(int fluid)
    {
        // Target at 512: an increment of at most 1.0 ms per tick over the same scene without fluid, at XSPH viscosity 0 and 0.01.
        const int lanes = 200000;
        var perLane = (double)WorldBytes(lanes, 0) / lanes;
        var perFluid = (double)(WorldBytes(lanes, lanes) - WorldBytes(lanes, 0)) / lanes;
        var memory = $"PARTICLE_COST bytes: {perLane:F0} per particle lane at fluid capacity 0 (the whole block; the fluid copies, nbr_start and nbr_count are 7 x 4 = 28 of it, where h1 carved 388 in every world), {perFluid:F0} more per fluid capacity (the neighbour arena, 96 entries)";
        Console.Error.WriteLine(memory);
        Reading(memory);
        // The one-run difference moves 0.3 ms from run to run, so the line is paired: the scene without fluid and the scene with it
        // run back to back (the order alternating, so a drifting machine favours neither), and the increment is the median of the pairs'
        // differences with its interquartile range. Printed, not asserted.
        const int pairs = 9;
        foreach (var viscosity in new[] { 0f, 0.01f })
        {
            var increments = new double[pairs];
            var withs = new double[pairs];
            var withouts = new double[pairs];
            for (var k = 0; k < pairs; k++)
            {
                if (k % 2 == 0) { withouts[k] = PourCost(0); withs[k] = PourCost(fluid, viscosity); }
                else { withs[k] = PourCost(fluid, viscosity); withouts[k] = PourCost(0); }
                increments[k] = withs[k] - withouts[k];
            }
            Array.Sort(increments);
            Array.Sort(withs);
            Array.Sort(withouts);
            var line = $"PARTICLE_COST {fluid} fluid particles r 1.25 cm viscosity {viscosity} poured on the 16 x 16 sheet scene, 4 substeps, 1 body, {pairs} interleaved pairs: median {withs[pairs / 2]:F3} ms per tick with, {withouts[pairs / 2]:F3} without; fluid increment median {increments[pairs / 2]:F3} ms per tick, IQR {increments[1] / 2 + increments[2] / 2:F3} to {increments[6] / 2 + increments[7] / 2:F3} (min {increments[0]:F3}, max {increments[pairs - 1]:F3})";
            Console.Error.WriteLine(line);
            Reading(line);
            Assert.True(withs[pairs / 2] > 0);
        }
        // Four liquid sensors over the pour, paired the same way against the same scene without them (same binary, the order alternating): the sensors' increment.
        {
            var increments = new double[pairs];
            var withs = new double[pairs];
            var withouts = new double[pairs];
            for (var k = 0; k < pairs; k++)
            {
                if (k % 2 == 0) { withouts[k] = PourCost(fluid); withs[k] = PourCost(fluid, 0f, 4); }
                else { withs[k] = PourCost(fluid, 0f, 4); withouts[k] = PourCost(fluid); }
                increments[k] = withs[k] - withouts[k];
            }
            Array.Sort(increments);
            Array.Sort(withs);
            Array.Sort(withouts);
            var line = $"PARTICLE_COST {fluid} fluid particles, 4 liquid sensors r 15 cm over the pour, 4 substeps, {pairs} interleaved pairs: median {withs[pairs / 2]:F3} ms per tick with, {withouts[pairs / 2]:F3} without; sensor increment median {increments[pairs / 2]:F3} ms per tick (min {increments[0]:F3}, max {increments[pairs - 1]:F3})";
            Console.Error.WriteLine(line);
            Reading(line);
            Assert.True(withs[pairs / 2] > 0);
        }
    }

    // ---- flesh of a limb tree (YMIR_ATTACH_FLAG_LIMB) --------------------------------------------------------

    private static Pos BodyPoint(Box3DSoaView v, int lane, Pos local)
    {
        float qx = v.RotationX[lane], qy = v.RotationY[lane], qz = v.RotationZ[lane], qw = v.RotationW[lane];
        var tx = 2f * (qy * local.Z - qz * local.Y); var ty = 2f * (qz * local.X - qx * local.Z); var tz = 2f * (qx * local.Y - qy * local.X);
        return (v.PositionX[lane] + local.X + qw * tx + (qy * tz - qz * ty),
                v.PositionY[lane] + local.Y + qw * ty + (qz * tx - qx * tz),
                v.PositionZ[lane] + local.Z + qw * tz + (qx * ty - qy * tx));
    }

    private static float Gap(Box3DSoaView v, int particle, Pos point) =>
        MathF.Sqrt(Sq(v.ParticleX[particle] - point.X) + Sq(v.ParticleY[particle] - point.Y) + Sq(v.ParticleZ[particle] - point.Z));

    // Two capsules of one limb tree, side by side with a gap between them (no overlap, no gravity): the upper arm's top cap centre
    // U = (0, 0.4, 0) and the forearm's bottom cap centre (0.2, 0.5, 0), both of radius 0.1. One particle sits 0.106 m from U toward
    // the forearm, inside the upper arm's contact reach (0.1 + 0.0125 = 0.1125) and attached to the forearm at its own place.
    private const float ArmRadius = 0.1f;
    private static readonly Pos ArmTop = (0f, 0.4f, 0f);
    private static readonly Pos FlankPoint = (0.0948f, 0.4474f, 0f);

    private static (Box3DSoaWorld World, int Upper, int Fore, Box3DSoaParticleHandle Flesh, Box3DSoaConstraintHandle Attachment, Pos Local) Arm(bool limb)
    {
        var world = Box3DSoaParticleTests.World(gravityY: 0f);
        var upper = world.AddCapsule(1, (0f, 0f, 0f), ArmRadius, 0.4f, 1000f);
        var fore = world.AddCapsule(2, (0.2f, 0.9f, 0f), ArmRadius, 0.4f, 1000f, parent: upper);
        var flesh = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc(FlankPoint, ParticleMass, ParticleRadius)], flesh);
        Pos local = (FlankPoint.X - 0.2f, FlankPoint.Y - 0.9f, FlankPoint.Z);
        var attachment = world.Attach(Box3DSoaAttachKind.Body, flesh[0], fore, local, limb: limb);
        return (world, upper, fore, flesh[0], attachment, local);
    }

    private static float ArmDeviation(bool limb)
    {
        var (world, _, fore, flesh, _, local) = Arm(limb);
        using (world)
        {
            var worst = 0f;
            for (var i = 0; i < 120; i++)
            {
                world.Step(Dt, Sub);
                worst = MathF.Max(worst, Gap(world.View, flesh.Lane, BodyPoint(world.View, fore, local)));
            }
            return worst;
        }
    }

    [Fact]
    public void Limb_flesh_ignores_the_bones_of_its_own_limb()
    {
        // The flesh particle is attached to the forearm and lies inside the upper arm's reach. With LIMB the contact skips the whole tree
        // (the upper arm is the forearm's parent: one root) and the particle stays on its attachment point; without it the upper arm
        // shoves the particle. Readings are the worst distance of the particle from its attachment point over 120 ticks.
        var withLimb = ArmDeviation(true);
        var without = ArmDeviation(false);
        Reading($"LIMB_FLESH worst deviation from the attachment point with LIMB {F(withLimb)} m, without {F(without)} m");
        Assert.True(withLimb < LimbBound, $"LIMB flesh sat {withLimb} m off its point");
        Assert.True(without > PushedFloor, $"the same particle without LIMB sat only {without} m off its point");
    }

    [Fact]
    public void Plain_attachment_skips_only_its_own_body_and_a_neighbour_still_collides()
    {
        // A two-particle rope at a sphere: the end A is tied inside the sphere without LIMB and stays on its point; B, one link out and
        // not attached, starts inside the sphere's reach and is pushed out of it. The held list names A's own body and nothing else.
        using var world = Box3DSoaParticleTests.World(gravityY: 0f);
        const float radius = 0.3f;
        var sphere = world.AddSphere(1, (0f, 0f, 0f), radius, 1000f);
        Pos a = (0f, 0.29f, 0f), b = (Spacing, 0.29f, 0f);
        var rope = Graph.Rope(world, [a, b], ParticleMass, ParticleRadius, bend: -1f);
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[0], sphere, a);
        var view = world.View;
        var worstA = 0f;
        for (var i = 0; i < 120; i++)
        {
            world.Step(Dt, Sub);
            worstA = MathF.Max(worstA, Gap(view, rope.Particles[0].Lane, BodyPoint(view, sphere, a)));
        }
        var lb = rope.Particles[1].Lane;
        var bFromCentre = MathF.Sqrt(Sq(view.ParticleX[lb] - view.PositionX[sphere]) + Sq(view.ParticleY[lb] - view.PositionY[sphere]) + Sq(view.ParticleZ[lb] - view.PositionZ[sphere]));
        Reading($"HELD_PLAIN attached end worst gap {F(worstA)} m; neighbour at {F(bFromCentre)} m from the sphere centre (surface plus radius {F(radius + ParticleRadius)})");
        Assert.True(worstA < HeldGap, $"the attached end sat {worstA} m off its point");
        var penetration = MathF.Max(0f, radius + ParticleRadius - bFromCentre);
        Assert.True(penetration < NeighbourDepth, $"the neighbour stayed {penetration} m inside the body's reach");
    }

    [Fact]
    public void Removing_the_limb_attachment_restores_contact_with_the_tree()
    {
        // The held list and limb body are derived at every attachment command: after Remove the particle meets the upper arm again.
        var (world, upper, _, flesh, attachment, _) = Arm(true);
        using (world)
        {
            Run(world, 30);
            var view = world.View;
            var cap = BodyPoint(view, upper, ArmTop);
            var before = Gap(view, flesh.Lane, cap);
            world.Remove([attachment]);
            Run(world, 30);
            cap = BodyPoint(view, upper, ArmTop);
            var after = Gap(view, flesh.Lane, cap);
            Reading($"LIMB_REMOVED distance from the upper arm's cap centre while attached {F(before)} m, 30 ticks after Remove {F(after)} m (surface plus radius {F(ArmRadius + ParticleRadius)})");
            Assert.True(before < ArmRadius + ParticleRadius, "the LIMB particle was not inside the upper arm's reach to begin with");
            var depth = MathF.Max(0f, ArmRadius + ParticleRadius - after);
            Assert.True(depth < RemovedDepth, $"after Remove the particle sat {depth} m inside the upper arm's reach");
        }
    }

    [Fact]
    public void Limb_attachment_to_a_world_point_is_refused()
    {
        using var world = Box3DSoaParticleTests.World(gravityY: 0f);
        var flesh = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0f, 1f, 0f), ParticleMass, ParticleRadius)], flesh);
        Assert.ThrowsAny<Exception>(() => world.Attach(Box3DSoaAttachKind.Static, flesh[0], point: (0f, 1f, 0f), limb: true));
    }

    private static float WorstGap(Box3DSoaWorld world, int body, Pos local, Box3DSoaParticleHandle particle, int steps = 120)
    {
        var worst = 0f;
        for (var i = 0; i < steps; i++)
        {
            world.Step(Dt, Sub);
            worst = MathF.Max(worst, Gap(world.View, particle.Lane, BodyPoint(world.View, body, local)));
        }
        return worst;
    }

    [Fact]
    public void Limb_flesh_still_meets_a_body_of_another_tree()
    {
        // The arm's flesh particle (LIMB, attached to the forearm) has a sphere of no limb tree beside it, inside its contact reach: the
        // sphere is not skipped and pushes the particle off its point. Readings are the worst distance from the point over 120 ticks.
        using var world = Box3DSoaParticleTests.World(gravityY: 0f);
        var upper = world.AddCapsule(1, (0f, 0f, 0f), ArmRadius, 0.4f, 1000f);
        var fore = world.AddCapsule(2, (0.2f, 0.9f, 0f), ArmRadius, 0.4f, 1000f, parent: upper);
        world.AddSphere(3, (0.35f, 0.9f, 0f), 0.04f, 1000f);
        Pos at = (0.306f, 0.9f, 0f);
        var flesh = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc(at, ParticleMass, ParticleRadius)], flesh);
        Pos local = (at.X - 0.2f, at.Y - 0.9f, at.Z);
        world.Attach(Box3DSoaAttachKind.Body, flesh[0], fore, local, limb: true);
        var worst = WorstGap(world, fore, local, flesh[0]);
        Reading($"LIMB_FOREIGN worst deviation from the attachment point beside a foreign sphere {F(worst)} m");
        Assert.True(worst > ForeignFloor, $"the LIMB particle sat only {worst} m off its point beside a foreign body");
    }

    [Fact]
    public void Limb_flesh_ignores_every_bone_up_its_chain_not_only_the_parent()
    {
        // Three capsules chained grandparent -> parent -> child, spaced 0.5 m. The particle is attached to the child and lies inside the
        // grandparent's reach: the limb-tree root is two hops from the child, and a walk that stopped at the parent would not skip it.
        using var world = Box3DSoaParticleTests.World(gravityY: 0f);
        var grand = world.AddCapsule(1, (0f, 0f, 0f), 0.1f, 0.4f, 1000f);
        var parent = world.AddCapsule(2, (0.5f, 0f, 0f), 0.1f, 0.4f, 1000f, parent: grand);
        var child = world.AddCapsule(3, (1.0f, 0f, 0f), 0.1f, 0.4f, 1000f, parent: parent);
        Pos at = (0.106f, 0f, 0f);
        var flesh = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc(at, ParticleMass, ParticleRadius)], flesh);
        Pos local = (at.X - 1.0f, at.Y, at.Z);
        world.Attach(Box3DSoaAttachKind.Body, flesh[0], child, local, limb: true);
        var worst = WorstGap(world, child, local, flesh[0]);
        Reading($"LIMB_CHAIN worst deviation from the attachment point, grandparent in reach {F(worst)} m");
        Assert.True(worst < ChainBound, $"the LIMB particle sat {worst} m off its point beside its grandparent");
    }

    [Fact]
    public void A_particle_created_after_an_attachment_does_not_skip_the_first_body()
    {
        // Creating particles rederives the held lists of the lanes that carry no attachment; a lane left at the zeroed arena's limb_body 0
        // would skip body 0's whole tree. The late particle sits inside sphere 0's reach and is pushed out of it.
        using var world = Box3DSoaParticleTests.World(gravityY: 0f);
        const float radius = 0.3f;
        var sphere = world.AddSphere(1, (0f, 0f, 0f), radius, 1000f);
        var first = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0f, 2f, 0f), ParticleMass, ParticleRadius)], first);
        world.Attach(Box3DSoaAttachKind.Body, first[0], sphere, (0f, 2f, 0f));
        var late = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new Box3DSoaParticleDesc((0.29f, 0f, 0f), ParticleMass, ParticleRadius)], late);
        Run(world, 60);
        var view = world.View;
        var fromCentre = MathF.Sqrt(Sq(view.ParticleX[late[0].Lane] - view.PositionX[sphere]) + Sq(view.ParticleY[late[0].Lane] - view.PositionY[sphere]) + Sq(view.ParticleZ[late[0].Lane] - view.PositionZ[sphere]));
        var depth = MathF.Max(0f, radius + ParticleRadius - fromCentre);
        Reading($"LIMB_LATE depth of the late particle inside the sphere's reach after 60 ticks {F(depth)} m");
        Assert.True(depth < LateDepth, $"the late particle stayed {depth} m inside body 0's reach");
    }

    [Fact]
    public void A_refused_attachment_leaves_the_held_lanes_as_they_were()
    {
        // A rope end beside the upper arm; the attach to the forearm asks for a tether, which the world has no capacity for, and is
        // refused. The refused command must leave the particle's held list and limb body untouched: the end then drifts exactly as in the
        // run that never tried. Readings are the end's distance from its start after 120 ticks, control and after the refusal.
        float Displacement(bool tryRefused)
        {
            using var world = Box3DSoaParticleTests.World(gravityY: 0f, tethers: 1, particles: 16, distances: 16, bends: 16);
            var upper = world.AddCapsule(1, (0f, 0f, 0f), ArmRadius, 0.4f, 1000f);
            var fore = world.AddCapsule(2, (0.2f, 0.9f, 0f), ArmRadius, 0.4f, 1000f, parent: upper);
            var f = FlankPoint;
            var rope = Graph.Rope(world, [f, (f.X, f.Y, f.Z + 0.03f), (f.X, f.Y, f.Z + 0.06f), (f.X, f.Y, f.Z + 0.09f)], ParticleMass, ParticleRadius, bend: -1f);
            if (tryRefused)
                Assert.ThrowsAny<Exception>(() => world.Attach(Box3DSoaAttachKind.Body, rope.Particles[0], fore, (f.X - 0.2f, f.Y - 0.9f, f.Z), limb: true, tether: true));
            Run(world, 120);
            var v = world.View;
            var l = rope.Particles[0].Lane;
            return MathF.Sqrt(Sq(v.ParticleX[l] - f.X) + Sq(v.ParticleY[l] - f.Y) + Sq(v.ParticleZ[l] - f.Z));
        }
        var control = Displacement(false);
        var refused = Displacement(true);
        Reading($"LIMB_REFUSED rope end displacement control {F(control)} m, after a refused LIMB+TETHER attach {F(refused)} m");
        Assert.True(MathF.Abs(refused - control) < RefusedBound, $"a refused attach moved the end {MathF.Abs(refused - control)} m from the control run");
    }

    [Fact]
    public void Limb_attachments_across_two_trees_skip_only_the_lowest_attachments_tree()
    {
        // The documented limit: a particle's LIMB attachments to bodies of two trees skip only the tree of its lowest live LIMB attachment
        // (the first made); the other tree's bones collide, except the bodies the particle is attached to. Two limbs, A (a1 -> x) and
        // B (a2 -> y); one particle per world is attached to x first and then y, and sits at its own place beside a1 or beside a2.
        static float Worst(bool besideSecondTree)
        {
            using var world = Box3DSoaParticleTests.World(gravityY: 0f);
            world.AddCapsule(1, (0f, 0f, 0f), 0.1f, 0.4f, 1000f);
            var x = world.AddCapsule(2, (0f, 2f, 0f), 0.1f, 0.4f, 1000f, parent: 0);
            world.AddCapsule(3, (1f, 0f, 0f), 0.1f, 0.4f, 1000f);
            var y = world.AddCapsule(4, (1f, 2f, 0f), 0.1f, 0.4f, 1000f, parent: 2);
            Pos at = besideSecondTree ? (1.106f, 0f, 0f) : (0.106f, 0f, 0f);
            var flesh = new Box3DSoaParticleHandle[1];
            world.CreateParticles([new Box3DSoaParticleDesc(at, ParticleMass, ParticleRadius)], flesh);
            Pos onX = (at.X, at.Y - 2f, at.Z);
            world.Attach(Box3DSoaAttachKind.Body, flesh[0], x, onX, limb: true);
            world.Attach(Box3DSoaAttachKind.Body, flesh[0], y, (at.X - 1f, at.Y - 2f, at.Z), limb: true);
            return WorstGap(world, x, onX, flesh[0]);
        }
        var skipped = Worst(false);
        var other = Worst(true);
        Reading($"LIMB_TWO_TREES worst deviation beside the skipped tree's bone {F(skipped)} m, beside the other tree's bone {F(other)} m");
        Assert.True(skipped < TwoTreesBound, $"the skipped tree's bone moved the particle {skipped} m");
        Assert.True(other > TwoTreesFloor, $"the other tree's bone moved the particle only {other} m");
    }

    [Fact]
    public void Held_body_contact_cost_reading()
    {
        // 600 particles, each attached (plain) to one of 19 capsules in a row and so skipping that body, every particle in reach of its
        // neighbours' bodies: the tick's cost, median of repeated runs of one scene. Printed, no bound.
        const int bodies = 19, count = 600, reps = 7;
        static double OneRun()
        {
            using var world = Box3DSoaParticleTests.World(bodies: bodies + 1, particles: count, distances: 1, bends: 1, attachments: count, tethers: 1, gravityY: 0f);
            var lanes = new int[bodies];
            for (var k = 0; k < bodies; k++) lanes[k] = world.AddCapsule((ulong)(k + 1), (0.12f * k, 0f, 0f), 0.05f, 0.15f, 1000f);
            var desc = new Box3DSoaParticleDesc[count];
            for (var i = 0; i < count; i++)
            {
                var k = i % bodies;
                var ang = 6.2831853f * (i / bodies) / (count / bodies + 1);
                desc[i] = new Box3DSoaParticleDesc((0.12f * k + 0.04f * MathF.Cos(ang), 0.2f * MathF.Sin(ang), 0.04f * MathF.Sin(ang)), ParticleMass, ParticleRadius);
            }
            var handles = new Box3DSoaParticleHandle[count];
            world.CreateParticles(desc, handles);
            for (var i = 0; i < count; i++)
                world.Attach(Box3DSoaAttachKind.Body, handles[i], lanes[i % bodies], (desc[i].X - 0.12f * (i % bodies), desc[i].Y, desc[i].Z));
            Run(world, 10);
            var sw = System.Diagnostics.Stopwatch.StartNew();
            Run(world, 60);
            return sw.Elapsed.TotalMilliseconds / 60.0;
        }
        var runs = new double[reps];
        for (var k = 0; k < reps; k++) runs[k] = OneRun();
        Array.Sort(runs);
        var line = $"HELD_COST {count} particles attached to {bodies} capsules, 4 substeps: median {F(runs[reps / 2])} ms per tick (min {F(runs[0])}, max {F(runs[reps - 1])})";
        Console.Error.WriteLine(line);
        Reading(line);
        Assert.True(runs[reps / 2] > 0);
    }

    // Bounds, by lodur:ruling:deterministic-bound-margin and deterministic-bound-zero-reading: the geometric mean of the shipped worst and the
    // least claimed mutant (a shipped zero counts as half a unit, a mutant zero as one unit; the unit is 1 um). The mutants, each a one-line
    // change to ymir_particles.c, and their readings (metres):
    //   Ml  the LIMB root compare deleted            Mr  the compare against the attached body's lane instead of its root
    //   Mx  the root skip applied to every particle  Mb  held_by true for any particle with a list
    //   Mh  the held list read as empty              Mp  limb_body not reset when the list is rederived
    // LimbBound: shipped worst 0 (-> 5e-7), least claimed mutant Ml = Mr 6.2855e-3, separation 12571; bound sqrt(5e-7 x 6.2855e-3).
    private const float LimbBound = 5.6e-5f;
    // PushedFloor: shipped (no LIMB) 6.2855e-3, least claimed mutant Mx = Mb 0 (-> 1e-6), separation 6285; floor sqrt(6.2855e-3 x 1e-6).
    private const float PushedFloor = 7.9e-5f;
    // The attached end's worst gap while its neighbour is pushed: shipped 4.66652e-3 (the rope link pulls it), least claimed mutant Mh 2.26468e-2,
    // separation 4.85; bound sqrt(4.66652e-3 x 2.26468e-2) = 1.028e-2. Held tests (b) sit in Plain_attachment_skips_only_its_own_body_*.
    private const float HeldGap = 1.028e-2f;
    // The neighbour's depth inside the sphere's reach at the end: shipped 0 (-> 5e-7), least claimed mutant Mx 2.0952e-2; bound sqrt(5e-7 x 2.0952e-2).
    private const float NeighbourDepth = 1.02e-4f;
    // The particle's depth inside the upper arm's reach 30 ticks after Remove: shipped 0 (-> 5e-7), least claimed mutant Mp 6.51e-3; bound sqrt(5e-7 x 6.51e-3).
    private const float RemovedDepth = 5.7e-5f;
    // Bounds of the s1 fix batch, by the same rules (the unit is 1 um; a shipped or mutant zero is half a unit or one unit). The mutants:
    //   Mg3 every body's limb-tree root set to 0     ML1 the root compare dropped, every body skipped once the particle has a limb body
    //   Mg2 the root walk stopped after one hop      Mi  the ISOLATED reset of limb_body deleted
    //   Mx  the held lists written before the tether-capacity check     Ml3 the highest LIMB attachment decides the skipped tree
    // ForeignFloor: shipped 3.52192e-3, least claimed mutant Mg3 = ML1 0 (-> 1e-6), separation 3522; floor sqrt(3.52192e-3 x 1e-6).
    private const float ForeignFloor = 5.9e-5f;
    // ChainBound: shipped 0 (-> 5e-7), Mg2 6.41614e-3, separation 12832; bound sqrt(5e-7 x 6.41614e-3).
    private const float ChainBound = 5.7e-5f;
    // LateDepth: shipped 0 (-> 5e-7), Mi 0.0225, separation 45000; bound sqrt(5e-7 x 0.0225).
    private const float LateDepth = 1.06e-4f;
    // RefusedBound, on the gap between the run after a refusal and the control run: shipped 0 (-> 5e-7), Mx 0.118058 (0.343645 - 0.225587),
    // separation 236116; bound sqrt(5e-7 x 0.118058).
    private const float RefusedBound = 2.43e-4f;
    // TwoTreesBound: shipped 7.5e-9 (-> 5e-7), least claimed mutant Ml3 6.76838e-3, separation 13537; bound sqrt(5e-7 x 6.76838e-3).
    private const float TwoTreesBound = 5.8e-5f;
    // TwoTreesFloor: shipped 5.90668e-3, least claimed mutant Ml3 = Mg3 = ML1 0 (-> 1e-6), separation 5907; floor sqrt(5.90668e-3 x 1e-6).
    private const float TwoTreesFloor = 7.7e-5f;
}
