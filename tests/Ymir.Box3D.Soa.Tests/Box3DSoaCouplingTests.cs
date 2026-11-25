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
    private static double PourCost(int fluid, float viscosity = 0f)
    {
        using var world = new Box3DSoaWorld(64, gravityY: -G, particles: new Box3DSoaParticleCapacities(16 * 48 + 256 + fluid, 16 * 47 + 480, 16 * 46 + 448, 16, 600, 2, fluid));
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f));
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
    }
}
