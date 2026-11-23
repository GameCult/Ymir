using System.Globalization;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

/// <summary>
/// Chains and bodies coupled inside Box3D's substep (docs/soa-world.md, particle section): a held weight stays held, momentum and
/// energy are kept through a throw and a snap, the sensors read what the structure carries, and a release takes effect in its step.
/// Every reading the tests judge is also appended to the file named by YMIR_READINGS when that variable is set.
/// </summary>
public sealed class Box3DSoaCouplingTests
{
    private const float Dt = 1f / 120f;
    private const int Sub = 4;
    private const float G = 10f;
    private const float ParticleMass = 0.02f;
    private const float ParticleRadius = 0.0125f;
    private const float Spacing = 0.03f;

    private static void Reading(string line)
    {
        var path = Environment.GetEnvironmentVariable("YMIR_READINGS");
        if (!string.IsNullOrEmpty(path)) File.AppendAllText(path, line + "\n");
    }

    private static string F(double x) => x.ToString("G6", CultureInfo.InvariantCulture);

    private static (float X, float Y, float Z)[] Line((float X, float Y, float Z) from, (float X, float Y, float Z) step, int count) =>
        Box3DSoaParticleTests.Line(from, step, count);

    private static float Dens(float mass, float radius) => mass / (4f / 3f * MathF.PI * radius * radius * radius);

    private static float Sq(float x) => x * x;

    private static Box3DSoaWorld World(float gravityX = 0f) =>
        new(8, gravityX: gravityX, gravityY: -G, jointCapacity: 2, particleCapacity: 256, chainCapacity: 8, attachmentCapacity: 8);

    private static void Run(Box3DSoaWorld world, int steps)
    {
        for (var i = 0; i < steps; i++) world.Step(Dt, Sub);
    }

    /// <summary>A chain of 48 particles hanging from a static point with a sphere of <paramref name="mass"/> tied under its end.</summary>
    private static (int Sphere, Box3DSoaAttachmentHandle Top, Box3DSoaAttachmentHandle Bottom, Box3DSoaChainHandle Chain) Hung(
        Box3DSoaWorld world, float mass, float radius = 0.1f, (float X, float Y, float Z) velocity = default)
    {
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var chain = world.CreateChain(1, positions, ParticleMass, ParticleRadius);
        var top = world.Attach(Box3DSoaAttachKind.Static, 0, point: positions[0]);
        var end = positions[47];
        var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Dens(mass, radius), velocity: velocity);
        var bottom = world.Attach(Box3DSoaAttachKind.Body, 47, sphere, (0f, radius, 0f));
        return (sphere, top, bottom, chain);
    }

    /// <summary>Kinetic and potential energy of the chain's particles and of the listed spheres (translation and spin), gravity along -y.</summary>
    private static double Energy(Box3DSoaView v, params (int Lane, float Mass, float Radius)[] spheres)
    {
        double e = 0;
        for (var i = 0; i < v.ParticleCount; i++)
        {
            if (v.ParticleChain[i] < 0 || v.ParticleInvMass[i] <= 0f) continue;
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

    private static double ChainMomentumX(Box3DSoaView v)
    {
        double p = 0;
        for (var i = 0; i < v.ParticleCount; i++)
            if (v.ParticleChain[i] >= 0 && v.ParticleInvMass[i] > 0f) p += v.ParticleVx[i] / v.ParticleInvMass[i];
        return p;
    }

    // ---- holding ---------------------------------------------------------------------------------------------

    [Theory]
    [InlineData(1f)]
    [InlineData(5f)]
    [InlineData(20f)]
    [InlineData(70f)]
    public void Top_link_tension_reads_the_load(float mass)
    {
        using var world = World();
        var (sphere, top, bottom, _) = Hung(world, mass);
        var view = world.View;
        Run(world, 240);
        double t0 = 0, t10 = 0, t46 = 0, topForce = 0, bodyForce = 0;
        for (var i = 0; i < 60; i++)
        {
            world.Step(Dt, Sub);
            t0 += view.ParticleLinkTension[0]; t10 += view.ParticleLinkTension[10]; t46 += view.ParticleLinkTension[46];
            topForce += view.AttachmentForceY[top.Lane]; bodyForce += view.AttachmentForceY[bottom.Lane];
        }
        t0 /= 60; t10 /= 60; t46 /= 60; topForce /= 60; bodyForce /= 60;
        double Below(int link) => (mass + (47 - link) * ParticleMass) * G; // the body and every particle past the link
        Reading($"TENSION M={mass} link0 {F(t0)}/{F(Below(0))} link10 {F(t10)}/{F(Below(10))} link46 {F(t46)}/{F(Below(46))} body {F(bodyForce)}/{F(-mass * G)} static {F(topForce)}/{F((mass + 48 * ParticleMass) * G)}");
        Assert.True(Math.Abs(t0 - Below(0)) < 0.01 * Below(0), $"link 0 reads {t0} N, load {Below(0)} N");
        Assert.True(Math.Abs(t10 - Below(10)) < 0.01 * Below(10), $"link 10 reads {t10} N, load {Below(10)} N");
        Assert.True(Math.Abs(t46 - Below(46)) < 0.01 * Below(46), $"link 46 reads {t46} N, load {Below(46)} N");
        Assert.True(Math.Abs(bodyForce + mass * G) < 0.005 * mass * G, $"the body's pull on the chain reads {bodyForce} N, weight {mass * G} N");
        var all = (mass + 48 * ParticleMass) * G;
        Assert.True(Math.Abs(topForce - all) < 0.01 * all, $"the static point's support reads {topForce} N, load {all} N");
        _ = sphere;
    }

    [Fact]
    public void Attached_end_stays_on_its_point()
    {
        // A 1 kg sphere hung under the chain: the end particle is tied to the sphere's top and the sphere is skipped by that
        // particle's contacts. A contact that pushed the end particle off its own body would read as a gap.
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
            // the body point (0, r, 0) rotated by q
            var tx = 2f * (qy * 0f - qz * radius); var ty = 2f * (qz * 0f - qx * 0f); var tz = 2f * (qx * radius - qy * 0f);
            var px = view.PositionX[sphere] + qw * tx + (qy * tz - qz * ty);
            var py = view.PositionY[sphere] + radius + qw * ty + (qz * tx - qx * tz);
            var pz = view.PositionZ[sphere] + qw * tz + (qx * ty - qy * tx);
            var gap = MathF.Sqrt(Sq(view.ParticleX[47] - px) + Sq(view.ParticleY[47] - py) + Sq(view.ParticleZ[47] - pz));
            worst = MathF.Max(worst, gap);
        }
        Reading($"ATTACHED_END worst gap {F(worst)} m");
        Assert.True(worst < 1e-4f, $"the end particle sat {worst} m from its body point");
    }

    [Fact]
    public void Tethers_hold_the_rest_path()
    {
        // A 70 kg particle at the end of a 48-particle chain from a static point: no particle may be farther from the
        // attachment than the rest path to it, however the load is routed.
        using var world = World();
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var masses = new float[48];
        Array.Fill(masses, ParticleMass);
        masses[47] = 70f;
        world.CreateChain(1, positions, masses, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, 0, point: positions[0]);
        var view = world.View;
        var worst = 0f;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt, Sub);
            for (var k = 1; k < 48; k++)
            {
                var d = MathF.Sqrt(Sq(view.ParticleX[k] - positions[0].X) + Sq(view.ParticleY[k] - positions[0].Y) + Sq(view.ParticleZ[k] - positions[0].Z));
                worst = MathF.Max(worst, d - k * Spacing);
            }
        }
        Reading($"TETHER_BOUND worst overshoot {F(worst)} m");
        Assert.True(worst < 1e-5f, $"a particle sits {worst} m past its rest path");
    }

    // ---- momentum and energy ---------------------------------------------------------------------------------

    [Theory]
    [InlineData(5f)]
    [InlineData(70f)]
    public void Free_chain_throw_keeps_momentum(float mass)
    {
        // A sphere thrown at 5 m/s with a 48 x 20 g chain tied behind it, the chain at rest: gravity acts on y only, so x momentum is
        // M x 5 for good, and the chain may only slow the body by picking up speed itself.
        const float radius = 0.1f;
        using var world = World();
        var positions = Line((0f, 10f, 0f), (-Spacing, 0f, 0f), 48);
        world.CreateChain(1, positions, ParticleMass, ParticleRadius);
        var sphere = world.AddSphere(10, (radius, 10f, 0f), radius, Dens(mass, radius), velocity: (5f, 0f, 0f));
        world.Attach(Box3DSoaAttachKind.Body, 0, sphere, (-radius, 0f, 0f));
        var view = world.View;
        var floor = 0.95 * mass * 5.0 / (mass + 48 * ParticleMass);
        double e0 = 0, worstMomentum = 0, worstEnergy = double.MinValue, firstVx = 0;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt, Sub);
            var momentum = mass * view.VelocityX[sphere] + ChainMomentumX(view);
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
        var positions = Line((0f, 10f - radius, 0f), (0f, -Spacing, 0f), 48); // hangs below the body
        world.CreateChain(1, positions, ParticleMass, ParticleRadius);
        var sphere = world.AddSphere(10, (0f, 10f, 0f), radius, Dens(mass, radius), velocity: (0f, 5f, 0f));
        world.Attach(Box3DSoaAttachKind.Body, 0, sphere, (0f, -radius, 0f));
        var view = world.View;
        var apex = 10f;
        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt, Sub);
            apex = MathF.Max(apex, view.PositionY[sphere]);
        }
        // The body alone would rise 1.25 m; the chain it picks up from rest takes the momentum it needs.
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
        // A 45 degree swing for 12 s, the weight as a tied body and as a heavy particle at the chain's end (the control the
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
                world.CreateChain(1, positions, masses, ParticleRadius);
                world.Attach(Box3DSoaAttachKind.Static, 0, point: positions[0]);
                (int, float, float)[] bodies = [];
                if (asBody)
                {
                    var end = positions[47];
                    var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Dens(mass, radius));
                    world.Attach(Box3DSoaAttachKind.Body, 47, sphere, (0f, radius, 0f));
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
        world.CreateChain(1, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, 0, point: positions[0]);
        var end = positions[47];
        var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Dens(mass, radius));
        world.Attach(Box3DSoaAttachKind.Body, 47, sphere, (0f, radius, 0f));
        var inertia = 0.4f * mass * radius * radius;
        (float X, float Y, float Z) centre = (end.X, end.Y - radius, end.Z);
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
    public void Detach_releases_in_step(bool destroyChain)
    {
        using var world = World();
        var (sphere, _, bottom, chain) = Hung(world, 70f);
        var view = world.View;
        Run(world, 3 * 120);
        if (destroyChain) world.DestroyChain(chain); else world.Detach(bottom);
        var worst = 0f;
        var impulse = float.MinValue;
        for (var k = 1; k <= 4; k++)
        {
            world.Step(Dt, Sub);
            worst = MathF.Max(worst, MathF.Abs(view.VelocityY[sphere] - (-G * Dt * k)));
            impulse = MathF.Max(impulse, view.ParticleImpulseY[sphere]);
        }
        Reading($"RELEASE destroyChain={destroyChain} worst vy error {F(worst)} m/s, greatest impulse Y {F(impulse)}");
        Assert.True(worst < 0.005f, $"the body is {worst} m/s off free fall in the step after the command");
        Assert.True(impulse <= 0f, $"the released chain still pushed the body up by {impulse} N s");
    }

    [Fact]
    public void Particle_impulse_is_what_Box3D_applied()
    {
        // M (v_after - v_before) - M g dt is, per axis, the impulse the chain gave the body this tick: the lane reads exactly that.
        const float radius = 0.1f;
        foreach (var thrown in new[] { false, true })
        {
            const float mass = 70f;
            using var world = World();
            int sphere;
            if (!thrown) sphere = Hung(world, mass).Sphere;
            else
            {
                var positions = Line((0f, 10f, 0f), (-Spacing, 0f, 0f), 48);
                world.CreateChain(1, positions, ParticleMass, ParticleRadius);
                sphere = world.AddSphere(10, (radius, 10f, 0f), radius, Dens(mass, radius), velocity: (5f, 0f, 0f));
                world.Attach(Box3DSoaAttachKind.Body, 0, sphere, (-radius, 0f, 0f));
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

    // ---- bodies and chains without the pull ----------------------------------------------------------------------

    [Theory]
    [InlineData(0.6f, true)]
    [InlineData(0f, false)]
    public void Chain_friction_holds_on_a_slope(float friction, bool holds)
    {
        // Gravity tilted 20 degrees on a chain lying on a static box: friction 0.6 (above tan 20) holds it, friction 0 lets it go.
        var tilt = 20f * MathF.PI / 180f;
        using var world = World(gravityX: G * MathF.Sin(tilt));
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f), friction: 0.6f);
        world.CreateChain(1, Line((0f, ParticleRadius, 0f), (0f, 0f, Spacing), 12), ParticleMass, ParticleRadius, friction: friction);
        var view = world.View;
        Run(world, 2 * 120);
        var slid = MathF.Abs(view.ParticleX[6]);
        Reading($"SLOPE mu={friction} slid {F(slid)} m in 2 s");
        if (holds) Assert.True(slid < 0.01f, $"friction {friction} let the chain slide {slid} m");
        else Assert.True(slid > 1f, $"friction 0 slid only {slid} m");
    }

    [Fact]
    public void Particles_step_with_no_awake_body()
    {
        // Box3D returns early from a world with nothing awake; the particle substeps run all the same.
        using var world = World();
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f));
        world.CreateChain(1, Line((0f, 1f, 0f), (0f, 0f, Spacing), 8), ParticleMass, ParticleRadius);
        var view = world.View;
        Run(world, 24); // 0.2 s
        var fell = 1f - view.ParticleY[3];
        Reading($"NO_AWAKE fell {F(fell)} m in 0.2 s (g t^2 / 2 = 0.2)");
        Assert.True(MathF.Abs(fell - 0.2f) < 0.02f * 0.2f, $"the chain fell {fell} m, expected 0.2 m");
        Run(world, 4 * 120);
        Assert.Equal(ParticleRadius, view.ParticleY[3], 1e-3f);
        Assert.True(MathF.Abs(view.ParticleVy[3]) < 1e-3f, "the chain rests on the box");
    }

    // ---- cost ---------------------------------------------------------------------------------------------------

    [Theory]
    [InlineData(1)]
    [InlineData(40)]
    public void Particle_cost_line(int bodies)
    {
        using var world = new Box3DSoaWorld(64, gravityY: -G, particleCapacity: 16 * 48, chainCapacity: 16, attachmentCapacity: 16);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f));
        for (var b = 1; b < bodies; b++)
            world.AddSphere((ulong)(100 + b), (0.2f * (b % 8), 0.3f + 0.25f * (b / 8), 0.5f), 0.08f, 800f);
        for (var c = 0; c < 16; c++) // a pile: chains zig-zag so particles meet their neighbours' links
            world.CreateChain((ulong)(10 + c), Line((0f, 0.5f + 0.03f * c, 0.02f * (c % 4)), (Spacing, 0f, 0f), 48), ParticleMass, ParticleRadius);
        Run(world, 60);
        var watch = System.Diagnostics.Stopwatch.StartNew();
        Run(world, 240);
        var line = $"PARTICLE_COST 16 chains x 48 particles, 4 substeps, {bodies} bod{(bodies == 1 ? "y" : "ies")}: {watch.Elapsed.TotalMilliseconds / 240:F3} ms per tick";
        Console.Error.WriteLine(line);
        Reading(line);
        Assert.True(watch.Elapsed.TotalMilliseconds > 0);
    }
}
