using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>The particle graph: the contract of docs/soa-world.md, particle section.</summary>
public sealed class Box3DSoaParticleTests
{
    internal const float Dt = 1f / 120f;
    internal const int Sub = 4;
    internal const float Spacing = 0.03f;
    internal const float ParticleMass = 0.02f;
    internal const float ParticleRadius = 0.0125f;

    private static ulong Fnv(ulong h, ReadOnlySpan<byte> bytes)
    {
        foreach (var b in bytes) { h ^= b; h *= 1099511628211ul; }
        return h;
    }

    private static ulong HashBodyLanes(Box3DSoaView v)
    {
        var h = 1469598103934665603ul;
        void F(ReadOnlySpan<float> lane) => h = Fnv(h, MemoryMarshal.AsBytes(lane));
        F(v.PositionX); F(v.PositionY); F(v.PositionZ);
        F(v.RotationX); F(v.RotationY); F(v.RotationZ); F(v.RotationW);
        F(v.VelocityX); F(v.VelocityY); F(v.VelocityZ);
        F(v.AngularVelocityX); F(v.AngularVelocityY); F(v.AngularVelocityZ);
        F(v.ContactImpulse);
        h = Fnv(h, MemoryMarshal.AsBytes(v.ContactCount));
        return h;
    }

    /// <summary>The ABI 8 determinism scene (a slab and 25 dropped spheres), created without particle capacities.</summary>
    private static Box3DSoaWorld Abi8Scene()
    {
        var world = new Box3DSoaWorld(capacity: 64);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (50f, 0.5f, 50f));
        ulong id = 100;
        for (var x = 0; x < 5; x++)
            for (var z = 0; z < 5; z++)
                world.AddSphere(id++, (x * 1.5f - 3f, 3f + 0.37f * ((x * 5 + z) % 7), z * 1.5f - 3f), radius: 0.5f, density: 1f,
                    velocity: (0.1f * z, 0f, -0.05f * x));
        return world;
    }

    // Recorded on the commit before any particle code existed (ABI 8, 5104b17), one hash per 50 steps at 1/60 s.
    private static readonly ulong[] Abi8Hashes = [13041258221528527995ul, 10457196283436582934ul, 7549714013947433797ul, 13409007737149865513ul, 2368270135655444881ul, 5732644473329431104ul];

    [Fact]
    public void Zero_capacity_world_is_ABI8_identical()
    {
        using var world = Abi8Scene();
        var got = new ulong[6];
        for (var c = 0; c < 6; c++)
        {
            for (var i = 0; i < 50; i++) world.Step(1f / 60f);
            got[c] = HashBodyLanes(world.View);
        }
        Assert.True(Abi8Hashes.AsSpan().SequenceEqual(got), string.Join(",", got.Select(g => g + "ul")));
    }

    // ---- scenes ---------------------------------------------------------------------------------------------

    internal static Box3DSoaWorld World(
        int bodies = 8, int particles = 256, int distances = 600, int bends = 600, int attachments = 8, int tethers = 600, int joints = 2,
        float gravityX = 0f, float gravityY = -10f, float gravityZ = 0f, int fluid = 0) =>
        new(bodies, gravityX: gravityX, gravityY: gravityY, gravityZ: gravityZ, jointCapacity: joints,
            particles: new Box3DSoaParticleCapacities(particles, distances, bends, attachments, tethers, 2, fluid));

    internal static void Run(Box3DSoaWorld world, int steps)
    {
        for (var i = 0; i < steps; i++) world.Step(Dt, Sub);
    }

    private static Pos[] Line(Pos from, Pos step, int count) => Graph.Line(from, step, count);
    private static float Sq(float x) => x * x;

    // ---- behaviour ------------------------------------------------------------------------------------------

    [Fact]
    public void Rope_hangs_near_rest_length()
    {
        using var world = World();
        var positions = Line((0f, 3f, 0f), (0f, -Spacing, 0f), 48);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        var view = world.View;
        var worst = 0f;
        for (var i = 0; i < 4 * 120; i++)
        {
            world.Step(Dt, Sub);
            worst = MathF.Max(worst, MathF.Abs(Graph.Stretch(view, rope)));
        }
        Assert.True(worst < 0.01f, $"stretch peaked at {worst}");
        Assert.Equal(3f - 47 * Spacing, view.ParticleY[47], 0.05f); // it hangs, not sags to the side or floats
        Assert.Equal(0f, view.ParticleX[47], 0.05f);
    }

    [Fact]
    public void Rope_holds_a_70kg_body()
    {
        using var world = World();
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        const float radius = 0.15f, mass = 70f;
        var end = positions[47];
        var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Graph.Dens(mass, radius));
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[47], sphere, (0f, radius, 0f), tether: true);
        var view = world.View;
        float worstStretch = 0f, worstSpeed = 0f, worstImpulse = 0f;
        for (var i = 0; i < 5 * 120; i++)
        {
            world.Step(Dt, Sub);
            if (i < 120) continue; // from 1 s to 5 s the body is still
            worstStretch = MathF.Max(worstStretch, MathF.Abs(Graph.Stretch(view, rope)));
            worstSpeed = MathF.Max(worstSpeed, MathF.Sqrt(Sq(view.VelocityX[sphere]) + Sq(view.VelocityY[sphere]) + Sq(view.VelocityZ[sphere])));
            worstImpulse = MathF.Max(worstImpulse, MathF.Abs(view.ParticleImpulseY[sphere] - mass * 10f * Dt));
        }
        Assert.True(worstStretch < 0.001f, $"stretch peaked at {worstStretch}");
        Assert.True(worstSpeed < 0.001f, $"the body moves at {worstSpeed} m/s");
        Assert.True(worstImpulse < 0.005f * mass * 10f * Dt, $"the impulse per tick strays {worstImpulse} N s from M g dt = {mass * 10f * Dt}");
        Assert.Equal(end.Y - radius, view.PositionY[sphere], 0.1f);
    }

    [Fact]
    public void Rope_drags_a_resting_box()
    {
        using var world = World();
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f), friction: 0.6f);
        world.SetMaterial(0, 0.3f);
        const float half = 0.15f;
        var count = 29;
        var positions = Line((0f, ParticleRadius, 0f), (Spacing, 0f, 0f), count);
        var tip = positions[count - 1];
        var box = world.AddBox(2, (tip.X + half, half, 0f), (half, half, half), 5f / (8f * half * half * half), friction: 0.6f);
        var anchor = world.AddAnchor(3, positions[0]);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[0], anchor, tether: true);
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[count - 1], box, (-half, ParticleRadius - half, 0f), tether: true);
        var view = world.View;
        var startX = view.PositionX[box];
        var peak = 0f;
        for (var i = 0; i < 2 * 120; i++)
        {
            world.SetAnchorTarget(anchor, (-(i + 1) * 0.5f / 120f, ParticleRadius, 0f), default, Dt);
            world.Step(Dt, Sub);
            peak = MathF.Max(peak, MathF.Abs(view.ParticleImpulseX[box]));
        }
        Assert.True(startX - view.PositionX[box] > 0.5f, $"the box moved {startX - view.PositionX[box]} m");
        Assert.True(peak > 0f, "particle_impulse reads the pull");
    }

    /// <summary>How deep particle <paramref name="i"/> sits inside body <paramref name="b"/>'s surface (negative: clear of it).</summary>
    private static float Penetration(Box3DSoaView v, int b, int i)
    {
        var px = v.ParticleX[i] - v.PositionX[b]; var py = v.ParticleY[i] - v.PositionY[b]; var pz = v.ParticleZ[i] - v.PositionZ[b];
        float qx = v.RotationX[b], qy = v.RotationY[b], qz = v.RotationZ[b], qw = v.RotationW[b];
        Pos Rot(float x, float y, float z, float s) // rotate by q (s = 1) or its inverse (s = -1)
        {
            var ux = qx * s; var uy = qy * s; var uz = qz * s;
            var tx = 2f * (uy * z - uz * y); var ty = 2f * (uz * x - ux * z); var tz = 2f * (ux * y - uy * x);
            return (x + qw * tx + (uy * tz - uz * ty), y + qw * ty + (uz * tx - ux * tz), z + qw * tz + (ux * ty - uy * tx));
        }
        var l = Rot(px, py, pz, -1f);
        float d;
        switch ((Box3DSoaKind)v.Kind[b])
        {
            case Box3DSoaKind.Sphere:
                d = MathF.Sqrt(Sq(l.X) + Sq(l.Y) + Sq(l.Z)) - v.Size0[b];
                break;
            case Box3DSoaKind.Capsule:
                var t = Math.Clamp(l.Y, -v.Size1[b], v.Size1[b]);
                d = MathF.Sqrt(Sq(l.X) + Sq(l.Y - t) + Sq(l.Z)) - v.Size0[b];
                break;
            default:
                float ax = MathF.Abs(l.X) - v.Size0[b], ay = MathF.Abs(l.Y) - v.Size1[b], az = MathF.Abs(l.Z) - v.Size2[b];
                d = MathF.Sqrt(Sq(MathF.Max(ax, 0f)) + Sq(MathF.Max(ay, 0f)) + Sq(MathF.Max(az, 0f))) + MathF.Min(MathF.Max(ax, MathF.Max(ay, az)), 0f);
                break;
        }
        return v.ParticleRadius[i] - d;
    }

    [Theory]
    [InlineData(Box3DSoaKind.Sphere)]
    [InlineData(Box3DSoaKind.Capsule)]
    [InlineData(Box3DSoaKind.Box)]
    [InlineData(Box3DSoaKind.StaticBox)]
    public void Bodies_push_particles_per_kind(Box3DSoaKind kind)
    {
        using var world = World();
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f), friction: 0.6f);
        world.SetMaterial(0, 0.3f);
        Graph.Rope(world, Line((0f, ParticleRadius, 0f), (Spacing, 0f, 0f), 12), ParticleMass, ParticleRadius);
        // The body starts half a particle radius into the rope's end and, if it moves, is pushed on into it.
        var x0 = -0.05f - 0.5f * ParticleRadius;
        var body = kind switch
        {
            Box3DSoaKind.Sphere => world.AddSphere(2, (x0, 0.05f, 0f), 0.05f, 500f),
            Box3DSoaKind.Capsule => world.AddCapsule(2, (x0 - 0.1f + 0.05f, 0.05f, 0f), 0.05f, 0.1f, 500f, rotation: (0f, 0f, 0.70710677f, 0.70710677f)),
            Box3DSoaKind.Box => world.AddBox(2, (x0, 0.05f, 0f), (0.05f, 0.05f, 0.05f), 500f),
            _ => world.AddStaticBox(2, (x0, 0.05f, 0f), (0.05f, 0.05f, 0.05f)),
        };
        var view = world.View;
        var worst = 0f;
        for (var step = 0; step < 120; step++)
        {
            if (kind != Box3DSoaKind.StaticBox) view.ForceX[body] = 6f; // pushes into the rope
            world.Step(Dt, Sub);
            if (step >= 60)
                for (var i = 0; i < 12; i++) worst = MathF.Max(worst, Penetration(view, body, i));
        }
        Assert.True(worst < 0.1f * ParticleRadius, $"{kind}: a particle sits {worst / ParticleRadius:P0} of its radius inside the body");
        Assert.True(view.ParticleX[0] > 0.001f, $"{kind}: the rope end was pushed on");
    }

    [Fact]
    public void Sheet_and_rope_are_one_code_path()
    {
        // A 16 x 16 sheet hung by two corners, made by a test-local builder out of the same distances and bends a rope uses.
        using var world = World(particles: 256, distances: 512, bends: 512, tethers: 1024);
        const float sp = 0.04f;
        var sheet = Graph.Sheet(world, (0f, 2f, 0f), (sp, 0f, 0f), (0f, -sp, 0f), 16, 16, ParticleMass, 0.02f, bend: 1e-4f);
        world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[0], point: (0f, 2f, 0f), tether: true);
        world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[15], point: (15 * sp, 2f, 0f), tether: true);
        var view = world.View;
        Run(world, 5 * 120);
        double fy = 0;
        for (var a = 0; a < 2; a++) fy += view.AttachmentForceY[a];
        var weight = 256 * ParticleMass * 10.0;
        Readings.Add($"SHEET attach Fy sum {fy:F3} N, weight {weight:F3} N");
        Assert.True(Math.Abs(fy - weight) < 0.01 * weight, $"the corners carry {fy} N of {weight} N");
        var maxStretch = 0f;
        foreach (var e in sheet.Edges) maxStretch = MathF.Max(maxStretch, MathF.Abs(Graph.Dist(view, view.DistanceA[e.Lane], view.DistanceB[e.Lane]) / view.DistanceRest[e.Lane] - 1f));
        Readings.Add($"SHEET max edge stretch {maxStretch:P3}");
        Assert.True(maxStretch < 0.05f, $"an edge stretched by {maxStretch}");
    }

    [Theory]
    [InlineData(1f)]
    [InlineData(5f)]
    [InlineData(20f)]
    [InlineData(70f)]
    public void Edge_tension_reads_the_load(float mass)
    {
        using var world = World();
        var (_, top, bottom, rope) = Box3DSoaCouplingTests.Hung(world, mass);
        var view = world.View;
        Run(world, 240);
        double t0 = 0, t10 = 0, t46 = 0, topForce = 0, bodyForce = 0;
        for (var i = 0; i < 60; i++)
        {
            world.Step(Dt, Sub);
            t0 += view.DistanceTension[rope.Edges[0].Lane]; t10 += view.DistanceTension[rope.Edges[10].Lane]; t46 += view.DistanceTension[rope.Edges[46].Lane];
            topForce += view.AttachmentForceY[top.Lane]; bodyForce += view.AttachmentForceY[bottom.Lane];
        }
        t0 /= 60; t10 /= 60; t46 /= 60; topForce /= 60; bodyForce /= 60;
        double Below(int edge) => (mass + (47 - edge) * ParticleMass) * 10.0;
        Readings.Add($"EDGE_TENSION M={mass} edge0 {t0:F3}/{Below(0):F3} edge10 {t10:F3}/{Below(10):F3} edge46 {t46:F3}/{Below(46):F3} body {bodyForce:F3}/{-mass * 10.0:F3} static {topForce:F3}");
        Assert.True(Math.Abs(t0 - Below(0)) < 0.01 * Below(0), $"edge 0 reads {t0} N, load {Below(0)} N");
        Assert.True(Math.Abs(t10 - Below(10)) < 0.01 * Below(10), $"edge 10 reads {t10} N, load {Below(10)} N");
        Assert.True(Math.Abs(t46 - Below(46)) < 0.01 * Below(46), $"edge 46 reads {t46} N, load {Below(46)} N");
        Assert.True(Math.Abs(bodyForce + mass * 10.0) < 0.005 * mass * 10.0, $"the body's pull reads {bodyForce} N, weight {mass * 10.0} N");
        var all = (mass + 48 * ParticleMass) * 10.0;
        Assert.True(Math.Abs(topForce - all) < 0.01 * all, $"the static point's support reads {topForce} N, load {all} N");
    }

    [Fact]
    public void Cut_removes_bends_and_rederives_tethers()
    {
        using var world = World();
        var positions = Line((0f, 4f, 0f), (0f, -Spacing, 0f), 40);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        var view = world.View;
        Assert.Equal(39, view.TetherCount);
        Assert.True(view.ParticleComponent[..40].ToArray().All(c => c == 0));
        Run(world, 60);
        world.Remove(rope.Edges[19]);
        Assert.Equal(19, world.View.TetherCount);
        Assert.Equal(0u, view.DistanceLive[19]);
        Assert.Equal(0u, view.BendLive[18]); // the two bends over edge 19 died with it
        Assert.Equal(0u, view.BendLive[19]);
        Assert.Equal(1u, view.BendLive[17]);
        Assert.Equal(0, view.ParticleComponent[19]);
        Assert.Equal(20, view.ParticleComponent[20]); // the tail is its own component, named by its lowest lane
        Assert.Equal(20, view.ParticleComponent[39]);
        Run(world, 120);
        Assert.True(view.ParticleY[19] > 4f - 19 * Spacing - 0.1f, "the head holds");
        Assert.True(view.ParticleY[39] < 4f - 39 * Spacing - 2f, "the tail falls");
        double top = 0;
        for (var i = 0; i < 30; i++) { world.Step(Dt, Sub); top += view.DistanceTension[rope.Edges[0].Lane]; }
        top /= 30;
        var load = 19 * ParticleMass * 10.0;
        Readings.Add($"CUT top edge {top:F4} N, load below {load:F4} N, tail y {world.View.ParticleY[39]:F3}");
        Assert.True(Math.Abs(top - load) < 0.01 * load, $"the top edge reads {top} N, the load below it is {load} N");
    }

    [Fact]
    public void Overload_never_removes_a_constraint()
    {
        using var world = World();
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        const float radius = 0.2f;
        var end = positions[47];
        var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Graph.Dens(300f, radius));
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[47], sphere, (0f, radius, 0f), tether: true);
        var view = world.View;
        Run(world, 3 * 120);
        var tension = 0f;
        foreach (var e in rope.Edges) tension = MathF.Max(tension, view.DistanceTension[e.Lane]);
        Assert.True(tension > 1000f, $"the load is read: {tension} N"); // 300 kg: far past any strength a material would give it
        for (var i = 0; i < 47; i++) Assert.Equal(1u, view.DistanceLive[i]);
        for (var i = 0; i < 46; i++) Assert.Equal(1u, view.BendLive[i]);
        for (var i = 0; i < 48; i++) Assert.Equal(1u, view.ParticleLive[i]);
        view = world.View;
        Assert.Equal(2, view.AttachmentCount);
        world.Remove(rope.Edges[20]); // only the command cuts
        Assert.Equal(0u, view.DistanceLive[20]);
    }

    [Fact]
    public void Destroy_particles_removes_their_constraints()
    {
        using var world = World();
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 48);
        var rope = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
        const float radius = 0.15f, mass = 70f;
        var end = positions[47];
        var sphere = world.AddSphere(10, (end.X, end.Y - radius, end.Z), radius, Graph.Dens(mass, radius));
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[47], sphere, (0f, radius, 0f), tether: true);
        var view = world.View;
        Run(world, 2 * 120);
        Assert.True(MathF.Abs(view.VelocityY[sphere]) < 0.001f, "held");
        world.DestroyParticles([rope.Particles[24]]);
        Assert.Equal(0u, view.ParticleLive[24]);
        Assert.Equal(0u, view.DistanceLive[23]); // its two distances
        Assert.Equal(0u, view.DistanceLive[24]);
        foreach (var k in new[] { 22, 23, 24 }) Assert.Equal(0u, view.BendLive[k]); // the bends over them
        Assert.Equal(1u, view.BendLive[21]);
        Assert.Equal(23 + 22, world.View.TetherCount); // the head 1..23 bound by the static point, the tail 25..46 by the body at 47
        Run(world, 1);
        Assert.True(view.VelocityY[sphere] < -0.05f, $"the held body fell the step after: vy {view.VelocityY[sphere]}");
    }

    [Theory]
    [InlineData(32)]
    public void Soft_bend_and_hard_limit_one_constraint(int substeps)
    {
        // A 24-particle cantilever held at both of its first particles: the soft pass alone, the hard limit alone and both.
        (float Drop, float Defl) Cantilever(float compliance, float limitDeg)
        {
            using var world = World(gravityY: -9.81f);
            var positions = Line((0f, 1f, 0f), (0.04f, 0f, 0f), 24);
            var rope = Graph.Rope(world, positions, ParticleMass, 0.02f, bend: compliance, maxAngle: limitDeg < 0 ? -1f : limitDeg * MathF.PI / 180f);
            world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0]);
            world.Attach(Box3DSoaAttachKind.Static, rope.Particles[1], point: positions[1]);
            var view = world.View;
            for (var i = 0; i < 300; i++) world.Step(1f / 60f, substeps);
            float worst = 0f;
            for (var k = 1; k + 1 < 24; k++)
            {
                (float X, float Y) a = (view.ParticleX[k] - view.ParticleX[k - 1], view.ParticleY[k] - view.ParticleY[k - 1]);
                (float X, float Y) b = (view.ParticleX[k + 1] - view.ParticleX[k], view.ParticleY[k + 1] - view.ParticleY[k]);
                var c = (a.X * b.X + a.Y * b.Y) / MathF.Sqrt((a.X * a.X + a.Y * a.Y) * (b.X * b.X + b.Y * b.Y));
                worst = MathF.Max(worst, MathF.Acos(MathF.Min(1f, c)) * 180f / MathF.PI);
            }
            return (1f - view.ParticleY[23], worst);
        }
        var stiff = Cantilever(1e-7f, -1f);
        var floppy = Cantilever(1e-4f, -1f);
        var thirty = Cantilever(-1f, 30f);
        var ten = Cantilever(-1f, 10f);
        Readings.Add($"BEND {substeps} substeps: stiff drop {stiff.Drop:F3} defl {stiff.Defl:F2}; floppy {floppy.Drop:F3} / {floppy.Defl:F2}; limit 30: {thirty.Drop:F3} / {thirty.Defl:F2}; limit 10: {ten.Drop:F3} / {ten.Defl:F2}");
        Assert.True(stiff.Drop < floppy.Drop, $"stiff {stiff.Drop} m, floppy {floppy.Drop} m");
        Assert.True(thirty.Defl <= 31f, $"the 30 degree limit allowed {thirty.Defl} degrees");
        Assert.True(ten.Defl <= 11.5f, $"the 10 degree limit allowed {ten.Defl} degrees");
        Assert.True(ten.Drop < thirty.Drop, "the tighter limit droops less");
        // A limit alone applies nothing inside its cone: a gently swinging hung rope is bit-identical with and without it.
        float[] Swing(bool limited)
        {
            using var world = World(gravityY: -10f);
            var step = (MathF.Sin(0.05f) * Spacing, -MathF.Cos(0.05f) * Spacing, 0f);
            var positions = Line((0f, 3f, 0f), step, 24);
            var rope = limited ? Graph.Rope(world, positions, ParticleMass, ParticleRadius, bend: -1f, maxAngle: 30f * MathF.PI / 180f)
                               : Graph.Rope(world, positions, ParticleMass, ParticleRadius, bend: -1f);
            world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: positions[0], tether: true);
            var view = world.View;
            Run(world, 60);
            return [.. view.ParticleX, .. view.ParticleY];
        }
        Assert.True(Swing(true).AsSpan().SequenceEqual(Swing(false)), "a limit-only bend moved something inside its cone");
    }

    [Fact]
    public void Contact_conserves_momentum()
    {
        // Zero gravity: a self-colliding rope thrown at 3 m/s into a self-colliding sheet and a loose blob; the sum of momentum
        // may not drift and the kinetic energy may not rise (a per-particle average of contacts fails both: it drifts by 5.6e-2).
        // 5e-4 is the float32 floor, not a contact property: translating the whole scene moves the position noise between
        // 0.56e-4 and 4.0e-4 under both contact walks.
        using var world = World(particles: 512, distances: 512, bends: 512, tethers: 8, gravityY: 0f);
        world.AddStaticBox(1, (0f, -100f, 0f), (1f, 1f, 1f));
        var sheet = Graph.Sheet(world, (0f, 0f, 0f), (0.04f, 0f, 0f), (0f, 0.04f, 0f), 12, 12, ParticleMass, 0.02f, Box3DSoaPhase.Of(7, true, true));
        var rope = Graph.Rope(world, Line((-0.6f, 0.2f, -0.3f), (0f, 0f, 0.04f), 31), 0.03f, 0.02f, Box3DSoaPhase.Of(8, true, true), bend: 1e-5f, velocity: (3f, 0f, 0f));
        var blob = new List<Box3DSoaParticleDesc>();
        for (var a = 0; a < 5; a++) for (var b = 0; b < 5; b++) for (var c = 0; c < 5; c++)
            blob.Add(new Box3DSoaParticleDesc((0.15f + a * 0.04f, 0.15f + b * 0.04f, 0.4f + c * 0.04f), 0.008f, 0.02f, Box3DSoaPhase.Of(9), velocity: (0f, 0f, -4f)));
        world.CreateParticles(blob.ToArray());
        var view = world.View;
        (double X, double Y, double Z, double E) Total()
        {
            double x = 0, y = 0, z = 0, e = 0;
            for (var i = 0; i < view.ParticleCount; i++)
            {
                if (view.ParticleLive[i] == 0 || view.ParticleInvMass[i] == 0f) continue;
                double m = 1.0 / view.ParticleInvMass[i];
                x += m * view.ParticleVx[i]; y += m * view.ParticleVy[i]; z += m * view.ParticleVz[i];
                e += 0.5 * m * (Sq(view.ParticleVx[i]) + Sq(view.ParticleVy[i]) + Sq(view.ParticleVz[i]));
            }
            return (x, y, z, e);
        }
        var p0 = Total();
        var emax = p0.E;
        for (var i = 0; i < 180; i++)
        {
            world.Step(1f / 60f, 8);
            emax = Math.Max(emax, Total().E);
        }
        var p1 = Total();
        var mag = Math.Sqrt(p0.X * p0.X + p0.Y * p0.Y + p0.Z * p0.Z);
        var drift = Math.Sqrt(Sq((float)(p1.X - p0.X)) + Sq((float)(p1.Y - p0.Y)) + Sq((float)(p1.Z - p0.Z)));
        Readings.Add($"MOMENTUM drift {drift / mag:P4}, energy max/initial {emax / p0.E:F6}");
        Assert.True(drift / mag < 5e-4, $"momentum drifted by {drift / mag:P4}");
        Assert.True(emax <= p0.E * (1 + 1e-5), $"kinetic energy rose from {p0.E} to {emax}");
        _ = sheet; _ = rope;
    }

    [Fact]
    public void Phase_rules()
    {
        using var world = World(gravityY: 0f);
        world.AddStaticBox(1, (0f, -100f, 0f), (1f, 1f, 1f));
        // Two pairs of particles flying at each other: different groups collide, one group without SELF_COLLIDE passes through.
        var apart = new Box3DSoaParticleDesc[]
        {
            new((0f, 0f, 0f), 0.02f, 0.02f, Box3DSoaPhase.Of(1), velocity: (1f, 0f, 0f)),
            new((0.2f, 0f, 0f), 0.02f, 0.02f, Box3DSoaPhase.Of(2), velocity: (-1f, 0f, 0f)),
            new((0f, 1f, 0f), 0.02f, 0.02f, Box3DSoaPhase.Of(3), velocity: (1f, 0f, 0f)),
            new((0.2f, 1f, 0f), 0.02f, 0.02f, Box3DSoaPhase.Of(3), velocity: (-1f, 0f, 0f)),
        };
        world.CreateParticles(apart);
        var view = world.View;
        Run(world, 60);
        Assert.True(view.ParticleX[1] - view.ParticleX[0] > 0.039f && view.ParticleX[0] < view.ParticleX[1], "different groups collided");
        Assert.True(view.ParticleX[2] > view.ParticleX[3] + 0.1f, "one group without SELF_COLLIDE passed through");
        // SELF_FILTER: two free particles that touch in the rest pose keep their distance; a folded rope's far parts do not overlap.
        var phase = Box3DSoaPhase.Of(4, true, true);
        var pair = new Box3DSoaParticleHandle[2];
        world.CreateParticles([new((0f, 3f, 0f), 0.02f, 0.0125f, phase), new((0.01f, 3f, 0f), 0.02f, 0.0125f, phase)], pair);
        var straight = Line((0f, 6f, 0f), (Spacing, 0f, 0f), 24);
        var folded = new Pos[24];
        for (var i = 0; i < 12; i++) folded[i] = (i * Spacing, 6f, 0f);
        for (var i = 12; i < 24; i++) folded[i] = ((23 - i) * Spacing, 6f, 0.02f); // the second arm lies 2 cm over the first
        var rope = Graph.Rope(world, folded, ParticleMass, ParticleRadius, phase, rest: straight, bend: -1f, maxAngle: -1f);
        view = world.View;
        Run(world, 120);
        var held = Graph.Dist(view, pair[0], pair[1]);
        Assert.True(MathF.Abs(held - 0.01f) < 0.0005f, $"a rest-touching pair was pushed apart to {held}");
        var closest = float.MaxValue;
        for (var i = 0; i < 24; i++)
            for (var j = i + 3; j < 24; j++)
                closest = MathF.Min(closest, Graph.Dist(view, rope.Particles[i], rope.Particles[j]));
        Assert.True(closest > 0.9f * 2f * ParticleRadius, $"far parts of the folded rope overlap: {closest}");
    }

    [Fact]
    public void Same_group_flags_need_both_to_collide_and_either_to_filter()
    {
        // One group, so only the flags decide: SELF_COLLIDE must be on for both particles, and SELF_FILTER on either one waives a
        // pair that touches in its rest pose, in either order.
        using var world = World(gravityY: 0f);
        world.CreateParticles(
        [
            new((0f, 0f, 0f), 0.02f, 0.02f, Box3DSoaPhase.Of(5, true), velocity: (1f, 0f, 0f)),
            new((0.2f, 0f, 0f), 0.02f, 0.02f, Box3DSoaPhase.Of(5), velocity: (-1f, 0f, 0f)),
            new((0f, 1f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(6, true, true)),
            new((0.01f, 1f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(6, true)),
            new((0f, 2f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(7, true)),
            new((0.01f, 2f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(7, true, true)),
            new((0f, 3f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(8, true)),
            new((0.01f, 3f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(8, true)),
        ]);
        var view = world.View;
        Run(world, 60);
        Assert.True(view.ParticleX[0] > view.ParticleX[1] + 0.1f, "SELF_COLLIDE on one particle of a pair collided it");
        Assert.True(MathF.Abs(view.ParticleX[3] - view.ParticleX[2] - 0.01f) < 5e-4f, "SELF_FILTER on the first particle did not waive a rest-touching pair");
        Assert.True(MathF.Abs(view.ParticleX[5] - view.ParticleX[4] - 0.01f) < 5e-4f, "SELF_FILTER on the second particle did not waive a rest-touching pair");
        Assert.True(view.ParticleX[7] - view.ParticleX[6] > 0.02f, "an unfiltered rest-touching pair did not push apart"); // the control: the same pair collides without the flag
    }

    [Fact]
    public void Pair_tether_pulls_only_when_the_points_are_too_far_apart()
    {
        // A slack zigzag rope between a static point and a free body: the end points are 0.35 m apart and the rest path is 0.78 m, so
        // the pair tether is slack and the body does not move. A bilateral bound would drag it out to the path length.
        using var world = World(gravityY: 0f);
        var zig = Enumerable.Range(0, 8).Select(i => ((float X, float Y, float Z))(i * 0.05f, (i % 2) * 0.1f, 0f)).ToArray();
        var rope = Graph.Rope(world, zig, 0.001f, 0.005f, bend: -1f);
        var body = world.AddSphere(10, (0.35f + 0.1f, 0.1f, 0f), 0.1f, Graph.Dens(1f, 0.1f));
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: zig[0], tether: true);
        world.Attach(Box3DSoaAttachKind.Body, rope.Particles[7], body, (-0.1f, 0f, 0f), tether: true);
        var view = world.View;
        Assert.True(view.TetherOther.ToArray().Any(o => o >= 0), "the pair tether exists");
        Run(world, 120);
        Assert.True(MathF.Abs(view.PositionX[body] - 0.45f) < 1e-3f, $"the slack tether moved the body to x {view.PositionX[body]}");
    }

    [Fact]
    public void A_refused_attachment_leaves_no_lane_and_no_tether()
    {
        // A 10-particle rope under tether capacity 10: the first tether point needs 9 tethers, a second one more than the capacity.
        using var world = World(particles: 16, distances: 16, bends: 16, attachments: 4, tethers: 10);
        var rope = Graph.Rope(world, Line((0f, 0f, 0f), (0.03f, 0f, 0f), 10), ParticleMass, ParticleRadius, bend: -1f);
        world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: (0f, 0f, 0f), tether: true);
        var view = world.View;
        var tethers = view.TetherCount;
        Assert.Throws<InvalidOperationException>(() => world.Attach(Box3DSoaAttachKind.Static, rope.Particles[9], point: (0.27f, 0f, 0f), tether: true));
        view = world.View;
        Assert.Equal(1, view.AttachmentCount);
        Assert.Equal(tethers, view.TetherCount);
        var kept = world.Attach(Box3DSoaAttachKind.Static, rope.Particles[9], point: (0.27f, 0f, 0f)); // no tether: it fits, in the lane the refusal freed
        Assert.Equal(1, kept.Lane);
        view = world.View;
        Assert.Equal(2, view.AttachmentCount);
    }

    [Fact]
    public void Each_particle_takes_the_nearest_point_of_its_two_nearest_islands()
    {
        // (a) A 5-rope held at both ends by static tether points: each middle particle has one tether per end, bounded by the
        // rest path to it, and the two world points get no pair tether between them (neither can move).
        using (var world = World())
        {
            var rope = Graph.Rope(world, Line((0f, 0f, 0f), (0.03f, 0f, 0f), 5), ParticleMass, ParticleRadius, bend: -1f);
            var first = world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: (0f, 0f, 0f), tether: true);
            var second = world.Attach(Box3DSoaAttachKind.Static, rope.Particles[4], point: (0.12f, 0f, 0f), tether: true);
            var view = world.View;
            Assert.Equal(6, view.TetherCount);
            Assert.DoesNotContain(view.TetherOther.ToArray(), o => o >= 0);
            for (var k = 1; k <= 3; k++)
            {
                Assert.Equal(0.03f * k, Bound(view, first.Lane, rope.Particles[k].Lane), 1e-5f);
                Assert.Equal(0.03f * (4 - k), Bound(view, second.Lane, rope.Particles[k].Lane), 1e-5f);
            }
        }
        // (b) A star: the centre is joined by arms of one, two and three edges to three static tether points, and is held to
        // the one- and two-edge points only: the third island is not among its two nearest.
        using (var world = World())
        {
            var star = new List<Box3DSoaParticleDesc> { new((0f, 0f, 0f), ParticleMass, ParticleRadius, 0) };
            Pos[] dir = [(1f, 0f, 0f), (-0.5f, 0.866f, 0f), (-0.5f, -0.866f, 0f)];
            var ends = new int[3];
            var arm = new List<(int From, int To)>();
            for (var a = 0; a < 3; a++)
            {
                var from = 0;
                for (var s2 = 1; s2 <= a + 1; s2++)
                {
                    star.Add(new Box3DSoaParticleDesc((dir[a].X * 0.03f * s2, dir[a].Y * 0.03f * s2, 0f), ParticleMass, ParticleRadius, 0));
                    arm.Add((from, star.Count - 1));
                    from = star.Count - 1;
                }
                ends[a] = from;
            }
            var particles = new Box3DSoaParticleHandle[star.Count];
            world.CreateParticles(star.ToArray(), particles);
            var edges = new Box3DSoaConstraintHandle[arm.Count];
            world.CreateDistances(arm.Select(x => new Box3DSoaDistanceDesc(particles[x.From], particles[x.To], 0f)).ToArray(), edges);
            var points = new Box3DSoaConstraintHandle[3];
            for (var a = 0; a < 3; a++)
                points[a] = world.Attach(Box3DSoaAttachKind.Static, particles[ends[a]], point: (dir[a].X * 0.03f * (a + 1), dir[a].Y * 0.03f * (a + 1), 0f), tether: true);
            var view = world.View;
            Assert.Equal(0.03f, Bound(view, points[0].Lane, particles[0].Lane), 1e-5f);
            Assert.Equal(0.06f, Bound(view, points[1].Lane, particles[0].Lane), 1e-5f);
            Assert.Equal(-1f, Bound(view, points[2].Lane, particles[0].Lane, absent: -1f));
        }
        // (c) A 9-rope with tether points on particles 0 and 1 (one island: they touch) and on 8: particle 4 is held to the
        // nearer of the island's two points, the one on particle 1, and to the point on 8, never to the one on 0.
        using (var world = World())
        {
            var line = Line((0f, 0f, 0f), (0.03f, 0f, 0f), 9);
            var rope = Graph.Rope(world, line, ParticleMass, ParticleRadius, bend: -1f);
            var p0 = world.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: line[0], tether: true);
            var p1 = world.Attach(Box3DSoaAttachKind.Static, rope.Particles[1], point: line[1], tether: true);
            var p8 = world.Attach(Box3DSoaAttachKind.Static, rope.Particles[8], point: line[8], tether: true);
            var view = world.View;
            var q = rope.Particles[4].Lane;
            Assert.Equal(0.09f, Bound(view, p1.Lane, q), 1e-5f);
            Assert.Equal(0.12f, Bound(view, p8.Lane, q), 1e-5f);
            Assert.Equal(-1f, Bound(view, p0.Lane, q, absent: -1f));
            Assert.Equal(12, view.TetherCount); // particles 2..7, two each
        }
    }

    // The bound of the one tether that holds `particle` to attachment `att` (a particle tether, not a pair route).
    private static float Bound(Box3DSoaView view, int att, int particle, float absent = float.NaN)
    {
        var found = new List<float>();
        for (var t = 0; t < view.TetherCount; t++)
            if (view.TetherAttachment[t] == att && view.TetherParticle[t] == particle && view.TetherOther[t] < 0) found.Add(view.TetherBound[t]);
        if (found.Count == 0 && !float.IsNaN(absent)) return absent;
        Assert.Single(found);
        return found[0];
    }

    [Fact]
    public void Every_tether_has_its_parent_on_a_32_by_32_sheet_pinned_at_20_points()
    {
        // The ranked islands of a particle are closed along each island's path tree in exact arithmetic only: float32 path sums
        // break the tie of two equal paths, so a particle can keep an island its parent dropped. The derivation then writes no
        // tether for it, and every tether's parent is the tether of the particle one edge nearer or that particle is a point.
        using var world = World(particles: 1024, distances: 2048, bends: 2048, attachments: 32, tethers: 4096);
        const float sp = 0.04f;
        var sheet = Graph.Sheet(world, (0f, 2f, 0f), (sp, 0f, 0f), (0f, -sp, 0f), 32, 32, ParticleMass, 0.02f);
        var held = new HashSet<int>();
        var checkedTethers = 0;
        void Check() // the invariant after each derivation: the ties fall where the pins stand at that command
        {
            var view = world.View;
            var pinned = new HashSet<int>(view.AttachmentParticle.ToArray());
            checkedTethers = 0;
            for (var t = 0; t < view.TetherCount; t++)
            {
                if (view.TetherAttachment[t] < 0 || view.TetherOther[t] >= 0) continue;
                var e = view.TetherEdge[t];
                var u = view.TetherParticle[t];
                var near = view.DistanceA[e] == u ? view.DistanceB[e] : view.DistanceA[e];
                var parent = view.TetherParent[t];
                if (parent < 0) Assert.True(pinned.Contains(near), $"tether {t} of particle {u} has no parent but its nearer particle {near} holds no point");
                else
                {
                    Assert.True(parent < t, $"tether {t} routes through a later lane");
                    Assert.Equal(near, view.TetherParticle[parent]);
                }
                checkedTethers++;
            }
        }
        for (var a = 0; a < 20; a++)
        {
            var i = a == 0 ? 0 : a == 1 ? 31 : a * 37 % 1024;
            if (!held.Add(i)) continue;
            world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[i], point: (i % 32 * sp, 2f - i / 32 * sp, 0f), tether: true);
            Check();
        }
        var view = world.View;
        Readings.Add($"PINNED_32 tethers {view.TetherCount}, particle tethers {checkedTethers}");
        Assert.True(checkedTethers > 1900, $"{checkedTethers} particle tethers");
    }

    [Fact]
    public void Sheet_hung_by_two_corner_groups()
    {
        // 16 x 16 hung by pins on top-row particles 0, 1, 14 and 15: two corners of two adjacent pins each. Ranking pins instead
        // of islands spends both of a particle's tethers on one corner's two pins and the sheet stretches by 11.6% at 5 s.
        // Read at 8 s: the sheet still rings at 5 s (Fy 50.636 N of 51.200, 1.10% under) and carries its weight within 0.2% from
        // 6 s (51.286, 51.227, 51.132 N at 6, 7, 8 s); the two-corner single-pin sheet settles by about 9 s the same way.
        using var world = World(particles: 256, distances: 512, bends: 512, attachments: 8, tethers: 1024);
        const float sp = 0.04f;
        var sheet = Graph.Sheet(world, (0f, 2f, 0f), (sp, 0f, 0f), (0f, -sp, 0f), 16, 16, ParticleMass, 0.02f, bend: 1e-4f);
        foreach (var i in new[] { 0, 15, 1, 14 })
            world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[i], point: (i * sp, 2f, 0f), tether: true);
        var view = world.View;
        Run(world, 8 * 120);
        double fy = 0;
        for (var a = 0; a < 4; a++) fy += view.AttachmentForceY[a];
        var weight = 256 * ParticleMass * 10.0;
        Readings.Add($"CORNER_GROUPS attach Fy sum {fy:F3} N, weight {weight:F3} N");
        Assert.True(Math.Abs(fy - weight) < 0.01 * weight, $"the pins carry {fy} N of {weight} N");
        var maxStretch = 0f;
        foreach (var e in sheet.Edges) maxStretch = MathF.Max(maxStretch, MathF.Abs(Graph.Dist(view, view.DistanceA[e.Lane], view.DistanceB[e.Lane]) / view.DistanceRest[e.Lane] - 1f));
        Readings.Add($"CORNER_GROUPS max edge stretch {maxStretch:P3}");
        Assert.True(maxStretch < 0.05f, $"an edge stretched by {maxStretch}");
    }

    [Fact]
    public void A_particle_held_by_two_bodies_touches_neither()
    {
        // One particle tied to the facing sides of two touching boxes sits on both surfaces. Contact skips every body the particle is
        // attached to; skipping only one pushes the particle, and so the other box, off its point by the particle's radius.
        using var world = World(gravityY: 0f);
        var left = world.AddBox(2, (-0.1f, 0f, 0f), (0.1f, 0.1f, 0.1f), 500f);
        var right = world.AddBox(3, (0.1f, 0f, 0f), (0.1f, 0.1f, 0.1f), 500f);
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new((0f, 0f, 0f), 0.02f, 0.0125f)], p);
        world.Attach(Box3DSoaAttachKind.Body, p[0], left, (0.1f, 0f, 0f));
        world.Attach(Box3DSoaAttachKind.Body, p[0], right, (-0.1f, 0f, 0f));
        var view = world.View;
        Run(world, 60);
        Readings.Add($"TWO_BODIES particle x {view.ParticleX[p[0].Lane]:G6}, boxes {view.PositionX[left]:G6} {view.PositionX[right]:G6}");
        Assert.True(MathF.Abs(view.ParticleX[p[0].Lane]) < 1e-3f && MathF.Abs(view.PositionX[left] + 0.1f) < 1e-3f && MathF.Abs(view.PositionX[right] - 0.1f) < 1e-3f,
            $"contact moved something: particle {view.ParticleX[p[0].Lane]}, boxes {view.PositionX[left]} {view.PositionX[right]}");
    }

    [Fact]
    public void Off_centre_attachment_spins_a_free_box()
    {
        // A particle tied to a point off a free box's centre and pulled by a world point: the box's angular velocity changes by
        // I^-1 (r x J), J being the impulse lane. Dropping the angular impulse (sub_angular, or the fork's angular argument) breaks the 2%; the proxy's rotation is held by the two tests after it.
        using var world = World(gravityY: 0f);
        const float half = 0.2f, mass = 1f;
        var box = world.AddBox(2, (0f, 0f, 0f), (half, half, half), mass / (8f * half * half * half));
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new((half, 0f, 0f), 0.05f, 0.02f)], p);
        world.Attach(Box3DSoaAttachKind.Body, p[0], box, (half, 0f, 0f));
        world.Attach(Box3DSoaAttachKind.Static, p[0], point: (half, 0.05f, 0f), compliance: 1e-3f);
        var view = world.View;
        world.Step(Dt, Sub);
        var j = (X: view.ParticleImpulseX[box], Y: view.ParticleImpulseY[box], Z: view.ParticleImpulseZ[box]);
        var inertia = mass * (2f * half) * (2f * half) / 6f; // a cube: m s^2 / 6
        var wz = (half * j.Y - 0f * j.X) / inertia; // r = (half, 0, 0): (r x J).z = half * Jy
        Assert.True(MathF.Abs(j.Y) > 0.001f, $"the pull is read: {j.Y}");
        Assert.True(MathF.Abs(view.AngularVelocityZ[box] - wz) < 0.02f * MathF.Abs(wz), $"omega_z {view.AngularVelocityZ[box]}, expected {wz}");
        Assert.True(MathF.Abs(view.VelocityY[box] - j.Y / mass) < 0.02f * MathF.Abs(j.Y / mass), $"vy {view.VelocityY[box]}, expected {j.Y / mass}");
    }

    /// <summary>The world position of a point fixed in a body.</summary>
    private static (float X, float Y, float Z) BodyPoint(Box3DSoaView v, int body, (float X, float Y, float Z) local)
    {
        float qx = v.RotationX[body], qy = v.RotationY[body], qz = v.RotationZ[body], qw = v.RotationW[body];
        var tx = 2f * (qy * local.Z - qz * local.Y); var ty = 2f * (qz * local.X - qx * local.Z); var tz = 2f * (qx * local.Y - qy * local.X);
        return (v.PositionX[body] + local.X + qw * tx + (qy * tz - qz * ty),
                v.PositionY[body] + local.Y + qw * ty + (qz * tx - qx * tz),
                v.PositionZ[body] + local.Z + qw * tz + (qx * ty - qy * tx));
    }

    private static float Gap(Box3DSoaView v, Box3DSoaParticleHandle p, int body, (float X, float Y, float Z) local)
    {
        var at = BodyPoint(v, body, local);
        return MathF.Sqrt(Sq(v.ParticleX[p.Lane] - at.X) + Sq(v.ParticleY[p.Lane] - at.Y) + Sq(v.ParticleZ[p.Lane] - at.Z));
    }

    [Fact]
    public void Gravity_acts_along_all_three_axes()
    {
        // A loose particle in z gravity falls g t^2 / 2 along z and nowhere else; the same for x and y.
        foreach (var axis in new[] { 0, 1, 2 })
        {
            using var world = World(gravityX: axis == 0 ? -10f : 0f, gravityY: axis == 1 ? -10f : 0f, gravityZ: axis == 2 ? -10f : 0f);
            var p = new Box3DSoaParticleHandle[1];
            world.CreateParticles([new((0f, 0f, 0f), ParticleMass, ParticleRadius)], p);
            Run(world, 120);
            var view = world.View;
            float[] fell = [view.ParticleX[p[0].Lane], view.ParticleY[p[0].Lane], view.ParticleZ[p[0].Lane]];
            Assert.True(MathF.Abs(fell[axis] + 5f) < 0.1f, $"axis {axis}: fell {fell[axis]} m, expected -5");
            for (var other = 0; other < 3; other++)
                if (other != axis) Assert.True(MathF.Abs(fell[other]) < 1e-4f, $"axis {axis}: moved {fell[other]} along {other}");
        }
    }

    [Fact]
    public void Spinning_body_carries_its_attachment_point()
    {
        // A free box spinning at 8 rad/s with a light particle tied 0.3 m off its centre: the proxy is predicted to the pose Box3D
        // reaches, so the particle stays on the body's point. Without the rotation prediction it trails by w h r = 5 mm every substep.
        using var world = World(gravityY: 0f);
        var box = world.AddBox(2, (0f, 0f, 0f), (0.4f, 0.1f, 0.1f), 500f);
        const float iyy = 16f / 12f * (0.8f * 0.8f + 0.2f * 0.2f), l = 8f * iyy; // a couple about +y: omega_y = 8
        world.ApplyImpulse(box, (0f, 0f, -l), (1f, 0f, 0f));
        world.ApplyImpulse(box, (0f, 0f, l), (0f, 0f, 0f));
        var p = new Box3DSoaParticleHandle[1];
        world.CreateParticles([new((0.3f, 0f, 0f), 0.001f, 0.02f)], p);
        world.Attach(Box3DSoaAttachKind.Body, p[0], box, (0.3f, 0f, 0f));
        var view = world.View;
        var worst = 0f;
        for (var i = 0; i < 120; i++)
        {
            world.Step(Dt, Sub);
            worst = MathF.Max(worst, Gap(view, p[0], box, (0.3f, 0f, 0f)));
        }
        Readings.Add($"SPIN_ATTACH worst gap {worst:G6} m");
        Assert.True(worst < 5e-4f, $"the particle trailed its body point by {worst} m");
    }

    [Fact]
    public void Two_ends_on_one_light_box_settle_together()
    {
        // Two particles pull a light free box's ends: the first correction turns the proxy by a fifth of a radian, and the second
        // attachment is solved against the turned pose. Left at its pose from before the turn it corrects the wrong point and the
        // gaps ring instead of closing.
        using var world = World(gravityY: 0f);
        const float half = 0.1f;
        var box = world.AddBox(2, (0f, 0f, 0f), (half, half, half), 0.2f / (8f * half * half * half));
        var p = new Box3DSoaParticleHandle[2];
        world.CreateParticles([new((half, 0.05f, 0f), 2f, 0.02f), new((-half, -0.05f, 0f), 2f, 0.02f)], p);
        world.Attach(Box3DSoaAttachKind.Body, p[0], box, (half, 0f, 0f));
        world.Attach(Box3DSoaAttachKind.Body, p[1], box, (-half, 0f, 0f));
        var view = world.View;
        var worst = 0f;
        for (var i = 0; i < 60; i++)
        {
            world.Step(Dt, Sub);
            if (i >= 20) worst = MathF.Max(worst, MathF.Max(Gap(view, p[0], box, (half, 0f, 0f)), Gap(view, p[1], box, (-half, 0f, 0f))));
        }
        Readings.Add($"TWO_ENDS worst gap after 20 steps {worst:G6} m");
        Assert.True(worst < 1e-3f, $"the ends were {worst} m off their points");
    }

    // ---- lanes ----------------------------------------------------------------------------------------------

    [Fact]
    public void Lanes_reused_after_a_step_and_refusals_change_nothing()
    {
        using var world = World(particles: 12, distances: 12, bends: 12, attachments: 2, tethers: 20);
        var positions = Line((0f, 5f, 0f), (0f, -Spacing, 0f), 10);
        var a = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        var attachment = world.Attach(Box3DSoaAttachKind.Static, a.Particles[0], point: positions[0], tether: true);
        world.Step(Dt, Sub);
        world.DestroyParticles(a.Particles);
        var view = world.View;
        Assert.Equal(0u, view.ParticleLive[0]);
        Assert.Equal(0f, view.ParticleY[0]);
        Assert.Equal(1u, view.ParticleGeneration[0]);
        Assert.Equal(world.StepCount, view.ParticleRemovedAt[0]);
        Assert.Equal((uint)Box3DSoaAttachKind.Removed, view.AttachmentKind[attachment.Lane]); // it would dangle
        Assert.Equal(0, world.View.TetherCount);
        Assert.Equal(2, world.FreeParticleSlots); // only the two lanes beyond the destroyed ones
        Assert.Throws<InvalidOperationException>(() => Graph.Rope(world, positions, ParticleMass, ParticleRadius)); // the lanes are held this step
        Assert.Equal(10, view.ParticleCount);
        world.Step(Dt, Sub);
        Assert.Equal(12, world.FreeParticleSlots);
        var b = Graph.Rope(world, positions, ParticleMass, ParticleRadius);
        Assert.Equal(a.Particles.Select(h => h.Lane), b.Particles.Select(h => h.Lane)); // lowest first
        Assert.Equal(a.Edges.Select(h => h.Lane), b.Edges.Select(h => h.Lane));
        Assert.Equal(1u, b.Particles[0].Generation);
        Assert.Throws<InvalidOperationException>(() => world.DestroyParticles([a.Particles[0]])); // stale
        Assert.Throws<InvalidOperationException>(() => world.Remove(a.Edges[0]));
        Assert.Throws<InvalidOperationException>(() => world.Remove(attachment));
        var c = world.Attach(Box3DSoaAttachKind.Static, b.Particles[0], point: positions[0]);
        world.Remove(c);
        world.Step(Dt, Sub);
        Assert.Equal(c.Lane, world.Attach(Box3DSoaAttachKind.Static, b.Particles[0], point: positions[0]).Lane);

        // a derivation that does not fit is refused whole
        using var small = World(particles: 12, distances: 12, bends: 12, attachments: 2, tethers: 5);
        var ps = new Box3DSoaParticleHandle[10];
        small.CreateParticles(Line((0f, 1f, 0f), (Spacing, 0f, 0f), 10).Select(q => new Box3DSoaParticleDesc(q, 0.02f, 0.0125f)).ToArray(), ps);
        small.Attach(Box3DSoaAttachKind.Static, ps[0], point: (0f, 1f, 0f), tether: true);
        var d = Enumerable.Range(0, 9).Select(i => new Box3DSoaDistanceDesc(ps[i], ps[i + 1])).ToArray();

        Assert.Throws<InvalidOperationException>(() => small.CreateDistances(d)); // would need 9 tethers
        Assert.Equal(0, small.View.DistanceCount);
        Assert.Equal(0, small.View.TetherCount);
        Assert.Equal(12, small.FreeDistanceSlots);
        small.CreateDistances(d[..5]); // 5 tethers fit
        Assert.Equal(5, small.View.TetherCount);
        Assert.Throws<InvalidOperationException>(() => small.CreateDistances(d[5..6])); // the sixth would not
        Assert.Equal(5, small.View.DistanceCount);
        Assert.Equal(5, small.View.TetherCount);

        // refusals
        Assert.Throws<InvalidOperationException>(() => small.CreateParticles([new((float.NaN, 0f, 0f), 1f, 0.01f)]));
        Assert.Throws<InvalidOperationException>(() => small.CreateParticles([new((0f, 0f, 0f), 1f, 0f)])); // no radius
        Assert.Throws<InvalidOperationException>(() => small.CreateParticles([new((0f, 0f, 0f), 1f, 0.01f, 1u << 24)])); // an unknown phase bit
        Assert.Throws<InvalidOperationException>(() => small.CreateParticles([new((0f, 0f, 0f), 1f, 0.01f, 0, 2)])); // no such material
        Assert.Throws<InvalidOperationException>(() => small.CreateDistances([new Box3DSoaDistanceDesc(ps[0], ps[0])]));
        Assert.Throws<InvalidOperationException>(() => small.Attach(Box3DSoaAttachKind.Body, ps[0], 5)); // no such body
        Assert.Throws<InvalidOperationException>(() => small.Attach(Box3DSoaAttachKind.Static, new Box3DSoaParticleHandle(99, 0)));
        Assert.Throws<InvalidOperationException>(() => small.Attach(Box3DSoaAttachKind.Static, ps[0], point: (float.PositiveInfinity, 0f, 0f)));
        Assert.Throws<InvalidOperationException>(() => small.CreateBends([new Box3DSoaBendDesc(new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Distance, 0, 0), new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Distance, 3, 0))])); // share no particle
        Assert.Equal(10, small.View.ParticleCount);
        Assert.Equal(0, small.View.BendCount);
        Assert.Equal(1, small.View.AttachmentCount);
        using var plain = new Box3DSoaWorld(2); // no particle capacity: every command is refused
        Assert.Throws<InvalidOperationException>(() => plain.CreateParticles([new((0f, 0f, 0f), 1f, 0.01f)]));
        Assert.Throws<InvalidOperationException>(() => new Box3DSoaWorld(2, particles: new Box3DSoaParticleCapacities(-1, 0, 0, 0, 0, 0)));
    }

    // ---- determinism --------------------------------------------------------------------------------------

    private static Box3DSoaWorld PileScene(bool nudge = false)
    {
        var world = World(bodies: 8, particles: 192, distances: 256, bends: 256, attachments: 8, tethers: 256);
        world.AddStaticBox(1, (0f, -0.5f, 0f), (20f, 0.5f, 20f), friction: 0.6f);
        world.SetMaterial(0, 0.3f);
        var sphere = world.AddSphere(2, (0.2f, 0.35f, 0.1f), 0.15f, 800f);
        world.AddBox(3, (-0.3f, 0.2f, -0.1f), (0.2f, 0.1f, 0.2f), 300f);
        var ropes = new Made[4];
        for (var c = 0; c < 4; c++)
        {
            var from = (-0.5f + 0.1f * c, 0.5f + 0.06f * c, -0.3f + 0.2f * c);
            var step = c % 2 == 0 ? (Spacing, 0f, 0.012f * c) : (0.01f * c, 0f, Spacing);
            var line = Line(from, step, 24);
            if (nudge && c == 2) line[7].Y = BitConverter.UInt32BitsToSingle(BitConverter.SingleToUInt32Bits(line[7].Y) + 1);
            ropes[c] = Graph.Rope(world, line, ParticleMass, ParticleRadius, Box3DSoaPhase.Of((uint)(c + 1), true, true));
        }
        Graph.Sheet(world, (0.1f, 0.9f, -0.2f), (0.03f, 0f, 0f), (0f, 0f, 0.03f), 8, 8, ParticleMass, ParticleRadius, Box3DSoaPhase.Of(9, true, true));
        world.Attach(Box3DSoaAttachKind.Body, ropes[3].Particles[23], sphere, (0f, 0.15f, 0f), tether: true); // a rope tied to a body on the pile
        world.Attach(Box3DSoaAttachKind.Static, ropes[0].Particles[0], point: (-0.5f, 0.5f, -0.3f), tether: true);
        return world;
    }

    private static ulong HashParticles(Box3DSoaView v)
    {
        var h = 1469598103934665603ul;
        void F(ReadOnlySpan<float> lane) => h = Fnv(h, MemoryMarshal.AsBytes(lane));
        F(v.ParticleX); F(v.ParticleY); F(v.ParticleZ); F(v.ParticleVx); F(v.ParticleVy); F(v.ParticleVz); F(v.DistanceTension);
        F(v.AttachmentForceX); F(v.AttachmentForceY); F(v.AttachmentForceZ);
        F(v.PositionX); F(v.PositionY); F(v.PositionZ); F(v.RotationX); F(v.RotationY); F(v.RotationZ); F(v.RotationW);
        F(v.VelocityX); F(v.VelocityY); F(v.VelocityZ); F(v.AngularVelocityX); F(v.AngularVelocityY); F(v.AngularVelocityZ);
        F(v.ParticleImpulseX); F(v.ParticleImpulseY); F(v.ParticleImpulseZ); F(v.TetherBound);
        h = Fnv(h, MemoryMarshal.AsBytes(v.TetherParent));
        h = Fnv(h, MemoryMarshal.AsBytes(v.ParticleComponent));
        return h;
    }

    private static ulong[] Checkpoints(bool nudge = false)
    {
        using var world = PileScene(nudge);
        var view = world.View;
        var hashes = new ulong[6];
        for (var c = 0; c < 6; c++)
        {
            Run(world, 50);
            hashes[c] = HashParticles(view);
        }
        return hashes;
    }

    // Recorded on this commit's build (gcc 13.2, -ffp-contract=off): the pile of four ropes and a sheet on two bodies. Re-pinned when each particle took
    // one tether to its nearest tether point (the hash covers the tether lanes) and pair contact summed in a new order; behaviour is held by the tests around it.
    private static readonly ulong[] PileHashes = [12993313564863515222ul, 2307790533006845688ul, 13436656559607346955ul, 6468437088969606470ul, 10562521318003626344ul, 7711170181992570593ul];

    [Fact]
    public void Particle_world_is_byte_deterministic()
    {
        var a = Checkpoints();
        var b = Checkpoints();
        Assert.True(a.AsSpan().SequenceEqual(b), "two runs diverged: " + string.Join(",", a) + " vs " + string.Join(",", b));
        Assert.True(a.Distinct().Count() == 6, "the scene moves");
        Assert.False(a.AsSpan().SequenceEqual(Checkpoints(nudge: true)), "a one-ulp input difference must diverge");
        Assert.True(PileHashes.AsSpan().SequenceEqual(a), string.Join(",", a.Select(g => g + "ul")));
    }

    [Fact]
    public async Task Particle_world_is_the_same_bytes_on_any_thread_under_load()
    {
        // The solver is single-threaded by construction; what is provable is that nothing in it depends on which thread
        // runs it or what else is running: eight worlds at once give the serial bytes.
        var serial = Checkpoints();
        var results = Enumerable.Range(0, 8).Select(_ => Task.Factory.StartNew(() => Checkpoints(), TaskCreationOptions.LongRunning)).ToArray();
        await Task.WhenAll(results);
        foreach (var r in results) Assert.True(Enumerable.SequenceEqual(serial, await r), "a concurrent run diverged from the serial one");
    }

    [Fact]
    public void Particle_step_and_commands_do_not_allocate()
    {
        using var world = PileScene();
        var view = world.View;
        Run(world, 20);
        var spare = new Box3DSoaParticleHandle[2];
        var fresh = new[] { new Box3DSoaParticleDesc((3f, 3f, 3f), 0.02f, 0.0125f), new Box3DSoaParticleDesc((3f, 3.03f, 3f), 0.02f, 0.0125f) };
        var edge = new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Distance, 10, view.DistanceGeneration[10]);
        var victim = new[] { new Box3DSoaParticleHandle(100, view.ParticleGeneration[100]) };
        using (var warm = PileScene()) // the same commands once on another world, so the JIT is not measured
        {
            var wv = warm.View;
            Run(warm, 2);
            warm.Remove(new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Distance, 10, wv.DistanceGeneration[10]));
            warm.DestroyParticles([new Box3DSoaParticleHandle(100, wv.ParticleGeneration[100])]);
            Run(warm, 2);
            warm.CreateParticles(fresh, spare);
            Run(warm, 2);
        }
        var sink = 0f;
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < 100; i++)
        {
            world.Step(Dt, Sub);
            sink += view.ParticleY[3] + view.DistanceTension[1] + view.ParticleImpulseX[2] + view.AttachmentForceY[0];
        }
        world.Remove(edge); // a cut
        world.DestroyParticles(victim); // a destroy
        world.Step(Dt, Sub);
        world.Step(Dt, Sub);
        world.CreateParticles(fresh, spare); // a create
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.True(sink != 0f);
        Assert.Equal(0L, allocated);
    }

    [Fact]
    public void Batches_over_the_stack_limit_do_not_allocate()
    {
        // 300 distances, 299 bends, removed whole: past 256 the handle scratch comes from a pool, not the heap.
        void Cycle(Box3DSoaWorld world, Box3DSoaParticleHandle[] particles, Box3DSoaDistanceDesc[] d, Box3DSoaConstraintHandle[] edges,
                   Box3DSoaBendDesc[] b, Box3DSoaConstraintHandle[] bends, bool refreshBends)
        {
            world.CreateDistances(d, edges);
            if (refreshBends) for (var i = 0; i < b.Length; i++) b[i] = new Box3DSoaBendDesc(edges[i], edges[i + 1], 1e-4f);
            world.CreateBends(b, bends);
            world.Remove(bends);
            world.Remove(edges);
        }
        (Box3DSoaWorld, Box3DSoaParticleHandle[], Box3DSoaDistanceDesc[], Box3DSoaConstraintHandle[], Box3DSoaBendDesc[], Box3DSoaConstraintHandle[]) Build()
        {
            var world = World(particles: 320, distances: 320, bends: 320, tethers: 8);
            var particles = new Box3DSoaParticleHandle[301];
            world.CreateParticles(Enumerable.Range(0, 301).Select(i => new Box3DSoaParticleDesc((i * 0.03f, 0f, 0f), ParticleMass, ParticleRadius)).ToArray(), particles);
            var d = Enumerable.Range(0, 300).Select(i => new Box3DSoaDistanceDesc(particles[i], particles[i + 1], 0f)).ToArray();
            return (world, particles, d, new Box3DSoaConstraintHandle[300], new Box3DSoaBendDesc[299], new Box3DSoaConstraintHandle[299]);
        }
        var (warm, wp, wd, we, wb, wbh) = Build();
        using (warm) Cycle(warm, wp, wd, we, wb, wbh, true); // the pool and the JIT, off the measured world
        var (world, p, dd, e, bb, bh) = Build();
        using (world)
        {
            world.CreateDistances(dd, e);
            for (var i = 0; i < bb.Length; i++) bb[i] = new Box3DSoaBendDesc(e[i], e[i + 1], 1e-4f);
            world.Remove(e); // the lanes come free after a step; the measured cycle then reuses the same arrays
            world.Step(Dt, Sub);
            var before = GC.GetAllocatedBytesForCurrentThread();
            Cycle(world, p, dd, e, bb, bh, true);
            var allocated = GC.GetAllocatedBytesForCurrentThread() - before;
            Assert.Equal(0L, allocated);
        }
    }

    // ---- NO_PARTICLE_CONTACT: lodur:ruling:ymir-flesh-meets-bodies-and-fluid -------------------------------------------------

    /// <summary>Two ropes of different groups across each other, the lower held taut at y 0.5 along x, the upper dropped on it along z. The closest cross pair over 2 s, in units of ri + rj.</summary>
    private static float RopesClosest(bool upperFlesh)
    {
        using var world = World(gravityY: -10f, attachments: 8);
        var lower = Line((-0.3f, 0.5f, 0f), (Spacing, 0f, 0f), 21);
        var a = Graph.Rope(world, lower, ParticleMass, ParticleRadius, Box3DSoaPhase.Of(1), bend: -1f);
        world.Attach(Box3DSoaAttachKind.Static, a.Particles[0], point: lower[0], tether: true);
        world.Attach(Box3DSoaAttachKind.Static, a.Particles[20], point: lower[20], tether: true);
        var b = Graph.Rope(world, Line((0f, 0.53f, -0.3f), (0f, 0f, Spacing), 21), ParticleMass, ParticleRadius,
            Box3DSoaPhase.Of(2, noParticleContact: upperFlesh), bend: -1f);
        var closest = float.MaxValue;
        for (var step = 0; step < 240; step++)
        {
            world.Step(Dt, Sub);
            var v = world.View;
            foreach (var i in a.Particles) foreach (var j in b.Particles)
                closest = MathF.Min(closest, Graph.Dist(v, i.Lane, j.Lane) / (2f * ParticleRadius));
        }
        return closest;
    }

    [Fact]
    public void A_no_particle_contact_rope_passes_through_a_rope_of_another_group()
    {
        // The closest cross pair over 2 s, in ri + rj, deterministic readings; bounds per lodur:ruling:deterministic-bound-margin.
        // Without the bit (a dropped rope rests on the other): shipped 0.759, least claimed mutant Mg (walk_half hashes with table_mask while grid_sort used
        // grid_mask: the walk misses most pairs) 0.045: bound sqrt(0.759 x 0.045) = 0.185, separation 16.9. With the bit: shipped 0.126, least claimed mutant
        // Mn (the bit ignored: grid_sort, build_grid and walk_half all treat the lane as solid) 0.759: bound sqrt(0.126 x 0.759) = 0.309, separation 6.0.
        // Mw (walk_half's skip alone removed) reads 0.126 where the lane's stale cell lands on nothing, and 0.063 with its cells written (it then walks only the
        // forward half of its pairs): it does not separate here, so it is unclaimed in this test; the skip is what keeps a lane's stale cell out of the walk, and
        // Flesh_lane_with_a_stale_cell_pushes_nothing claims it. A resting rope read 0.759, not 0.95: soft contact under the rope's weight, not this cut.
        var contact = RopesClosest(false);
        var through = RopesClosest(true);
        var line = $"FLESH_PASS closest cross pair in ri + rj: without the bit {contact:F3}, with it on the dropped rope {through:F3}";
        Console.Error.WriteLine(line);
        Readings.Add(line);
        Assert.True(contact > 0.185f, "two ropes of different groups did not stop each other: " + line);
        Assert.True(through < 0.309f, "a rope with the bit did not pass through: " + line);
    }

    /// <summary>Two particles of different groups 0.010 m apart (0.4 of ri + rj) in cell (0, 0, 0), no gravity, one carrying the bit. The pair's separation after 10 ticks, in ri + rj.</summary>
    private static float StaleCellSeparation(bool fleshFirst)
    {
        using var world = World(gravityY: 0f);
        var flesh = Box3DSoaPhase.Of(1, noParticleContact: true);
        var plain = Box3DSoaPhase.Of(2);
        var p = new Box3DSoaParticleHandle[2];
        world.CreateParticles([new((0.001f, 0.001f, 0.001f), ParticleMass, ParticleRadius, fleshFirst ? flesh : plain), new((0.011f, 0.001f, 0.001f), ParticleMass, ParticleRadius, fleshFirst ? plain : flesh)], p);
        Run(world, 10);
        return Graph.Dist(world.View, p[0].Lane, p[1].Lane) / (2f * ParticleRadius);
    }

    [Fact]
    public void A_flesh_lane_is_not_walked_by_the_particle_contact_pass()
    {
        // A lane with the bit is in no grid cell, so its cell lanes keep their creation value (0, 0, 0). If the walk visited it, it would find the other
        // particle in that cell and push it. Readings: shipped (the walk skips it) 0.400 (the start, untouched); walk_half's skip removed (the lane walks
        // its stale cell) 24.400: bound sqrt(0.400 x 24.4) = 3.12, separation 61. Lane order matters: the walk pairs a lane with higher lanes of its cell,
        // so the flesh lane is the lower one here.
        var line = $"FLESH_WALK separation after 10 ticks in ri + rj: flesh lane first {StaleCellSeparation(true):F3}";
        Console.Error.WriteLine(line);
        Readings.Add(line);
        Assert.True(StaleCellSeparation(true) < StaleCellBound, "the flesh lane pushed a particle of another group: " + line);
    }

    private const float StaleCellBound = 3.12f;

    /// <summary>A 12 x 12 sheet of touching particles (pitch 2 r) carrying the flag, held at its four corners by static pins, at y 0.5.</summary>
    private static (Box3DSoaWorld World, Made Sheet) FleshSheet(int fluid)
    {
        var world = World(particles: 144 + 216 + 8, distances: 400, bends: 400, attachments: 8, tethers: 8, fluid: fluid);
        var sheet = Graph.Sheet(world, (-0.1375f, 0.5f, -0.1375f), (2f * ParticleRadius, 0f, 0f), (0f, 0f, 2f * ParticleRadius), 12, 12, ParticleMass, ParticleRadius,
            Box3DSoaPhase.Of(1, true, true, noParticleContact: true));
        foreach (var i in new[] { 0, 11, 132, 143 })
        {
            var v = world.View;
            world.Attach(Box3DSoaAttachKind.Static, sheet.Particles[i], point: (v.ParticleX[i], v.ParticleY[i], v.ParticleZ[i]));
        }
        return (world, sheet);
    }

    [Fact]
    public void A_no_particle_contact_sheet_still_meets_fluid_and_bodies()
    {
        // Mf (the bit also left out of the fluid grid) reads 216 of 216 through; shipped 0. Mn (the bit ignored) reads 0 here, as this scene does not
        // see particle contact: not a mutant of this test.
        // Fluid: a 6 x 6 x 6 block poured on the sheet. A flat sheet has no walls, so the block spreads off its edge (that part falls for good): only
        // fluid over the sheet's interior (a pitch in from every edge) counts, and none of it may be below the sheet's lowest particle after 2 s.
        var (world, sheet) = FleshSheet(216);
        int through;
        using (world)
        {
            world.SetMaterial(1, 0.3f, 0, 0.01f);
            var block = Graph.Block(world, (-0.0625f, 0.56f, -0.0625f), 6, 6, 6, ParticleRadius, 0.015625f, Box3DSoaPhase.Of(2, fluid: true), material: 1);
            Run(world, 240);
            var v = world.View;
            var plane = sheet.Particles.Min(h => v.ParticleY[h.Lane]);
            const float edge = 5.5f * 2f * ParticleRadius - 2f * ParticleRadius;
            through = block.Count(h => v.ParticleY[h.Lane] < plane && MathF.Abs(v.ParticleX[h.Lane]) < edge && MathF.Abs(v.ParticleZ[h.Lane]) < edge);
            var line = $"FLESH_FLUID over the sheet's interior, below its lowest particle ({plane:F3}) after 2 s: {through} of {block.Length} (off the edge: {block.Count(h => v.ParticleY[h.Lane] < plane) - through})";
            Console.Error.WriteLine(line);
            Readings.Add(line);
        }
        Assert.Equal(0, through);

        // Body: a sphere dropped on the sheet rests on it, its centre within r + 2 particle radii of the sheet particle under it.
        var (bodyWorld, bodySheet) = FleshSheet(0);
        using (bodyWorld)
        {
            const float r = 0.04f;
            var sphere = bodyWorld.AddSphere(100, (0f, 0.6f, 0f), r, 1000f);
            Run(bodyWorld, 240);
            var v = bodyWorld.View;
            var under = bodySheet.Particles.OrderBy(h => Graph.Sq(v.ParticleX[h.Lane] - v.PositionX[sphere]) + Graph.Sq(v.ParticleZ[h.Lane] - v.PositionZ[sphere])).First();
            var gap = v.PositionY[sphere] - v.ParticleY[under.Lane];
            var line = $"FLESH_BODY sphere centre above the sheet particle under it: {gap:F4} (r {r}, bound r + 2 particle radii {r + 2f * ParticleRadius:F4})";
            Console.Error.WriteLine(line);
            Readings.Add(line);
            Assert.True(gap > 0f && gap <= r + 2f * ParticleRadius, "the sphere did not rest on the sheet: " + line);
        }
    }

    [Fact]
    public void Fluid_with_no_particle_contact_is_refused_and_creates_nothing()
    {
        using var world = World(fluid: 4);
        Assert.Throws<InvalidOperationException>(() => world.CreateParticles(
        [
            new((0f, 0f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(1)),
            new((0.1f, 0f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(1, fluid: true, noParticleContact: true)),
        ]));
        Assert.Equal(0, world.View.ParticleCount);
        world.CreateParticles([new((0f, 0f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(1, noParticleContact: true)), new((0.1f, 0f, 0f), 0.02f, 0.0125f, Box3DSoaPhase.Of(1, fluid: true))]);
        Assert.Equal(2, world.View.ParticleCount);
    }

    /// <summary>Four 16 x 16 sheets, each wrapped round a capsule and attached to it at every particle, no gravity; ms per tick over 120 ticks.</summary>
    private static double FleshCost(bool flesh)
    {
        using var world = World(bodies: 8, particles: 4 * 256 + 8, distances: 4 * 496 + 8, bends: 8, attachments: 4 * 256 + 8, tethers: 8, gravityY: 0f);
        const float radius = 0.0765f;
        for (var s = 0; s < 4; s++)
        {
            var body = world.AddCapsule((ulong)(10 + s), (0f, 0f, 0.4f * s), radius, 0.225f, 100f, rotation: (0f, 0f, 0.70710678f, 0.70710678f)); // local Y along world -X
            var desc = new Box3DSoaParticleDesc[256];
            var local = new (float X, float Y, float Z)[256];
            for (var j = 0; j < 16; j++) for (var i = 0; i < 16; i++)
            {
                var th = 2f * MathF.PI * i / 16f;
                var along = -0.225f + Spacing * j;
                desc[j * 16 + i] = new Box3DSoaParticleDesc((-along, radius * MathF.Cos(th), 0.4f * s + radius * MathF.Sin(th)), ParticleMass, ParticleRadius,
                    Box3DSoaPhase.Of((uint)(s + 1), noParticleContact: flesh));
                local[j * 16 + i] = (radius * MathF.Sin(th), along, radius * MathF.Cos(th));
            }
            var p = new Box3DSoaParticleHandle[256];
            world.CreateParticles(desc, p);
            var d = new List<Box3DSoaDistanceDesc>();
            for (var j = 0; j < 16; j++) for (var i = 0; i < 16; i++)
            {
                if (j + 1 < 16) d.Add(new(p[j * 16 + i], p[(j + 1) * 16 + i]));
                d.Add(new(p[j * 16 + i], p[j * 16 + (i + 1) % 16]));
            }
            world.CreateDistances(d.ToArray());
            for (var k = 0; k < 256; k++) world.Attach(Box3DSoaAttachKind.Body, p[k], body, local[k]);
        }
        Run(world, 30);
        var watch = System.Diagnostics.Stopwatch.StartNew();
        Run(world, 120);
        return watch.Elapsed.TotalMilliseconds / 120;
    }

    [Fact]
    public void Flesh_cost_line()
    {
        // Printed, not bounded: probe P7 read a live Fella at 5.44 ms in the solid contact walk and 1.04 ms out of it.
        var off = new double[5];
        var on = new double[5];
        FleshCost(true); FleshCost(false); // warm both
        for (var k = 0; k < 5; k++) { off[k] = FleshCost(false); on[k] = FleshCost(true); }
        Array.Sort(off); Array.Sort(on);
        var line = $"PARTICLE_COST four 16 x 16 sheets, every particle attached to a capsule, 4 substeps, median of 5 alternated runs: without NO_PARTICLE_CONTACT {off[2]:F3} ms per tick, with it {on[2]:F3} ms per tick";
        Console.Error.WriteLine(line);
        Readings.Add(line);
        Assert.True(off[2] > 0 && on[2] > 0);
    }
}

/// <summary>The structural promises of the particle module, read from its source.</summary>
public sealed class Box3DSoaParticleSourceTests
{
    private static string Root()
    {
        var dir = AppContext.BaseDirectory;
        while (dir != null && !File.Exists(Path.Combine(dir, "native", "Ymir.Box3D", "ymir_particles.c"))) dir = Path.GetDirectoryName(dir);
        return dir ?? throw new FileNotFoundException("native/Ymir.Box3D");
    }

    private static string Native() => Path.Combine(Root(), "native", "Ymir.Box3D");

    /// <summary>The text of a function from its signature line to its closing brace at column 0.</summary>
    private static string Body(string source, string signature)
    {
        var start = source.IndexOf(signature, StringComparison.Ordinal);
        while (start >= 0 && source[(start + signature.Length)..].TrimStart().StartsWith(';')) // a prototype, not the definition
            start = source.IndexOf(signature, start + 1, StringComparison.Ordinal);
        Assert.True(start >= 0, signature);
        var open = source.IndexOf("\n{", start, StringComparison.Ordinal);
        var end = source.IndexOf("\n}", open, StringComparison.Ordinal);
        return source.Substring(start, end - start + 2);
    }

    private static string StripComments(string s) => Regex.Replace(s, @"//[^\n]*|/\*.*?\*/", "", RegexOptions.Singleline);

    private static int Count(string text, string pattern) => Regex.Matches(text, pattern).Count;

    private static int Calls(string name) =>
        Directory.GetFiles(Native(), "*.c").Where(f => !f.EndsWith("ymir_particles.c")).Concat(Directory.GetFiles(Native(), "*.h").Where(f => !f.EndsWith("ymir_particles.h")))
            .Sum(f => Count(File.ReadAllText(f), name + @"\("));

    [Fact]
    public void Particle_source_rules()
    {
        var c = File.ReadAllText(Path.Combine(Native(), "ymir_particles.c"));
        var h = File.ReadAllText(Path.Combine(Native(), "ymir_particles.h"));
        var code = StripComments(c + h);
        Assert.Equal(0, Count(c + h, @"b3[A-Z_]|box3d\.h"));
        var outside = c.Replace(Body(c, "ymir_particles* ymir_particles_create("), "").Replace(Body(c, "void ymir_particles_free("), "");
        Assert.Equal(0, Count(outside, @"malloc|calloc|realloc|free\("));
        Assert.Equal(0, Count(c, @"\bomp\b|#pragma|pthread|_Atomic|atomic_"));
        // no object kind: a rope, a sheet, a cloth or a chain is what a caller builds, never a name in the solver or its binding
        var bindings = string.Join("\n", Directory.GetFiles(Path.Combine(Root(), "src", "Ymir.Box3D"), "*.cs").Select(f => StripComments(File.ReadAllText(f))));
        Assert.Equal(0, Count(code + bindings, @"(?<![A-Za-z])(rope|sheet|cloth|chain|blood|water|Rope|Sheet|Cloth|Chain|Blood|Water)(?![a-z])"));
        // tether lanes and the component lane have one writer: the derivation
        var outsideWriters = c.Replace(Body(c, "static int32_t rederive_topology("), "");
        var rest = StripComments(outsideWriters);
        Assert.Equal(0, Count(rest, @"->(component|tether_att|tether_particle|tether_parent|tether_edge|tether_bound)\[[^\]]*\]\s*=[^=]|->tether_count\s*=[^=]"));
        // every call names a mode; only a command that changed no distance and no attachment may derive the isolated lanes
        Assert.Equal(0, Count(c, @"rederive_topology\(p\)"));
        foreach (var command in new[] { "create_particles", "destroy_particles", "create_distances", "create_bends", "attach", "remove" })
            Assert.Contains("rederive_topology(p, ", Body(c, $"int32_t ymir_particles_{command}("));
        foreach (var command in new[] { "create_distances", "create_bends", "attach", "remove" })
            Assert.DoesNotContain("TOPO_ISOLATED", Body(c, $"int32_t ymir_particles_{command}("));
        // emission derives the isolated lanes only (a new particle holds no constraint); destroy names both, the graph changing when a victim held one
        var created = Body(c, "int32_t ymir_particles_create_particles(");
        Assert.Contains("TOPO_ISOLATED", created);
        Assert.DoesNotContain("TOPO_FULL", created);
        var destroyed = Body(c, "int32_t ymir_particles_destroy_particles(");
        Assert.Contains("TOPO_ISOLATED", destroyed);
        Assert.Contains("TOPO_FULL", destroyed);
        Assert.Equal(0, Count(c + h, @"PxReal|physx"));
        // the retired owners stay gone
        var retired = string.Join("|", "create_" + "chain", "split_" + "chain", "destroy_" + "chain", "chain_" + "first", "chain_" + "size", "chain_stable_" + "id",
            "Chain" + "Handle", "Create" + "Chain", "Split" + "Chain", "Destroy" + "Chain", "FreeChain" + "Slots", "link_" + "tension", "Link" + "Tension");
        foreach (var dir in new[] { "native", "src", "tests", "docs" })
            foreach (var f in Directory.GetFiles(Path.Combine(Root(), dir), "*.*", SearchOption.AllDirectories).Where(f => !f.Contains("obj") && !f.Contains("bin") && (f.EndsWith(".c") || f.EndsWith(".h") || f.EndsWith(".cs") || f.EndsWith(".md"))))
                Assert.True(Count(File.ReadAllText(f), retired) == 0, f);
        Assert.Equal(0, Count(c + h, @"PARTICLE_" + @"PARTICLE|find_range|mark_slots|stable_id_taken|rest_pos|add_struct"));
        Assert.Equal(0, Count(c, @"1\.0f / \(float\)contacts"));
        // no fluid path runs in a world with no live fluid: both passes sit behind the counter
        var substep = StripComments(Body(c, "void ymir_particles_substep("));
        Assert.Equal(1, Count(substep, @"if \(p->live_fluid > 0\)\s*\{\s*fluid_density\(p\);"));
        Assert.Equal(1, Count(substep, @"if \(p->live_fluid > 0 && viscous_material\(p\)\)\s*\{\s*fluid_viscosity\(p\);"));
        // fluid has one writer of live_fluid pair: the two commands
        Assert.Equal(0, Count(StripComments(c.Replace(Body(c, "int32_t ymir_particles_create_particles("), "").Replace(Body(c, "int32_t ymir_particles_destroy_particles("), "")), @"live_fluid\s*(\+\+|--|[-+]?=[^=])"));
    }

    [Fact]
    public void The_particle_phase_runs_in_the_substep_callback_and_never_removes()
    {
        var soa = File.ReadAllText(Path.Combine(Native(), "ymir_box3d_soa.c"));
        var c = File.ReadAllText(Path.Combine(Native(), "ymir_particles.c"));
        // begin and end once per tick, in the step; the substep once, in the callback Box3D runs inside its own substeps
        Assert.Equal(1, Calls("ymir_particles_begin"));
        Assert.Equal(1, Calls("ymir_particles_end"));
        Assert.Equal(1, Calls("ymir_particles_substep"));
        var step = Body(soa, "YMIR_EXPORT int32_t YMIR_CALL ymir_box3d_soa_step(");
        var callback = Body(soa, "static void ymir_soa_particle_substep(b3WorldId world_id, int sub_step, float h, void* context)");
        Assert.Contains("ymir_particles_begin(", step);
        Assert.Contains("ymir_particles_end(", step);
        Assert.Contains("ymir_particles_substep(", callback);
        Assert.Equal(1, Count(soa, @"b3World_SetSubStepCallback\("));
        Assert.Contains("live_particles == 0", callback);
        // a body is touched only through the callback's two fork accessors: never pushed after the step, never teleported
        Assert.Equal(0, Count(step + callback, @"b3Body_ApplyLinearImpulse|b3Body_ApplyAngularImpulse|b3Body_SetTransform"));
        Assert.Equal(1, Count(soa, @"b3Body_GetSubStepState\("));
        Assert.Equal(1, Count(soa, @"b3Body_ApplySubStepImpulse\("));
        Assert.Contains("b3Body_GetSubStepState(", callback);
        Assert.Contains("b3Body_ApplySubStepImpulse(", callback);
        // no topology change inside the step
        var phases = new[] { Body(c, "int32_t ymir_particles_begin("), Body(c, "void ymir_particles_substep("), Body(c, "void ymir_particles_end(") };
        foreach (var text in phases.Append(step).Append(callback))
            Assert.Equal(0, Count(text, @"rederive_topology|kill_|ymir_particles_(create|destroy|remove|attach)"));
        Assert.Equal(0, Count(c, @"carry|hard_impulse|cancel|k_body|end_force_scale|tension_diff|add_link_load|ymir_particles_step"));
        Assert.Equal(0, Count(soa, @"carry|ymir_particles_step|ymir_soa_apply_particle_impulses"));
    }

    [Fact]
    public void The_solid_grid_is_sized_by_its_members_and_the_fluid_grid_is_not()
    {
        var c = StripComments(File.ReadAllText(Path.Combine(Native(), "ymir_particles.c")));
        Assert.Equal(0, Count(Body(c, "static void grid_sort("), @"table_mask"));
        Assert.Equal(0, Count(Body(c, "YMIR_INLINE void walk_half("), @"table_mask"));
        Assert.Equal(1, Count(c, @"grid_mask\s*="));
        Assert.Contains("grid_mask =", Body(c, "static void build_grid("));
        Assert.Contains("grid_sort(p, inv_cell, 0, p->table_mask)", Body(c, "static float fluid_grid("));
    }
}
