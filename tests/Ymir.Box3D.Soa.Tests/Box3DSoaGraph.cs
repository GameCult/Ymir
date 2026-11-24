using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>What a builder made: the particles, the distances between them and the bends over pairs of distances.</summary>
internal sealed record Made(Box3DSoaParticleHandle[] Particles, Box3DSoaConstraintHandle[] Edges, Box3DSoaConstraintHandle[] Bends);

/// <summary>
/// Test-local builders. Ymir has no rope or sheet: both are only the constraints these functions emit through the
/// public commands, so the tests exercise exactly the surface a game would.
/// </summary>
internal static class Graph
{
    public static Pos[] Line(Pos from, Pos step, int count)
    {
        var p = new Pos[count];
        for (var i = 0; i < count; i++) p[i] = (from.X + step.X * i, from.Y + step.Y * i, from.Z + step.Z * i);
        return p;
    }

    /// <summary>Particles at <paramref name="positions"/> (rest pose <paramref name="rest"/> or the same), a distance between neighbours and a bend over each pair of distances.</summary>
    public static Made Rope(
        Box3DSoaWorld world, Pos[] positions, float mass, float radius, uint phase = 0, float stretch = 0f, float bend = 1e-4f,
        float maxAngle = -1f, Pos[]? rest = null, float[]? masses = null, Pos velocity = default, uint material = 0)
    {
        var n = positions.Length;
        var desc = new Box3DSoaParticleDesc[n];
        for (var i = 0; i < n; i++)
            desc[i] = new Box3DSoaParticleDesc(positions[i], masses?[i] ?? mass, radius, phase, material, rest?[i], velocity);
        var particles = new Box3DSoaParticleHandle[n];
        world.CreateParticles(desc, particles);
        var edges = new Box3DSoaConstraintHandle[n - 1];
        var d = new Box3DSoaDistanceDesc[n - 1];
        for (var i = 0; i + 1 < n; i++) d[i] = new Box3DSoaDistanceDesc(particles[i], particles[i + 1], stretch);
        world.CreateDistances(d, edges);
        var bends = Array.Empty<Box3DSoaConstraintHandle>();
        if (bend >= 0f || maxAngle >= 0f)
        {
            bends = new Box3DSoaConstraintHandle[Math.Max(0, n - 2)];
            var b = new Box3DSoaBendDesc[bends.Length];
            for (var i = 0; i + 2 < n; i++) b[i] = new Box3DSoaBendDesc(edges[i], edges[i + 1], bend, maxAngle);
            world.CreateBends(b, bends);
        }
        return new Made(particles, edges, bends);
    }

    /// <summary>An <paramref name="nu"/> x <paramref name="nv"/> grid: distances along both axes, bends along both.</summary>
    public static Made Sheet(
        Box3DSoaWorld world, Pos origin, Pos du, Pos dv, int nu, int nv, float mass, float radius, uint phase = 0,
        float stretch = 0f, float bend = 1e-4f)
    {
        var desc = new Box3DSoaParticleDesc[nu * nv];
        for (var j = 0; j < nv; j++)
            for (var i = 0; i < nu; i++)
                desc[j * nu + i] = new Box3DSoaParticleDesc(
                    (origin.X + du.X * i + dv.X * j, origin.Y + du.Y * i + dv.Y * j, origin.Z + du.Z * i + dv.Z * j), mass, radius, phase);
        var p = new Box3DSoaParticleHandle[nu * nv];
        world.CreateParticles(desc, p);
        var d = new List<Box3DSoaDistanceDesc>();
        for (var j = 0; j < nv; j++) for (var i = 0; i + 1 < nu; i++) d.Add(new(p[j * nu + i], p[j * nu + i + 1], stretch));
        var uCount = d.Count;
        for (var j = 0; j + 1 < nv; j++) for (var i = 0; i < nu; i++) d.Add(new(p[j * nu + i], p[(j + 1) * nu + i], stretch));
        var e = new Box3DSoaConstraintHandle[d.Count];
        world.CreateDistances(d.ToArray(), e);
        var b = new List<Box3DSoaBendDesc>();
        for (var j = 0; j < nv; j++) for (var i = 0; i + 2 < nu; i++) b.Add(new(e[j * (nu - 1) + i], e[j * (nu - 1) + i + 1], bend));
        for (var j = 0; j + 2 < nv; j++) for (var i = 0; i < nu; i++) b.Add(new(e[uCount + j * nu + i], e[uCount + (j + 1) * nu + i], bend));
        var bends = new Box3DSoaConstraintHandle[b.Count];
        world.CreateBends(b.ToArray(), bends);
        return new Made(p, e, bends);
    }

    public static float Dist(Box3DSoaView v, int a, int b) =>
        MathF.Sqrt(Sq(v.ParticleX[a] - v.ParticleX[b]) + Sq(v.ParticleY[a] - v.ParticleY[b]) + Sq(v.ParticleZ[a] - v.ParticleZ[b]));

    public static float Sq(float x) => x * x;

    /// <summary>The summed length of the given distances over their summed rest lengths, minus 1.</summary>
    public static float Stretch(Box3DSoaView v, Made m)
    {
        float length = 0f, rest = 0f;
        foreach (var e in m.Edges)
        {
            length += Dist(v, v.DistanceA[e.Lane], v.DistanceB[e.Lane]);
            rest += v.DistanceRest[e.Lane];
        }
        return length / rest - 1f;
    }

    public static float Dens(float mass, float radius) => mass / (4f / 3f * MathF.PI * radius * radius * radius);
}
