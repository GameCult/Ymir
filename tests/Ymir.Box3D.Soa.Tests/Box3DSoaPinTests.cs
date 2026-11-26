using System.Globalization;
using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

using Pos = (float X, float Y, float Z);

/// <summary>
/// Pinned particles are dormant (docs/soa-world.md, pinned, dormant, kinematic, anchor). The scene is four capsules on a hinge chain, each
/// with a 6 x 6 sheet of flesh attached to it (NO_PARTICLE_CONTACT, edges, bends), and stitches between neighbouring sheets.
/// </summary>
public sealed class Box3DSoaPinTests
{
    private const int Chain = 4, Side = 6, PerSheet = Side * Side;
    private const float Step = 0.05f, Offset = 0.08f, Pitch = 0.32f;

    private static string F(double x) => x.ToString("G6", CultureInfo.InvariantCulture);

    private static void Reading(string line)
    {
        Console.Error.WriteLine(line);
        var path = Environment.GetEnvironmentVariable("YMIR_READINGS");
        if (!string.IsNullOrEmpty(path)) File.AppendAllText(path, line + "\n");
    }

    private sealed class Scene : IDisposable
    {
        public Box3DSoaWorld World = null!;
        public int[] Bones = null!;
        public Box3DSoaParticleHandle[][] P = null!;
        public Pos[][] Local = null!;
        public Box3DSoaConstraintHandle[][] Anchor = null!; // per sheet, per particle: its attachment
        public Box3DSoaConstraintHandle[][] Stitch = null!; // stitch k joins the bottom row of sheet k to the top row of sheet k + 1
        public Box3DSoaParticleHandle[] All = null!;
        public Box3DSoaView View = default!;
        public void Dispose() => World.Dispose();
    }

    private static Scene Chained(int fluid = 0, int extra = 0)
    {
        var s = new Scene
        {
            World = Box3DSoaParticleTests.World(bodies: 8, particles: Chain * PerSheet + 8 + extra, distances: 320, bends: 320, attachments: Chain * PerSheet + 8 + extra,
                tethers: 8, joints: 8, gravityX: 4f, gravityY: -10f, fluid: fluid),
            Bones = new int[Chain], P = new Box3DSoaParticleHandle[Chain][], Local = new Pos[Chain][],
            Anchor = new Box3DSoaConstraintHandle[Chain][], Stitch = new Box3DSoaConstraintHandle[Chain - 1][],
        };
        var w = s.World;
        var top = w.AddStaticBox(1, (0f, 2.05f, 0f), (0.1f, 0.05f, 0.1f));
        var sheets = new Made[Chain];
        for (var k = 0; k < Chain; k++)
        {
            var cy = 1.85f - Pitch * k;
            s.Bones[k] = w.AddCapsule((ulong)(k + 2), (0f, cy, 0f), 0.05f, 0.1f, 500f, parent: k == 0 ? top : s.Bones[k - 1], limb: k);
            w.AddJoint(Box3DSoaJointKind.Revolute, k == 0 ? top : s.Bones[k - 1], s.Bones[k], (0f, k == 0 ? 2f : cy + Pitch / 2f, 0f), (0f, 0f, 1f));
            sheets[k] = Graph.Sheet(w, (-0.125f, cy - 0.125f, Offset), (Step, 0f, 0f), (0f, Step, 0f), Side, Side, 0.02f, 0.0125f,
                Box3DSoaPhase.Of((uint)(k + 1), noParticleContact: true));
            s.P[k] = sheets[k].Particles;
            s.Local[k] = new Pos[PerSheet];
            for (var q = 0; q < PerSheet; q++) s.Local[k][q] = (-0.125f + Step * (q % Side), -0.125f + Step * (q / Side), Offset);
            s.Anchor[k] = new Box3DSoaConstraintHandle[PerSheet];
            for (var q = 0; q < PerSheet; q++) s.Anchor[k][q] = w.Attach(Box3DSoaAttachKind.Body, s.P[k][q], s.Bones[k], s.Local[k][q], compliance: 1e-4f, limb: true, oneWay: true);
        }
        for (var k = 0; k + 1 < Chain; k++)
        {
            var d = new Box3DSoaDistanceDesc[Side];
            for (var i = 0; i < Side; i++) d[i] = new Box3DSoaDistanceDesc(s.P[k][i], s.P[k + 1][Side * (Side - 1) + i]);
            s.Stitch[k] = new Box3DSoaConstraintHandle[Side];
            w.CreateDistances(d, s.Stitch[k]);
        }
        s.All = s.P.SelectMany(x => x).ToArray();
        s.View = w.View;
        return s;
    }

    private static Pos Rotate(float x, float y, float z, float w, Pos v)
    {
        var tx = 2f * (y * v.Z - z * v.Y); var ty = 2f * (z * v.X - x * v.Z); var tz = 2f * (x * v.Y - y * v.X);
        return (v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
    }

    /// <summary>The world point the particle's attachment holds now, and the velocity of that point.</summary>
    private static (Pos Point, Pos Velocity) AnchorNow(Scene s, int sheet, int q)
    {
        var v = s.View;
        var b = s.Bones[sheet];
        var r = Rotate(v.RotationX[b], v.RotationY[b], v.RotationZ[b], v.RotationW[b], s.Local[sheet][q]);
        var p = (v.PositionX[b] + r.X, v.PositionY[b] + r.Y, v.PositionZ[b] + r.Z);
        var wv = (v.AngularVelocityX[b], v.AngularVelocityY[b], v.AngularVelocityZ[b]);
        var vel = (v.VelocityX[b] + wv.Item2 * r.Z - wv.Item3 * r.Y, v.VelocityY[b] + wv.Item3 * r.X - wv.Item1 * r.Z, v.VelocityZ[b] + wv.Item1 * r.Y - wv.Item2 * r.X);
        return (p, vel);
    }

    private static float Dist(Pos a, Pos b) => MathF.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

    private static Pos At(Box3DSoaView v, int i) => (v.ParticleX[i], v.ParticleY[i], v.ParticleZ[i]);

    private static ulong Fnv(ulong h, ReadOnlySpan<byte> bytes)
    {
        foreach (var b in bytes) { h ^= b; h *= 1099511628211ul; }
        return h;
    }

    private static ulong HashAll(Box3DSoaView v)
    {
        var h = 1469598103934665603ul;
        void F(ReadOnlySpan<float> lane) => h = Fnv(h, MemoryMarshal.AsBytes(lane));
        F(v.ParticleX); F(v.ParticleY); F(v.ParticleZ); F(v.ParticleVx); F(v.ParticleVy); F(v.ParticleVz); F(v.DistanceTension);
        F(v.AttachmentForceX); F(v.AttachmentForceY); F(v.AttachmentForceZ);
        F(v.PositionX); F(v.PositionY); F(v.PositionZ); F(v.RotationX); F(v.RotationY); F(v.RotationZ); F(v.RotationW);
        F(v.VelocityX); F(v.VelocityY); F(v.VelocityZ); F(v.AngularVelocityX); F(v.AngularVelocityY); F(v.AngularVelocityZ);
        F(v.ParticleImpulseX); F(v.ParticleImpulseY); F(v.ParticleImpulseZ);
        h = Fnv(h, MemoryMarshal.AsBytes(v.ParticlePinned));
        return h;
    }

    private static float[][] ParticleLanes(Box3DSoaView v) =>
        [v.ParticleX.ToArray(), v.ParticleY.ToArray(), v.ParticleZ.ToArray(), v.ParticleVx.ToArray(), v.ParticleVy.ToArray(), v.ParticleVz.ToArray()];

    // Every body lane a particle could have moved: pose, velocities.
    private static float[][] BodyLanes(Box3DSoaView v) =>
        [v.PositionX.ToArray(), v.PositionY.ToArray(), v.PositionZ.ToArray(), v.RotationX.ToArray(), v.RotationY.ToArray(), v.RotationZ.ToArray(), v.RotationW.ToArray(),
         v.VelocityX.ToArray(), v.VelocityY.ToArray(), v.VelocityZ.ToArray(), v.AngularVelocityX.ToArray(), v.AngularVelocityY.ToArray(), v.AngularVelocityZ.ToArray()];

    private static byte[] Bytes(Box3DSoaView v)
    {
        var all = new List<byte>();
        foreach (var lane in ParticleLanes(v)) all.AddRange(MemoryMarshal.AsBytes(lane.AsSpan()).ToArray());
        all.AddRange(MemoryMarshal.AsBytes(v.ParticlePinned).ToArray());
        return all.ToArray();
    }

    private static void Run(Scene s, int steps) => Box3DSoaParticleTests.Run(s.World, steps);

    // ---- (a) ------------------------------------------------------------------------------------------------

    [Fact]
    public void All_pinned_costs_nothing_and_moves_nothing()
    {
        using var a = Chained();
        using var b = Chained();
        Run(a, 20); Run(b, 20);
        Assert.True(a.View.ParticlePinned.ToArray().All(x => x == 0u));
        var pos = ParticleLanes(a.View);
        var swing0 = (a.View.PositionX[a.Bones[3]], a.View.PositionY[a.Bones[3]]);
        a.World.SetPinned(a.All, true);
        b.World.DestroyParticles(b.All);
        Run(a, 240); Run(b, 240);
        var v = a.View;
        Assert.True(v.ParticlePinned.ToArray().Take(Chain * PerSheet).All(x => x == 1u));
        var now = ParticleLanes(v);
        for (var l = 0; l < 6; l++) Assert.True(MemoryMarshal.AsBytes(now[l].AsSpan()).SequenceEqual(MemoryMarshal.AsBytes(pos[l].AsSpan())), $"pinned lane {l} moved");
        for (var i = 0; i < v.Count; i++) Assert.True(v.ParticleImpulseX[i] == 0f && v.ParticleImpulseY[i] == 0f && v.ParticleImpulseZ[i] == 0f, "a body took an impulse from dormant flesh");
        Assert.True(v.DistanceTension.ToArray().All(t => t == 0f) && Enumerable.Range(0, v.AttachmentCount).All(i => v.AttachmentForceX[i] == 0f && v.AttachmentForceY[i] == 0f && v.AttachmentForceZ[i] == 0f));
        var mine = BodyLanes(v); var theirs = BodyLanes(b.View);
        for (var l = 0; l < mine.Length; l++) Assert.True(MemoryMarshal.AsBytes(mine[l].AsSpan()).SequenceEqual(MemoryMarshal.AsBytes(theirs[l].AsSpan())), $"body lane {l} differs from the world without particles");
        Assert.True(MathF.Abs(v.PositionX[a.Bones[3]] - swing0.Item1) > 0.05f, "the chain swings: " + v.PositionX[a.Bones[3]]);
    }

    // ---- (b) ------------------------------------------------------------------------------------------------

    // Bounds by lodur:ruling:deterministic-bound-margin and deterministic-bound-zero-reading: the geometric mean of the shipped worst and the least claimed
    // mutant, and each separation at least 1.2. The mutants, each a one-line change to ymir_particles.c or ymir_box3d_soa.c, with the readings in the order
    // wake position m / wake velocity m/s / kinematic deviation m / kinematic velocity m/s / stitch length against the all-live world over rest:
    //   shipped  0 / 1.20137e-7 / 2.40274e-7 / 2.07946e-2 / 1.62519e-3
    //   Mk1  the kinematic pos not written at the substep's start     14.9 on kinematic velocity (the one-substep step), the rest as shipped
    //   Mk2  the kinematic pp not written after the proxies predict   1.86 m / 890.667 / 3.87965
    //   Mw   w ignores pinned (a kinematic particle gets weight)      kinematic deviation 9.13432e-3 / velocity 4.3849 / stitch 1.85605e-2
    //   Mv   the wake writes no velocity                              wake velocity 0.695914
    //   Mg   set_pinned does not gather the proxies first             wake position 2.38419e-7 / wake velocity 6.95331e-4
    //   Mr   set_pinned skips rederive_active                         kinematic deviation 9.28324e-3 / velocity 9.27735e-2; (a), (d) fail by name
    //   Mo   act built in descending lane order                       the four determinism hashes fail
    // WakeAnchor: shipped worst 0 (counts half the unit, 1 ulp at 1 to 2 m = 1.19209e-7, so 5.96046e-8), least claimed mutant Mg 2.38419e-7, separation 4; bound sqrt(5.96046e-8 x 2.38419e-7).
    private const float WakeAnchor = 1.2e-7f;
    // WakeVelocity: shipped worst 1.20137e-7, least claimed mutant Mg 6.95331e-4, separation 5788; bound sqrt(1.20137e-7 x 6.95331e-4).
    private const float WakeVelocity = 9.14e-6f;
    // KinAnchor: shipped worst 2.40274e-7, least claimed mutant Mw 9.13432e-3, separation 38016; bound sqrt(2.40274e-7 x 9.13432e-3).
    private const float KinAnchor = 4.685e-5f;
    // KinVelocity: shipped worst 2.07946e-2, least claimed mutant Mr 9.27735e-2, separation 4.46; bound sqrt(2.07946e-2 x 9.27735e-2).
    private const float KinVelocity = 4.39e-2f;
    // StitchDifference: shipped worst 1.62519e-3, least claimed mutant Mw 1.85605e-2, separation 11.4; bound sqrt(1.62519e-3 x 1.85605e-2). Mr reads 2.51e-4 (below shipped): unclaimed here.
    private const float StitchDifference = 5.49e-3f;

    [Fact]
    public void A_woken_sheet_starts_on_its_anchors_and_its_neighbours_are_held_on_theirs()
    {
        using var s = Chained();
        using var live = Chained(); // the same scene with every particle live all along: the stitches' lengths there are the geometry of the hinges
        var w = s.World;
        w.SetPinned(s.All, true);
        Run(s, 30);
        Run(live, 30);
        var v = s.View;
        var dormant = new List<(int Lane, Pos At)>(); // pinned and on no live constraint with an unpinned particle: the far sheet, and the rows of the near ones away from sheet 1
        for (var q = 0; q < PerSheet; q++) dormant.Add((s.P[3][q], At(v, s.P[3][q])));
        for (var q = Side; q < PerSheet; q++) dormant.Add((s.P[0][q], At(v, s.P[0][q])));
        for (var q = 0; q < PerSheet - Side; q++) dormant.Add((s.P[2][q], At(v, s.P[2][q])));
        var asleep = Bytes(v);
        w.SetPinned(s.P[1], false);
        double wakePos = 0, wakeVel = 0;
        for (var q = 0; q < PerSheet; q++)
        {
            var (point, vel) = AnchorNow(s, 1, q);
            wakePos = Math.Max(wakePos, Dist(At(v, s.P[1][q]), point));
            wakeVel = Math.Max(wakeVel, Dist((v.ParticleVx[s.P[1][q]], v.ParticleVy[s.P[1][q]], v.ParticleVz[s.P[1][q]]), vel));
        }
        Assert.True(v.ParticlePinned[s.P[1][0]] == 0u && v.ParticlePinned[s.P[0][0]] == 1u);
        var kin = new List<(int Sheet, int Q)>();
        for (var q = 0; q < Side; q++) { kin.Add((0, q)); kin.Add((2, Side * (Side - 1) + q)); } // bottom row of sheet 0, top row of sheet 2
        double kinDev = 0, kinVel = 0, stretch = 0;
        for (var step = 0; step < 60; step++)
        {
            var sub = step == 0 ? 1 : Box3DSoaParticleTests.Sub; // one substep first: the velocity of a kinematic particle the step leaves is the last substep's, so with four the first one's is overwritten
            s.World.Step(Box3DSoaParticleTests.Dt, sub);
            live.World.Step(Box3DSoaParticleTests.Dt, sub);
            foreach (var (sheet, q) in kin)
            {
                var i = s.P[sheet][q].Lane;
                kinDev = Math.Max(kinDev, Dist(At(v, i), AnchorNow(s, sheet, q).Point));
                kinVel = Math.Max(kinVel, Dist((v.ParticleVx[i], v.ParticleVy[i], v.ParticleVz[i]), AnchorNow(s, sheet, q).Velocity));
            }
            foreach (var k in new[] { 0, 1 })
                foreach (var e in s.Stitch[k])
                    stretch = Math.Max(stretch, MathF.Abs(Dist(At(v, v.DistanceA[e.Lane]), At(v, v.DistanceB[e.Lane])) - Dist(At(live.View, live.View.DistanceA[e.Lane]), At(live.View, live.View.DistanceB[e.Lane]))) / v.DistanceRest[e.Lane]);
        }
        Reading($"PIN_WAKE wake position {F(wakePos)} m, wake velocity {F(wakeVel)} m/s, kinematic deviation after steps {F(kinDev)} m, kinematic velocity against the anchor point's {F(kinVel)} m/s, stitch length differing from the all-live world (over rest) {F(stretch)}");
        Assert.True(wakePos <= WakeAnchor, $"a woken particle is {wakePos} from its anchor");
        Assert.True(wakeVel <= WakeVelocity, $"a woken particle's velocity differs from its anchor point's by {wakeVel}");
        Assert.True(kinDev <= KinAnchor, $"a kinematic particle is {kinDev} from its anchor");
        Assert.True(kinVel <= KinVelocity, $"a kinematic particle moves {kinVel} m/s off its anchor point");
        Assert.True(stretch <= StitchDifference, $"a stitch differs from the all-live world by {stretch}");
        // what no walk reaches did not move: the far sheet, and the dormant rows of the near ones
        foreach (var (lane, was) in dormant) Assert.Equal(was, At(v, lane));
        Assert.False(Bytes(v).SequenceEqual(asleep));
    }

    // ---- (c) ------------------------------------------------------------------------------------------------

    private static unsafe int NativePin(Box3DSoaWorld world, Box3DSoaParticleHandle h, uint pinned)
    {
        var handle = (nint)typeof(Box3DSoaWorld).GetField("_handle", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)!.GetValue(world)!;
        return Box3DSoaNative.ymir_box3d_soa_set_pinned(handle, 1, &h, pinned);
    }

    private static void Refused(Scene s, Action command, string what)
    {
        var before = HashAll(s.View);
        var e = Assert.Throws<InvalidOperationException>(command);
        Assert.Contains("status 1", e.Message);
        Assert.True(before == HashAll(s.View), $"{what}: the refusal changed a lane");
    }

    [Fact]
    public void Pinning_refuses_what_it_cannot_hold_and_changes_nothing()
    {
        using var s = Chained(fluid: 4, extra: 16);
        var w = s.World;
        var fluid = new Box3DSoaParticleHandle[1];
        w.CreateParticles([new Box3DSoaParticleDesc((0.5f, 1f, 0.5f), 0.02f, 0.0125f, Box3DSoaPhase.Of(9, fluid: true))], fluid);
        var bare = new Box3DSoaParticleHandle[1];
        w.CreateParticles([new Box3DSoaParticleDesc((0.5f, 1.5f, 0.5f), 0.02f, 0.0125f, Box3DSoaPhase.Of(8, noParticleContact: true))], bare);
        var rope = Graph.Rope(w, Graph.Line((1f, 1f, 0f), (0.03f, 0f, 0f), 6), 0.02f, 0.0125f, Box3DSoaPhase.Of(7, noParticleContact: true));
        w.Attach(Box3DSoaAttachKind.Static, rope.Particles[0], point: (1f, 1f, 0f), tether: true);
        w.Attach(Box3DSoaAttachKind.Body, rope.Particles[5], s.Bones[0], (0f, 0f, 0.2f)); // an anchor, but the rope's component is tethered
        var doomed = new Box3DSoaParticleHandle[1];
        w.CreateParticles([new Box3DSoaParticleDesc((0.5f, 2f, 0.5f), 0.02f, 0.0125f, Box3DSoaPhase.Of(6, noParticleContact: true))], doomed);
        w.Attach(Box3DSoaAttachKind.Static, doomed[0], point: (0.5f, 2f, 0.5f));
        var view = w.View;
        s.View = view;
        Run(s, 5);
        w.DestroyParticles(doomed);
        Refused(s, () => w.SetPinned(fluid, true), "a fluid particle");
        Refused(s, () => w.SetPinned(bare, true), "no anchor");
        Refused(s, () => w.SetPinned([rope.Particles[5]], true), "a tethered component");
        Refused(s, () => w.SetPinned(doomed, true), "a stale handle");
        Refused(s, () => w.SetPinned([s.P[0][0], fluid[0]], true), "a batch with one bad handle: the good one is not pinned either");
        Assert.Equal(0u, view.ParticlePinned[s.P[0][0]]);
        var beforeTwo = HashAll(view);
        Assert.Equal(1, NativePin(w, s.P[0][0], 2)); // INVALID_ARGUMENT
        Assert.Equal(beforeTwo, HashAll(view));
        // a pinned particle's anchor cannot be removed; another attachment of it can
        w.SetPinned([s.P[0][0], s.P[1][0]], true);
        Assert.Equal(1u, view.ParticlePinned[s.P[0][0]]);
        var extra = w.Attach(Box3DSoaAttachKind.Body, s.P[0][0], s.Bones[0], s.Local[0][0]); // a higher lane: not the anchor
        Refused(s, () => w.Remove(s.Anchor[0][0]), "a pinned particle's anchor");
        w.Remove(extra);
        w.SetPinned([s.P[1][0]], false);
        w.Remove(s.Anchor[1][0]); // woken: its anchor goes like any attachment
        // destroying a pinned particle frees its lane to 0
        var victim = s.P[0][1];
        w.SetPinned([victim], true);
        Assert.Equal(1u, view.ParticlePinned[victim]);
        w.DestroyParticles([victim]);
        Assert.Equal(0u, view.ParticlePinned[victim.Lane]);
    }

    // ---- (d) ------------------------------------------------------------------------------------------------

    /// <summary>The set of particles a step may move, recomputed from the pinned lane and the live constraints: the unpinned, and the pinned ones a live distance or bend with an unpinned particle reaches.</summary>
    private static HashSet<int> MayMove(Box3DSoaView v)
    {
        var set = new HashSet<int>();
        for (var i = 0; i < v.ParticleCount; i++) if (v.ParticleLive[i] != 0 && v.ParticlePinned[i] == 0) set.Add(i);
        var unpinned = set.ToHashSet();
        for (var e = 0; e < v.DistanceCount; e++)
            if (v.DistanceLive[e] != 0 && (unpinned.Contains(v.DistanceA[e]) || unpinned.Contains(v.DistanceB[e]))) { set.Add(v.DistanceA[e]); set.Add(v.DistanceB[e]); }
        for (var k = 0; k < v.BendCount; k++)
            if (v.BendLive[k] != 0 && (unpinned.Contains(v.BendA[k]) || unpinned.Contains(v.BendB[k]) || unpinned.Contains(v.BendC[k]))) { set.Add(v.BendA[k]); set.Add(v.BendB[k]); set.Add(v.BendC[k]); }
        return set;
    }

    private static void AssertLists(Scene s, string after)
    {
        var v = s.View;
        var may = MayMove(v);
        var before = new[] { v.ParticleX.ToArray(), v.ParticleY.ToArray(), v.ParticleZ.ToArray() };
        s.World.Step(Box3DSoaParticleTests.Dt, Box3DSoaParticleTests.Sub);
        for (var i = 0; i < v.ParticleCount; i++)
        {
            var moved = before[0][i] != v.ParticleX[i] || before[1][i] != v.ParticleY[i] || before[2][i] != v.ParticleZ[i];
            Assert.True(moved == may.Contains(i), $"{after}: particle {i} moved {moved}, active or kinematic {may.Contains(i)}");
        }
        var booked = new HashSet<int>(); // an active bend books on both its distances
        for (var k = 0; k < v.BendCount; k++)
            if (v.BendLive[k] != 0 && (v.ParticlePinned[v.BendA[k]] == 0u || v.ParticlePinned[v.BendB[k]] == 0u || v.ParticlePinned[v.BendC[k]] == 0u)) { booked.Add(v.BendE1[k]); booked.Add(v.BendE2[k]); }
        for (var e = 0; e < v.DistanceCount; e++)
            if (v.DistanceLive[e] != 0 && v.ParticlePinned[v.DistanceA[e]] == 1u && v.ParticlePinned[v.DistanceB[e]] == 1u && !booked.Contains(e))
                Assert.True(v.DistanceTension[e] == 0f, $"{after}: an inactive distance {e} carries tension");
        for (var a = 0; a < v.AttachmentCount; a++)
            if (v.AttachmentKind[a] != 2u && v.ParticlePinned[v.AttachmentParticle[a]] == 1u)
                Assert.True(v.AttachmentForceX[a] == 0f && v.AttachmentForceY[a] == 0f && v.AttachmentForceZ[a] == 0f, $"{after}: a dormant attachment {a} carries force");
    }

    [Fact]
    public void The_active_lists_follow_every_wake_and_pin()
    {
        using var s = Chained();
        var w = s.World;
        w.SetPinned(s.All, true);
        AssertLists(s, "all pinned");
        w.SetPinned(s.P[1], false);
        AssertLists(s, "sheet 1 woken");
        w.SetPinned(s.P[2], false);
        AssertLists(s, "sheets 1 and 2 woken");
        w.SetPinned(s.P[1], true);
        AssertLists(s, "sheet 1 pinned again");
        w.SetPinned([s.P[0][7], s.P[3][35]], false);
        AssertLists(s, "two single particles woken");
        w.Remove(s.Stitch[1][2]);
        AssertLists(s, "a stitch cut");
        w.DestroyParticles([s.P[2][3]]);
        AssertLists(s, "a woken particle destroyed");
        w.SetPinned(s.All.Where(p => s.View.ParticleLive[p.Lane] != 0).ToArray(), false);
        AssertLists(s, "everything woken");
    }

    // ---- (e) ------------------------------------------------------------------------------------------------

    private static ulong[] PinHashesRun(bool nudge = false)
    {
        using var s = Chained();
        var w = s.World;
        var hashes = new ulong[6];
        w.SetPinned(s.All, true);
        for (var c = 0; c < 6; c++)
        {
            for (var t = 0; t < 40; t++)
            {
                var tick = c * 40 + t;
                if (tick == 20) w.SetPinned(s.P[1], false);
                if (tick == 80) w.SetPinned(s.P[1], true);
                if (tick == 100) w.SetPinned([.. s.P[0], .. s.P[3]], false);
                if (tick == 160) w.SetPinned(s.P[2], false);
                if (tick == 200) w.SetPinned(s.P[0], true);
                if (nudge && tick == 21) w.SetPinned([s.P[2][5]], false);
                w.Step(Box3DSoaParticleTests.Dt, Box3DSoaParticleTests.Sub);
            }
            hashes[c] = HashAll(s.View);
        }
        return hashes;
    }

    // Recorded on this commit's build (MSVC /fp:precise, Release and Debug agree): pin and wake commands at fixed ticks on the chain scene, one hash per 40 steps (the hash covers the pinned lane).
    private static readonly ulong[] PinHashes = [1479169756272998701ul, 11282791371326620524ul, 7232528760214525229ul, 13136815359770158322ul, 17491214340810821373ul, 13380942627298567997ul];

    [Fact]
    public void Pin_scene_is_byte_deterministic()
    {
        var a = PinHashesRun();
        var b = PinHashesRun();
        Assert.True(a.AsSpan().SequenceEqual(b), "two runs diverged: " + string.Join(",", a) + " vs " + string.Join(",", b));
        Assert.True(a.Distinct().Count() == 6, "the scene moves");
        Assert.False(a.AsSpan().SequenceEqual(PinHashesRun(nudge: true)), "one more wake must diverge");
        Assert.True(PinHashes.AsSpan().SequenceEqual(a), string.Join(",", a.Select(g => g + "ul")));
    }

    // ---- (g) ------------------------------------------------------------------------------------------------

    private static (Box3DSoaWorld World, Box3DSoaParticleHandle[][] Sheets) Fellas(int capsules)
    {
        var w = Box3DSoaParticleTests.World(bodies: capsules + 1, particles: capsules * PerSheet + 8, distances: capsules * 60 + 8, bends: capsules * 48 + 8,
            attachments: capsules * PerSheet + 8, tethers: 8, joints: 0, gravityY: 0f);
        var sheets = new Box3DSoaParticleHandle[capsules][];
        var all = new List<Box3DSoaAttachmentDesc>();
        for (var c = 0; c < capsules; c++)
        {
            var origin = (0.5f * (c % 10), 0f, 0.5f * (c / 10));
            var bone = w.AddCapsule((ulong)(c + 1), origin, 0.05f, 0.1f, 500f);
            var sheet = Graph.Sheet(w, (origin.Item1 - 0.125f, -0.125f, origin.Item3 + Offset), (Step, 0f, 0f), (0f, Step, 0f), Side, Side, 0.02f, 0.0125f,
                Box3DSoaPhase.Of((uint)(c + 1), noParticleContact: true));
            sheets[c] = sheet.Particles;
            for (var q = 0; q < PerSheet; q++)
                all.Add(new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, sheet.Particles[q], bone, (-0.125f + Step * (q % Side), -0.125f + Step * (q / Side), Offset), compliance: 1e-4f, limb: true, oneWay: true));
        }
        w.Attach(all.ToArray());
        return (w, sheets);
    }

    [Fact]
    public void Dormant_flesh_cost_line()
    {
        // 76 capsules each with a 6 x 6 attached sheet (2,736 particles, about four Fellas' 2,644): live, all pinned, one sheet woken, and the bodies with no
        // particles, alternately 5 times, milliseconds per step over 10 steps after 3 warm-up steps (the Debug suite pays for every step of the live world). Printed, no bound.
        static double Median(double[] x) { Array.Sort(x); return x[x.Length / 2]; }
        const int n = 76;
        var names = new[] { "live", "pinned", "one woken", "bodies only" };
        var samples = names.Select(_ => new double[5]).ToArray();
        for (var r = 0; r < 5; r++)
        {
            for (var m = 0; m < 4; m++)
            {
                var mode = (m + r) % 4;
                var (w, sheets) = Fellas(n);
                using (w)
                {
                    var all = sheets.SelectMany(x => x).ToArray();
                    if (mode == 3) w.DestroyParticles(all);
                    if (mode == 1 || mode == 2) w.SetPinned(all, true);
                    if (mode == 2) w.SetPinned(sheets[0], false);
                    Box3DSoaParticleTests.Run(w, 3);
                    var watch = System.Diagnostics.Stopwatch.StartNew();
                    Box3DSoaParticleTests.Run(w, 10);
                    samples[mode][r] = watch.Elapsed.TotalMilliseconds / 10.0;
                }
            }
        }
        var line = "PIN_COST 76 capsules, 2736 particles, ms per step, medians of 5 interleaved: " + string.Join(", ", names.Select((nm, i) => $"{nm} {F(Median(samples[i]))}"));
        Reading(line);
        Assert.True(Median(samples[1]) > 0);
    }
}
