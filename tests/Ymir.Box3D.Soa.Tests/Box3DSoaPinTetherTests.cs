using System.Runtime.InteropServices;
using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

/// <summary>
/// No distance component holds both a pinned particle and a live TETHER attachment (docs/soa-world.md, pinned): a tether loads every lane of its
/// component and no walk moves a pinned one. rederive_topology checks it once for every command; these read it through each door a
/// command opens, and through random command sequences.
/// </summary>
public sealed class Box3DSoaPinTetherTests
{
    private const float R = 0.0125f;

    private static ulong Fnv(ulong h, ReadOnlySpan<byte> bytes)
    {
        foreach (var b in bytes) { h ^= b; h *= 1099511628211ul; }
        return h;
    }

    /// <summary>Every lane the view exposes for particles, constraints, tethers and bodies, with the counts and free slots.</summary>
    private static ulong State(Box3DSoaWorld w)
    {
        var v = w.View;
        var h = 1469598103934665603ul;
        void L<T>(ReadOnlySpan<T> lane) where T : unmanaged => h = Fnv(h, MemoryMarshal.AsBytes(lane));
        void I(long x) => h = Fnv(h, BitConverter.GetBytes(x));
        I(v.ParticleCount); I(v.DistanceCount); I(v.BendCount); I(v.AttachmentCount); I(v.TetherCount);
        I(w.FreeDistanceSlots); I(w.FreeBendSlots); I(w.FreeAttachmentSlots);
        L(v.ParticleX); L(v.ParticleY); L(v.ParticleZ); L(v.ParticleVx); L(v.ParticleVy); L(v.ParticleVz); L(v.ParticleInvMass);
        L(v.ParticleLive); L(v.ParticleGeneration); L(v.ParticleComponent); L(v.ParticlePinned); L(v.ParticleAnchor);
        L(v.ParticleImpulseX); L(v.ParticleImpulseY); L(v.ParticleImpulseZ);
        L(v.DistanceA); L(v.DistanceB); L(v.DistanceRest); L(v.DistanceCompliance); L(v.DistanceTension); L(v.DistanceLive); L(v.DistanceGeneration);
        L(v.BendA); L(v.BendB); L(v.BendC); L(v.BendE1); L(v.BendE2); L(v.BendLive); L(v.BendGeneration);
        L(v.AttachmentKind); L(v.AttachmentFlags); L(v.AttachmentGeneration); L(v.AttachmentParticle); L(v.AttachmentBody);
        L(v.AttachmentPointX); L(v.AttachmentPointY); L(v.AttachmentPointZ); L(v.AttachmentForceX); L(v.AttachmentForceY); L(v.AttachmentForceZ);
        L(v.TetherAttachment); L(v.TetherParticle); L(v.TetherParent); L(v.TetherEdge); L(v.TetherOther); L(v.TetherBound);
        L(v.PositionX); L(v.PositionY); L(v.PositionZ); L(v.RotationX); L(v.RotationY); L(v.RotationZ); L(v.RotationW);
        L(v.VelocityX); L(v.VelocityY); L(v.VelocityZ); L(v.AngularVelocityX); L(v.AngularVelocityY); L(v.AngularVelocityZ);
        return h;
    }

    private static void Step(Box3DSoaWorld w, int n) => Box3DSoaParticleTests.Run(w, n);

    /// <summary>Soul's scenes: rope A (4 particles, R0..R2 on a bone, `pinA` pins them) and rope B (3 particles, B0 tethered to a ball), apart.</summary>
    private sealed class Scene : IDisposable
    {
        public Box3DSoaWorld W = null!;
        public int Bone, Ball;
        public Box3DSoaParticleHandle[] A = null!, B = null!;
        public void Dispose() => W.Dispose();
    }

    private static Scene Make(bool pinA = true, bool tetherB = true)
    {
        var s = new Scene { W = new Box3DSoaWorld(8, gravityY: 0f, particles: new Box3DSoaParticleCapacities(64, 32, 32, 32, 64, 2, 0, 0, 0)) };
        var w = s.W;
        s.Bone = w.AddAnchor(1, (0f, 1f, 0f));
        s.Ball = w.AddSphere(5, (0.45f, 1.0f, 0f), 0.05f, 1000f);
        s.A = Graph.Rope(w, Graph.Line((0f, 1f, 0f), (0.05f, 0f, 0f), 4), 0.02f, R, Box3DSoaPhase.Of(4, noParticleContact: true)).Particles;
        for (var k = 0; k < 3; k++) w.Attach(Box3DSoaAttachKind.Body, s.A[k], s.Bone, (0.05f * k, 0f, 0f), limb: true, oneWay: true);
        s.B = Graph.Rope(w, Graph.Line((0.2f, 1f, 0f), (0.05f, 0f, 0f), 3), 0.02f, R, Box3DSoaPhase.Of(5, noParticleContact: true)).Particles;
        if (tetherB) w.Attach(Box3DSoaAttachKind.Body, s.B[2], s.Ball, (-0.15f, 0f, 0f), tether: true);
        if (pinA) w.SetPinned(s.A[..3], true);
        Step(w, 3);
        return s;
    }

    /// <summary>The command is refused (status 1) with every lane as in a twin that never saw it, and stays so over ten steps (the active lists too).</summary>
    private static void RefusedLikeTwin(string what, Action<Scene> command, Func<Scene>? make = null)
    {
        make ??= () => Make();
        using var s = make();
        using var twin = make();
        Assert.Equal(State(twin.W), State(s.W));
        var e = Assert.Throws<InvalidOperationException>(() => command(s));
        Assert.Contains("status 1", e.Message);
        Assert.True(State(twin.W) == State(s.W), $"{what}: the refusal changed a lane");
        Step(s.W, 10);
        Step(twin.W, 10);
        Assert.True(State(twin.W) == State(s.W), $"{what}: the refused world steps differently");
    }

    [Fact]
    public void A_tether_on_pinned_flesh_is_refused_whole()
    {
        RefusedLikeTwin("a tether on a pinned particle", s => s.W.Attach(Box3DSoaAttachKind.Body, s.A[1], s.Ball, (-0.4f, 0f, 0f), tether: true));
        RefusedLikeTwin("a batch: a plain attach, then a tether on a pinned particle", s => s.W.Attach([
            new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, s.A[3], s.Bone, (0.15f, 0f, 0f)),
            new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, s.A[0], s.Ball, (-0.45f, 0f, 0f), tether: true)]));
    }

    [Fact]
    public void A_tether_on_a_live_neighbour_of_pinned_flesh_is_refused_whole()
    {
        RefusedLikeTwin("a tether on live R3 of a component with pinned R0..R2", s => s.W.Attach(Box3DSoaAttachKind.Body, s.A[3], s.Ball, (-0.3f, 0f, 0f), tether: true));
    }

    [Fact]
    public void A_distance_joining_pinned_flesh_to_a_tethered_component_is_refused_whole()
    {
        RefusedLikeTwin("one distance A3-B0", s => s.W.CreateDistances([new Box3DSoaDistanceDesc(s.A[3], s.B[0])]));
        RefusedLikeTwin("a batch: one harmless distance, then the join", s => s.W.CreateDistances([
            new Box3DSoaDistanceDesc(s.B[0], s.B[2]), new Box3DSoaDistanceDesc(s.A[3], s.B[0])]));
        // the controls: woken, or with nothing tethered, the same join is accepted
        using (var woke = Make())
        {
            woke.W.SetPinned(woke.A[..3], false);
            woke.W.CreateDistances([new Box3DSoaDistanceDesc(woke.A[3], woke.B[0])]);
            Assert.Equal(woke.W.View.ParticleComponent[woke.A[0]], woke.W.View.ParticleComponent[woke.B[2]]);
        }
        using (var untethered = Make(tetherB: false))
            untethered.W.CreateDistances([new Box3DSoaDistanceDesc(untethered.A[3], untethered.B[0])]);
    }

    [Fact]
    public void A_pin_in_a_tethered_component_is_refused_whole()
    {
        static Scene Joined()
        {
            var s = Make(pinA: false);
            s.W.CreateDistances([new Box3DSoaDistanceDesc(s.A[3], s.B[0])]);
            return s;
        }
        RefusedLikeTwin("a pin of R0..R2 joined to tethered B", s => s.W.SetPinned(s.A[..3], true), Joined);
        RefusedLikeTwin("a pin of R1 alone", s => s.W.SetPinned([s.A[1]], true), Joined);
    }

    // ---- random command sequences ---------------------------------------------------------------------------

    private static bool Holds(Box3DSoaView v, out string why)
    {
        var n = v.ParticleCount;
        var root = new int[n];
        for (var i = 0; i < n; i++) root[i] = i;
        int Find(int i) { while (root[i] != i) i = root[i] = root[root[i]]; return i; }
        for (var e = 0; e < v.DistanceCount; e++)
            if (v.DistanceLive[e] != 0) root[Find(v.DistanceA[e])] = Find(v.DistanceB[e]);
        var pinned = new HashSet<int>();
        var tethered = new HashSet<int>();
        for (var i = 0; i < n; i++) if (v.ParticleLive[i] != 0 && v.ParticlePinned[i] != 0) pinned.Add(Find(i));
        for (var a = 0; a < v.AttachmentCount; a++)
            if (v.AttachmentKind[a] != (uint)Box3DSoaAttachKind.Removed && (v.AttachmentFlags[a] & 1u) != 0) tethered.Add(Find(v.AttachmentParticle[a]));
        // the view's component lane names the same partition
        for (var i = 0; i < n; i++)
            for (var j = i + 1; j < n; j++)
                if (v.ParticleLive[i] != 0 && v.ParticleLive[j] != 0 && (Find(i) == Find(j)) != (v.ParticleComponent[i] == v.ParticleComponent[j]))
                {
                    why = $"particles {i} and {j}: component lane disagrees with the distances";
                    return false;
                }
        pinned.IntersectWith(tethered);
        why = pinned.Count == 0 ? "" : $"component of {pinned.First()} holds pinned flesh and a tether";
        return pinned.Count == 0;
    }

    [Theory]
    [InlineData(1)] [InlineData(2)] [InlineData(3)] [InlineData(4)] [InlineData(5)] [InlineData(6)] [InlineData(7)] [InlineData(8)]
    public void No_command_sequence_puts_pinned_flesh_and_a_tether_in_one_component(int seed)
    {
        const int N = 12;
        var rng = new Random(seed);
        using var w = new Box3DSoaWorld(8, gravityY: -10f, particles: new Box3DSoaParticleCapacities(N, 48, 32, 64, 8192, 2, 0, 0, 0));
        var bone = w.AddAnchor(1, (0f, 1f, 0f));
        var ball = w.AddSphere(5, (0f, 0.5f, 0.5f), 0.05f, 100f);
        var p = new Box3DSoaParticleHandle[N];
        var desc = new Box3DSoaParticleDesc[N];
        for (var i = 0; i < N; i++) desc[i] = new Box3DSoaParticleDesc((0.06f * i, 1f, 0f), 0.02f, R, Box3DSoaPhase.Of((uint)(1 + i % 3), noParticleContact: true));
        w.CreateParticles(desc, p);
        for (var i = 0; i < N; i++) w.Attach(Box3DSoaAttachKind.Body, p[i], bone, (0.06f * i, 0f, 0f), compliance: 1e-4f, limb: true, oneWay: true);
        Box3DSoaParticleHandle Any() => p[rng.Next(N)];
        Box3DSoaAttachmentDesc Tether(Box3DSoaParticleHandle q) => rng.Next(2) == 0
            ? new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Static, q, -1, (w.View.ParticleX[q], w.View.ParticleY[q], 0f), tether: true)
            : new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, q, ball, (0f, 0.5f, -0.5f), tether: true);
        int accepted = 0, refused = 0, pins = 0, joins = 0, tethers = 0;
        var refusedBy = new int[12];
        for (var c = 0; c < 400; c++)
        {
            var v = w.View;
            var kind = rng.Next(12);
            Action command;
            switch (kind)
            {
                case 0:
                {
                    var d = new Box3DSoaDistanceDesc[1 + rng.Next(2)];
                    for (var k = 0; k < d.Length; k++)
                    {
                        var a = rng.Next(N);
                        d[k] = new Box3DSoaDistanceDesc(p[a], p[(a + 1 + rng.Next(N - 1)) % N], 1e-4f);
                    }
                    command = () => { w.CreateDistances(d); joins++; };
                    break;
                }
                case 1: case 2:
                {
                    var d = rng.Next(2) == 0 ? new[] { Tether(Any()) } : new[] { new Box3DSoaAttachmentDesc(Box3DSoaAttachKind.Body, Any(), bone, (0f, 0f, 0f)), Tether(Any()) };
                    command = () => { w.Attach(d); tethers++; };
                    break;
                }
                case 3: case 4:
                {
                    var set = Enumerable.Range(0, 1 + rng.Next(4)).Select(_ => Any()).ToArray();
                    command = () => { w.SetPinned(set, true); pins++; };
                    break;
                }
                case 5:
                {
                    var set = Enumerable.Range(0, 1 + rng.Next(4)).Select(_ => Any()).ToArray();
                    command = () => w.SetPinned(set, false);
                    break;
                }
                case 6: case 7: case 8:
                {
                    var live = Enumerable.Range(0, v.DistanceCount).Where(e => v.DistanceLive[e] != 0).ToArray();
                    if (live.Length == 0) continue;
                    var e = live[rng.Next(live.Length)];
                    var h = new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Distance, e, v.DistanceGeneration[e]);
                    command = () => w.Remove(h);
                    break;
                }
                case 9: case 10:
                {
                    // mostly a tether, so components free up for pins again
                    var tethered = rng.Next(3) != 0;
                    var live = Enumerable.Range(0, v.AttachmentCount).Where(a => v.AttachmentKind[a] != (uint)Box3DSoaAttachKind.Removed && (!tethered || (v.AttachmentFlags[a] & 1u) != 0)).ToArray();
                    if (live.Length == 0) continue;
                    var a = live[rng.Next(live.Length)];
                    var h = new Box3DSoaConstraintHandle(Box3DSoaConstraintType.Attachment, a, v.AttachmentGeneration[a]);
                    command = () => w.Remove(h);
                    break;
                }
                default:
                {
                    // a bend over two live distances sharing a particle, else a step
                    var live = Enumerable.Range(0, v.DistanceCount).Where(e => v.DistanceLive[e] != 0 && v.DistanceRest[e] > 0f).ToArray();
                    var pair = live.SelectMany(e1 => live.Where(e2 => e2 > e1 &&
                        ((v.DistanceA[e1] == v.DistanceA[e2] ? 1 : 0) + (v.DistanceA[e1] == v.DistanceB[e2] ? 1 : 0) + (v.DistanceB[e1] == v.DistanceA[e2] ? 1 : 0) + (v.DistanceB[e1] == v.DistanceB[e2] ? 1 : 0)) == 1)
                        .Select(e2 => (e1, e2))).ToArray();
                    if (pair.Length == 0 || rng.Next(2) == 0) { command = () => Step(w, 1); break; }
                    var (x, y) = pair[rng.Next(pair.Length)];
                    var b = new Box3DSoaBendDesc(new(Box3DSoaConstraintType.Distance, x, v.DistanceGeneration[x]), new(Box3DSoaConstraintType.Distance, y, v.DistanceGeneration[y]));
                    command = () => w.CreateBends([b]);
                    break;
                }
            }
            var before = State(w);
            try
            {
                command();
                accepted++;
            }
            catch (InvalidOperationException)
            {
                refused++;
                refusedBy[kind]++;
                Assert.True(before == State(w), $"seed {seed} command {c} (kind {kind}): a refusal changed a lane");
            }
            Assert.True(Holds(w.View, out var why), $"seed {seed} after command {c} (kind {kind}): {why}");
        }
        Console.Error.WriteLine($"PIN_TETHER seed {seed}: accepted {accepted}, refused {refused}, pins {pins}, joins {joins}, tethers {tethers}; refused joins {refusedBy[0]}, tethers {refusedBy[1] + refusedBy[2]}, pins {refusedBy[3] + refusedBy[4]}");
        Assert.True(pins > 10 && joins > 10 && tethers > 10, $"seed {seed}: the sequence exercised too little (refused {refused}, pins {pins}, joins {joins}, tethers {tethers})");
    }
}
