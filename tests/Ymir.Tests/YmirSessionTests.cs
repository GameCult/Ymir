using Ymir.Core;

namespace Ymir.Tests;

public sealed class YmirSessionTests
{
    [Fact]
    public void RetainedBodyAdvancesWithoutSnapshotResubmission()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "retained-motion",
            [Body("ship", positionX: 0.0f, velocityX: 2.0f)]));

        var first = session.Step(Step("step-1", 0, 0.25f));
        var second = session.Step(Step("step-2", 1, 0.25f));

        Assert.Equal(YmirCommandOutcome.Accepted, first.Receipt.Outcome);
        Assert.Equal(1, first.Revision);
        Assert.Equal(2, second.Revision);
        Assert.Equal(2, second.StepIndex);
        Assert.True(second.World.Bodies.Single().Position.X > first.World.Bodies.Single().Position.X);
        Assert.Equal(2.0f, second.World.Bodies.Single().Velocity.X, 4);
    }

    [Fact]
    public void ForceReceiptIsIdempotentAndForceIsConsumedByOneStep()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest("force-once", [Body("ship")]));
        var command = new YmirApplyForceCommand(Header("force", 0), "ship", new Vec2(10.0f, 0.0f));

        var accepted = session.ApplyForce(command);
        var replay = session.ApplyForce(command);
        var conflict = session.ApplyForce(command with { Force = new Vec2(20.0f, 0.0f) });
        var revisionConflict = session.ApplyForce(command with { Header = Header("force", 1) });
        var forced = session.Step(Step("step-forced", 1, 0.1f));
        var unforced = session.Step(Step("step-unforced", 2, 0.1f));

        Assert.Same(accepted, replay);
        Assert.Equal(1, accepted.AfterRevision);
        Assert.Equal(2, session.Info.StepIndex);
        Assert.Equal(YmirCommandError.CommandIdConflict, conflict.Error);
        Assert.Equal(YmirCommandOutcome.Rejected, conflict.Outcome);
        Assert.NotEqual(accepted.ReceiptId, conflict.ReceiptId);
        Assert.Equal(YmirCommandError.CommandIdConflict, revisionConflict.Error);
        Assert.Equal(forced.World.Bodies.Single().Velocity.X, unforced.World.Bodies.Single().Velocity.X, 4);
    }

    [Fact]
    public void OmissionCannotRemoveBodyAndRemoveIsExplicit()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "explicit-remove",
            [Body("a"), Body("b", positionX: 10.0f)]));

        var velocity = session.SetVelocity(new YmirSetBodyVelocityCommand(
            Header("set-a", 0), "a", new Vec2(1.0f, 0.0f), 0.0f));
        var stepped = session.Step(Step("step", 1, 0.1f));
        var removed = session.Remove(new YmirRemoveBodyCommand(Header("remove-b", 2), "b"));

        Assert.Equal(YmirCommandOutcome.Accepted, velocity.Outcome);
        Assert.Equal(2, stepped.World.Bodies.Count);
        Assert.Equal(YmirCommandOutcome.Accepted, removed.Outcome);
        Assert.Equal(["a"], session.Snapshot().Bodies.Select(body => body.Id).ToArray());
    }

    [Fact]
    public void ContactEpisodeEmitsBeginThenEndThenNewBegin()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "contact-lifecycle",
            [
                Body("a", positionX: 0.0f, isStatic: true),
                Body("b", positionX: 1.5f)
            ]));

        var began = session.Step(Step("begin", 0, 0.01f));
        var persistent = session.Step(Step("persistent", 1, 0.01f));
        session.Teleport(new YmirTeleportBodyCommand(
            Header("separate", 2), "b", new Vec2(10.0f, 0.0f), new Vec2(0.0f, 1.0f)));
        var ended = session.Step(Step("end", 3, 0.01f));
        session.Teleport(new YmirTeleportBodyCommand(
            Header("return", 4), "b", new Vec2(1.5f, 0.0f), new Vec2(0.0f, 1.0f)));
        var beganAgain = session.Step(Step("begin-again", 5, 0.01f));

        var firstBegin = Assert.Single(began.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);
        Assert.DoesNotContain(persistent.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);
        var end = Assert.Single(ended.ContactFacts, fact => fact.Kind == YmirContactFactKind.End);
        var secondBegin = Assert.Single(beganAgain.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);
        Assert.Equal(firstBegin.ContactId, end.ContactId);
        Assert.NotEqual(firstBegin.ContactId, secondBegin.ContactId);
        Assert.Equal("a", firstBegin.BodyA);
        Assert.Equal("b", firstBegin.BodyB);
    }

    [Fact]
    public void ReplayCheckpointPreservesContactLineageAndCommandLedger()
    {
        using var original = YmirSession.Create(new YmirSessionCreateRequest(
            "checkpoint-contact",
            [
                Body("ship", positionX: 0.0f, isStatic: true),
                Body("pickup", positionX: 1.5f)
            ]));
        var began = original.Step(Step("begin", 0, 0.01f));
        var firstBegin = Assert.Single(began.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);
        var checkpoint = original.Checkpoint();
        var payload = YmirSessionCheckpointCodec.Encode(checkpoint);
        checkpoint = YmirSessionCheckpointCodec.Decode(payload);

        using var restored = YmirSession.Restore(checkpoint);
        var duplicate = restored.Step(Step("begin", 0, 0.01f));
        Assert.Equal(began.Receipt.ReceiptId, duplicate.Receipt.ReceiptId);
        Assert.Equal(began.Receipt.Outcome, duplicate.Receipt.Outcome);
        Assert.Equal(began.Receipt.BeforeRevision, duplicate.Receipt.BeforeRevision);
        Assert.Equal(began.Receipt.AfterRevision, duplicate.Receipt.AfterRevision);
        Assert.Equal(began.Receipt.ProducedFactIds, duplicate.Receipt.ProducedFactIds);
        Assert.Equal(began.ContactFacts.Select(value => value.FactId), duplicate.ContactFacts.Select(value => value.FactId));
        Assert.Equal(checkpoint.SessionGeneration, restored.Info.SessionGeneration);

        var conflict = restored.Step(Step("begin", 0, 0.02f));
        Assert.Equal(YmirCommandError.CommandIdConflict, conflict.Receipt.Error);
        var persistent = restored.Step(Step("persistent", 1, 0.01f));
        Assert.DoesNotContain(persistent.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);
        restored.Teleport(new YmirTeleportBodyCommand(
            Header("separate", 2), "pickup", new Vec2(10.0f, 0.0f), new Vec2(0.0f, 1.0f)));
        var ended = restored.Step(Step("end", 3, 0.01f));
        var end = Assert.Single(ended.ContactFacts, fact => fact.Kind == YmirContactFactKind.End);
        Assert.Equal(firstBegin.ContactId, end.ContactId);
        restored.Teleport(new YmirTeleportBodyCommand(
            Header("return", 4), "pickup", new Vec2(1.5f, 0.0f), new Vec2(0.0f, 1.0f)));
        var beganAgain = restored.Step(Step("begin-again", 5, 0.01f));
        var secondBegin = Assert.Single(beganAgain.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);
        Assert.NotEqual(firstBegin.ContactId, secondBegin.ContactId);
    }

    [Fact]
    public void ReplayCheckpointRestoresPendingForceAndRejectsCorruptJournal()
    {
        using var original = YmirSession.Create(new YmirSessionCreateRequest(
            "checkpoint-force", [Body("ship")]));
        original.ApplyForce(new YmirApplyForceCommand(Header("force", 0), "ship", new Vec2(10.0f, 0.0f)));
        var checkpoint = original.Checkpoint();

        using var restored = YmirSession.Restore(checkpoint);
        var stepped = restored.Step(Step("step", 1, 0.1f));
        Assert.True(stepped.World.Bodies.Single().Velocity.X > 0.0f);

        var corrupt = checkpoint with { Journal = [] };
        Assert.Throws<InvalidOperationException>(() => YmirSession.Restore(corrupt));
        var wrongRuntime = checkpoint with { RuntimeFingerprint = "some-other-solver" };
        Assert.Throws<InvalidOperationException>(() => YmirSession.Restore(wrongRuntime));
        var corruptIdentity = YmirSessionCheckpointCodec.Encode(checkpoint);
        corruptIdentity[32] ^= 0xff;
        Assert.Throws<InvalidDataException>(() => YmirSessionCheckpointCodec.Decode(corruptIdentity));
        Assert.Throws<InvalidDataException>(() =>
            YmirSessionCheckpointCodec.Decode(YmirSessionCheckpointCodec.Encode(checkpoint).Concat(new byte[] { 1 }).ToArray()));
    }

    [Fact]
    public void RemoveAndRecreateChangesBodyGeneration()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "body-generation",
            [Body("wall", isStatic: true), Body("pickup", positionX: 1.5f)]));
        var first = session.Step(Step("begin-1", 0, 0.01f));
        var firstFact = Assert.Single(first.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);
        session.Remove(new YmirRemoveBodyCommand(Header("remove", 1), "pickup"));
        var removed = session.Step(Step("removed", 2, 0.01f));
        session.Spawn(new YmirSpawnBodyCommand(Header("spawn", 3), Body("pickup", positionX: 1.5f)));
        var second = session.Step(Step("begin-2", 4, 0.01f));
        var secondFact = Assert.Single(second.ContactFacts, fact =>
            fact.Kind == YmirContactFactKind.Begin &&
            (fact.BodyA == "pickup" || fact.BodyB == "pickup"));

        var firstGeneration = BodyGeneration(firstFact, "pickup");
        Assert.Contains(removed.ContactFacts, fact =>
            fact.Kind == YmirContactFactKind.End && fact.ContactId == firstFact.ContactId);
        Assert.True(BodyGeneration(secondFact, "pickup") > firstGeneration);
        Assert.NotEqual(firstFact.ContactId, secondFact.ContactId);
    }

    [Fact]
    public void SnapshotImmediatelyReflectsCanonicalNativeMutationState()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "canonical-snapshot",
            [Body("ship") with { Direction = new Vec2(0.0f, 2.0f) }]));

        Assert.Equal(new Vec2(0.0f, 1.0f), session.Snapshot().Bodies.Single().Direction);
        session.Teleport(new YmirTeleportBodyCommand(
            Header("teleport", 0), "ship", new Vec2(3.0f, 4.0f), new Vec2(2.0f, 0.0f)));
        var teleported = session.Snapshot().Bodies.Single();
        Assert.Equal(1.0f, teleported.Direction!.Value.X, 5);
        Assert.Equal(0.0f, teleported.Direction.Value.Y, 5);
        Assert.Equal(new Vec2(3.0f, 4.0f), teleported.Position);

        session.Configure(new YmirConfigureBodyCommand(
            Header("static", 1), "ship", 2.0f, 1.0f, true, 0.5f));
        var configured = session.Snapshot().Bodies.Single();
        Assert.True(configured.IsStatic);
        Assert.Equal(2.0f, configured.Radius);
        Assert.Equal(0.5f, configured.Restitution);
    }

    [Fact]
    public void StaleAndDisposedCommandsFailClosedWithoutMutation()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest("stale", [Body("ship")]));
        var accepted = session.SetVelocity(new YmirSetBodyVelocityCommand(
            Header("accepted", 0), "ship", new Vec2(1.0f, 0.0f), 0.0f));
        var stale = session.SetVelocity(new YmirSetBodyVelocityCommand(
            Header("stale", 0), "ship", new Vec2(100.0f, 0.0f), 0.0f));
        var disposed = session.Dispose(new YmirDisposeSessionCommand(Header("dispose", 1)));
        var afterDispose = session.Remove(new YmirRemoveBodyCommand(Header("late", 2), "ship"));

        Assert.Equal(YmirCommandOutcome.Accepted, accepted.Outcome);
        Assert.Equal(YmirCommandError.StaleRevision, stale.Error);
        Assert.Equal(YmirCommandOutcome.Accepted, disposed.Outcome);
        Assert.Equal(YmirCommandError.SessionDisposed, afterDispose.Error);
        Assert.Equal(2, afterDispose.BeforeRevision);
        Assert.Equal(afterDispose.BeforeRevision, afterDispose.AfterRevision);
    }

    [Fact]
    public void TypedFingerprintRejectsDelimiterCollisionsAndNullPayloads()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest("fingerprints", []));
        var accepted = session.Spawn(new YmirSpawnBodyCommand(
            Header("spawn", 0), Body("body|with|delimiters")));
        var conflict = session.Spawn(new YmirSpawnBodyCommand(
            Header("spawn", 0), Body("body") with { Position = new Vec2(1.0f, 2.0f) }));
        var nullBody = session.Spawn(new YmirSpawnBodyCommand(Header("null-body", 1), null!));
        var nullFields = session.Step(new YmirStepSessionCommand(Header("null-fields", 1), 0.1f, null!));

        Assert.Equal(YmirCommandOutcome.Accepted, accepted.Outcome);
        Assert.Equal(YmirCommandError.CommandIdConflict, conflict.Error);
        Assert.Equal(YmirCommandError.InvalidCommand, nullBody.Error);
        Assert.Equal(YmirCommandError.InvalidCommand, nullFields.Receipt.Error);
        Assert.Equal(1, session.Info.Revision);
    }

    [Fact]
    public void StaticBodyVelocityCommandIsRejectedWithoutARevisionLie()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "static-velocity", [Body("wall", isStatic: true)]));

        var receipt = session.SetVelocity(new YmirSetBodyVelocityCommand(
            Header("move-wall", 0), "wall", new Vec2(10.0f, 0.0f), 2.0f));

        Assert.Equal(YmirCommandOutcome.Rejected, receipt.Outcome);
        Assert.Equal(YmirCommandError.StaticBody, receipt.Error);
        Assert.Equal(0, session.Info.Revision);
        Assert.Equal(Vec2.Zero, session.Snapshot().Bodies.Single().Velocity);
    }

    [Fact]
    public void CollisionProfilesExcludeOwnGroupAndPayloadPeers()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "collision-profile",
            [
                Body("source") with
                {
                    CollisionCategoryBits = 1,
                    CollisionMaskBits = 2,
                    CollisionGroupIndex = -7
                },
                Body("own-payload") with
                {
                    CollisionCategoryBits = 2,
                    CollisionMaskBits = 1,
                    CollisionGroupIndex = -7
                },
                Body("other-payload") with
                {
                    CollisionCategoryBits = 2,
                    CollisionMaskBits = 1,
                    CollisionGroupIndex = -8
                }
            ]));

        var result = session.Step(Step("profile-step", 0, 1.0f / 60.0f));
        var begin = Assert.Single(result.ContactFacts, fact => fact.Kind == YmirContactFactKind.Begin);

        Assert.Equal(new HashSet<string> { "source", "other-payload" }, new HashSet<string> { begin.BodyA, begin.BodyB });
    }

    [Fact]
    public void BodyCanOptOutOfRadialFields()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "field-profile",
            [Body("affected", positionX: 1.0f), Body("ignored", positionX: -5.0f) with { ParticipatesInFields = false }]));

        var result = session.Step(new YmirStepSessionCommand(
            Header("field-step", 0),
            0.1f,
            [new RadialField("gravity", Vec2.Zero, 10.0f, 10.0f)]));

        Assert.NotEqual(0.0f, result.World.Bodies.Single(body => body.Id == "affected").Velocity.X);
        Assert.Equal(0.0f, result.World.Bodies.Single(body => body.Id == "ignored").Velocity.X, 5);
    }

    [Fact]
    public void KinematicTargetDoesNotAcceptBulletImpulse()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "bullet-kinematic",
            [
                Body("bullet", positionX: -10.0f, velocityX: 100.0f) with { IsBullet = true },
                Body("target") with { IsKinematic = true }
            ]));

        var ccdStep = session.Step(Step("ccd", 0, 0.2f));
        var eventStep = session.Step(Step("event", 1, 1.0f / 60.0f));

        Assert.Empty(ccdStep.ContactFacts);
        Assert.Contains(eventStep.ContactFacts, fact => fact.Kind is YmirContactFactKind.Begin or YmirContactFactKind.Hit);
        var target = eventStep.World.Bodies.Single(body => body.Id == "target");
        Assert.Equal(Vec2.Zero, target.Velocity);
        Assert.Equal(Vec2.Zero, target.Position);
    }

    [Fact]
    public void RetainedCastObservesNamedRevisionAndExplicitCandidates()
    {
        using var session = YmirSession.Create(new YmirSessionCreateRequest(
            "retained-cast",
            [Body("source", positionX: -2.0f), Body("target", positionX: 2.0f), Body("ignored", positionX: 0.0f)]));

        var result = session.CastCircle(new YmirSessionCircleCastQuery(
            "payload-sweep",
            0,
            new Vec2(-5.0f, 0.0f),
            new Vec2(1.0f, 0.0f),
            10.0f,
            0.25f,
            ["target"]));

        Assert.Equal(YmirQueryError.None, result.Error);
        Assert.Equal(0, result.ObservedRevision);
        Assert.Equal(0, result.ObservedStepIndex);
        Assert.Equal("target", Assert.Single(result.Hits).BodyId);
        Assert.Equal(0, session.Info.Revision);
    }

    [Fact]
    public void RetainedQueriesFailClosedForStaleUnknownAndDisposedSessions()
    {
        var session = YmirSession.Create(new YmirSessionCreateRequest("query-failures", [Body("target")]));

        var stale = session.OverlapCircle(new YmirSessionCircleOverlapQuery(
            "stale", 1, Vec2.Zero, 1.0f, ["target"]));
        var unknown = session.OverlapCircle(new YmirSessionCircleOverlapQuery(
            "unknown", 0, Vec2.Zero, 1.0f, ["missing"]));
        session.Dispose();
        var disposed = session.OverlapCircle(new YmirSessionCircleOverlapQuery(
            "disposed", 0, Vec2.Zero, 1.0f, ["target"]));

        Assert.Equal(YmirQueryError.StaleRevision, stale.Error);
        Assert.Equal(YmirQueryError.UnknownBody, unknown.Error);
        Assert.Equal(YmirQueryError.SessionDisposed, disposed.Error);
        Assert.Empty(stale.Hits);
        Assert.Empty(unknown.Hits);
        Assert.Empty(disposed.Hits);
    }

    private static YmirCommandHeader Header(string id, long revision) => new(id, revision);

    private static long BodyGeneration(YmirContactFact fact, string bodyId) =>
        fact.BodyA == bodyId ? fact.BodyAGeneration : fact.BodyBGeneration;

    private static YmirStepSessionCommand Step(string id, long revision, float deltaTime) =>
        new(Header(id, revision), deltaTime, []);

    private static PhysicsBody Body(
        string id,
        float positionX = 0.0f,
        float velocityX = 0.0f,
        bool isStatic = false) =>
        new(
            id,
            new Vec2(positionX, 0.0f),
            new Vec2(velocityX, 0.0f),
            1.0f,
            1.0f,
            isStatic,
            0.0f,
            new Vec2(0.0f, 1.0f));
}
