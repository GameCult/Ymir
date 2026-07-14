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
