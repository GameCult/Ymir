using Ymir.Box3D;

namespace Ymir.Core;

public sealed record YmirSessionCreateRequest(
    string SessionId,
    IReadOnlyList<PhysicsBody> InitialBodies,
    float InitialTime = 0.0f);

public static class YmirSessionCheckpointContract
{
    public const string FormatId = "gamecult.ymir.session_checkpoint.replay.v1";
    public const string JournalChunkFormatId = "gamecult.ymir.session_journal_chunk.v1";
    public const string ResumeDescriptorFormatId = "gamecult.ymir.session_resume.v1";
    public static string RuntimeFingerprint =>
        $"gamecult.ymir.replay.v1|{Box3DRuntime.ValidatedBuildId}";
}

public sealed record YmirSessionJournalChunk(
    string FormatId,
    string RuntimeFingerprint,
    string SessionId,
    string SessionGeneration,
    long FirstEntryIndex,
    IReadOnlyList<YmirSessionJournalEntry> Entries);

public sealed record YmirSessionResumeDescriptor(
    string FormatId,
    string RuntimeFingerprint,
    string SessionId,
    string SessionGeneration,
    long Revision,
    long StepIndex,
    float Time,
    long NextBodyGeneration,
    IReadOnlyList<PhysicsBody> InitialBodies,
    float InitialTime,
    long JournalEntryCount,
    string JournalDigest,
    YmirWorld World,
    IReadOnlyList<YmirContactEpisodeCheckpoint> ActiveContactEpisodes);

public sealed record YmirSessionPersistenceCapture(
    YmirSessionJournalChunk? JournalChunk,
    YmirSessionResumeDescriptor ResumeDescriptor);

public sealed record YmirSessionCheckpoint(
    string FormatId,
    string RuntimeFingerprint,
    string SessionId,
    string SessionGeneration,
    long Revision,
    long StepIndex,
    float Time,
    long NextBodyGeneration,
    IReadOnlyList<PhysicsBody> InitialBodies,
    float InitialTime,
    IReadOnlyList<YmirSessionJournalEntry> Journal,
    string JournalDigest,
    YmirWorld World,
    IReadOnlyList<YmirContactEpisodeCheckpoint> ActiveContactEpisodes);

public sealed record YmirContactEpisodeCheckpoint(
    string BodyA,
    long BodyAGeneration,
    string BodyB,
    long BodyBGeneration,
    long StartStepIndex,
    long Ordinal);

public abstract record YmirSessionJournalEntry;

public sealed record YmirSpawnBodyJournalEntry(YmirSpawnBodyCommand Command) : YmirSessionJournalEntry;
public sealed record YmirRemoveBodyJournalEntry(YmirRemoveBodyCommand Command) : YmirSessionJournalEntry;
public sealed record YmirTeleportBodyJournalEntry(YmirTeleportBodyCommand Command) : YmirSessionJournalEntry;
public sealed record YmirSetBodyVelocityJournalEntry(YmirSetBodyVelocityCommand Command) : YmirSessionJournalEntry;
public sealed record YmirConfigureBodyJournalEntry(YmirConfigureBodyCommand Command) : YmirSessionJournalEntry;
public sealed record YmirApplyForceJournalEntry(YmirApplyForceCommand Command) : YmirSessionJournalEntry;
public sealed record YmirApplyTorqueJournalEntry(YmirApplyTorqueCommand Command) : YmirSessionJournalEntry;
public sealed record YmirStepSessionJournalEntry(YmirStepSessionCommand Command) : YmirSessionJournalEntry;

public sealed record YmirCommandHeader(string CommandId, long ExpectedRevision);

public sealed record YmirSpawnBodyCommand(YmirCommandHeader Header, PhysicsBody Body);

public sealed record YmirRemoveBodyCommand(YmirCommandHeader Header, string BodyId);

public sealed record YmirTeleportBodyCommand(
    YmirCommandHeader Header,
    string BodyId,
    Vec2 Position,
    Vec2 Direction);

public sealed record YmirSetBodyVelocityCommand(
    YmirCommandHeader Header,
    string BodyId,
    Vec2 LinearVelocity,
    float AngularVelocity);

public sealed record YmirConfigureBodyCommand(
    YmirCommandHeader Header,
    string BodyId,
    float Radius,
    float Mass,
    bool IsStatic,
    float Restitution,
    bool IsKinematic = false);

public sealed record YmirApplyForceCommand(
    YmirCommandHeader Header,
    string BodyId,
    Vec2 Force);

public sealed record YmirApplyTorqueCommand(
    YmirCommandHeader Header,
    string BodyId,
    float Torque);

public sealed record YmirStepSessionCommand(
    YmirCommandHeader Header,
    float DeltaTime,
    IReadOnlyList<RadialField> Fields,
    int SubstepCount = 4);

public sealed record YmirDisposeSessionCommand(YmirCommandHeader Header);

public enum YmirCommandOutcome
{
    Accepted,
    Rejected
}

public enum YmirCommandError
{
    None,
    StaleRevision,
    CommandIdConflict,
    DuplicateBody,
    UnknownBody,
    StaticBody,
    InvalidCommand,
    SessionDisposed,
    SessionFaulted
}

public sealed record YmirCommandReceipt(
    string ReceiptId,
    string SessionId,
    string SessionGeneration,
    string CommandId,
    string CommandKind,
    YmirCommandOutcome Outcome,
    YmirCommandError Error,
    long BeforeRevision,
    long AfterRevision,
    long ObservedStepIndex,
    IReadOnlyList<string> ProducedFactIds);

public enum YmirContactFactKind
{
    Begin,
    Hit,
    End
}

public sealed record YmirContactFact(
    string FactId,
    string ContactId,
    YmirContactFactKind Kind,
    string SessionId,
    string SessionGeneration,
    long StepIndex,
    string BodyA,
    long BodyAGeneration,
    string BodyB,
    long BodyBGeneration,
    Vec2? Point,
    Vec2? Normal,
    float? Penetration,
    float? RelativeSpeed);

public sealed record YmirSessionStepResult(
    YmirCommandReceipt Receipt,
    YmirWorld World,
    long Revision,
    long StepIndex,
    IReadOnlyList<YmirContactFact> ContactFacts);

public sealed record YmirSessionCircleCastQuery(
    string QueryId,
    long ExpectedRevision,
    Vec2 Origin,
    Vec2 Direction,
    float Distance,
    float Radius,
    IReadOnlyList<string> CandidateBodyIds);

public sealed record YmirSessionCircleOverlapQuery(
    string QueryId,
    long ExpectedRevision,
    Vec2 Center,
    float Radius,
    IReadOnlyList<string> CandidateBodyIds);

public enum YmirQueryError
{
    None,
    StaleRevision,
    UnknownBody,
    InvalidQuery,
    SessionDisposed,
    SessionFaulted
}

public sealed record YmirSessionCircleCastResult(
    string QueryId,
    string SessionId,
    string SessionGeneration,
    long ObservedRevision,
    long ObservedStepIndex,
    YmirQueryError Error,
    IReadOnlyList<CircleCastHit> Hits);

public sealed record YmirSessionCircleOverlapResult(
    string QueryId,
    string SessionId,
    string SessionGeneration,
    long ObservedRevision,
    long ObservedStepIndex,
    YmirQueryError Error,
    IReadOnlyList<CircleOverlapHit> Hits);

public sealed record YmirSessionInfo(
    string SessionId,
    string SessionGeneration,
    long Revision,
    long StepIndex,
    bool IsDisposed,
    bool IsFaulted);
