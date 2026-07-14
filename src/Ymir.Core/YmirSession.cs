using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using Ymir.Box3D;

namespace Ymir.Core;

/// <summary>
/// Owns one retained Box3D world. Ordinary operations are explicit commands;
/// complete snapshots are accepted only while the session is created.
/// </summary>
public sealed class YmirSession : IDisposable
{
    private readonly object _gate = new();
    private readonly Box3DSession _native = new();
    private readonly Dictionary<string, ulong> _activeKeysById = new(StringComparer.Ordinal);
    private readonly Dictionary<ulong, string> _idsByKey = new();
    private readonly Dictionary<ulong, long> _generationsByKey = new();
    private readonly Dictionary<string, PhysicsBody> _bodies = new(StringComparer.Ordinal);
    private readonly Dictionary<Box3DContactKey, ActiveEpisode> _episodesByNativeContact = new();
    private readonly Dictionary<string, LedgerEntry> _ledger = new(StringComparer.Ordinal);
    private readonly string _sessionId;
    private readonly string _sessionGeneration;
    private ulong _nextNativeKey = 1;
    private long _nextBodyGeneration = 1;
    private long _revision;
    private long _stepIndex;
    private float _time;
    private RadialField[] _fields = [];
    private bool _disposed;
    private bool _faulted;

    private YmirSession(YmirSessionCreateRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        if (string.IsNullOrWhiteSpace(request.SessionId))
        {
            throw new ArgumentException("Session id is required.", nameof(request));
        }
        if (!float.IsFinite(request.InitialTime) || request.InitialTime < 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "Initial time must be finite and non-negative.");
        }

        _sessionId = request.SessionId;
        _sessionGeneration = Guid.NewGuid().ToString("N", CultureInfo.InvariantCulture);
        _time = request.InitialTime;

        var initial = request.InitialBodies ?? throw new ArgumentNullException(nameof(request.InitialBodies));
        ValidateUniqueBodyIds(initial);
        try
        {
            foreach (var body in initial.OrderBy(body => body.Id, StringComparer.Ordinal))
            {
                ValidateBody(body, allowTorque: false);
                SpawnCore(body);
            }
        }
        catch
        {
            _native.Dispose();
            _disposed = true;
            throw;
        }
    }

    public static YmirSession Create(YmirSessionCreateRequest request) => new(request);

    public YmirSessionInfo Info
    {
        get
        {
            lock (_gate)
            {
                return new YmirSessionInfo(
                    _sessionId, _sessionGeneration, _revision, _stepIndex, _disposed, _faulted);
            }
        }
    }

    public YmirCommandReceipt Spawn(YmirSpawnBodyCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "body.spawn", writer => WriteBody(writer, command.Body));
            if (Preflight(command.Header, fingerprint, "body.spawn") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "body.spawn", fingerprint);
            }
            if (!TryValidateBody(command.Body, allowTorque: false))
            {
                return RejectAndStore(command.Header, fingerprint, "body.spawn", YmirCommandError.InvalidCommand);
            }
            if (_activeKeysById.ContainsKey(command.Body.Id))
            {
                return RejectAndStore(command.Header, fingerprint, "body.spawn", YmirCommandError.DuplicateBody);
            }

            SpawnCore(command.Body);
            return AcceptAndStore(command.Header, fingerprint, "body.spawn");
        }
    }

    public YmirCommandReceipt Remove(YmirRemoveBodyCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "body.remove", writer => WriteString(writer, command.BodyId));
            if (Preflight(command.Header, fingerprint, "body.remove") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "body.remove", fingerprint);
            }
            if (string.IsNullOrWhiteSpace(command.BodyId) ||
                !_activeKeysById.TryGetValue(command.BodyId, out var key))
            {
                return RejectAndStore(command.Header, fingerprint, "body.remove", YmirCommandError.UnknownBody);
            }

            RunNative(() =>
            {
                _native.Remove(key);
                _activeKeysById.Remove(command.BodyId);
                _bodies.Remove(command.BodyId);
            });
            return AcceptAndStore(command.Header, fingerprint, "body.remove");
        }
    }

    public YmirCommandReceipt Teleport(YmirTeleportBodyCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "body.teleport", writer =>
            {
                WriteString(writer, command.BodyId);
                Write(writer, command.Position);
                Write(writer, command.Direction);
            });
            if (Preflight(command.Header, fingerprint, "body.teleport") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "body.teleport", fingerprint);
            }
            if (!TryBody(command.BodyId, out var key, out var body))
            {
                return RejectAndStore(command.Header, fingerprint, "body.teleport", YmirCommandError.UnknownBody);
            }
            if (!Finite(command.Position) || !ValidDirection(command.Direction))
            {
                return RejectAndStore(command.Header, fingerprint, "body.teleport", YmirCommandError.InvalidCommand);
            }

            var direction = Normalize(command.Direction);
            RunNative(() =>
            {
                _native.Teleport(key, command.Position.X, command.Position.Y, direction.X, direction.Y);
                _bodies[command.BodyId] = body with { Position = command.Position, Direction = direction };
                UpdateBodyProjection(_native.CopyBodies());
            });
            return AcceptAndStore(command.Header, fingerprint, "body.teleport");
        }
    }

    public YmirCommandReceipt SetVelocity(YmirSetBodyVelocityCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "body.set_velocity", writer =>
            {
                WriteString(writer, command.BodyId);
                Write(writer, command.LinearVelocity);
                writer.Write(command.AngularVelocity);
            });
            if (Preflight(command.Header, fingerprint, "body.set_velocity") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "body.set_velocity", fingerprint);
            }
            if (!TryBody(command.BodyId, out var key, out var body))
            {
                return RejectAndStore(command.Header, fingerprint, "body.set_velocity", YmirCommandError.UnknownBody);
            }
            if (body.IsStatic)
            {
                return RejectAndStore(command.Header, fingerprint, "body.set_velocity", YmirCommandError.StaticBody);
            }
            if (!Finite(command.LinearVelocity) || !float.IsFinite(command.AngularVelocity))
            {
                return RejectAndStore(command.Header, fingerprint, "body.set_velocity", YmirCommandError.InvalidCommand);
            }

            RunNative(() =>
            {
                _native.SetVelocity(key, command.LinearVelocity.X, command.LinearVelocity.Y, command.AngularVelocity);
                _bodies[command.BodyId] = body with
                {
                    Velocity = command.LinearVelocity,
                    AngularVelocity = command.AngularVelocity
                };
                UpdateBodyProjection(_native.CopyBodies());
            });
            return AcceptAndStore(command.Header, fingerprint, "body.set_velocity");
        }
    }

    public YmirCommandReceipt Configure(YmirConfigureBodyCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "body.configure", writer =>
            {
                WriteString(writer, command.BodyId);
                writer.Write(command.Radius);
                writer.Write(command.Mass);
                writer.Write(command.IsStatic);
                writer.Write(command.Restitution);
            });
            if (Preflight(command.Header, fingerprint, "body.configure") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "body.configure", fingerprint);
            }
            if (!TryBody(command.BodyId, out var key, out var body))
            {
                return RejectAndStore(command.Header, fingerprint, "body.configure", YmirCommandError.UnknownBody);
            }
            if (!float.IsFinite(command.Radius) || command.Radius <= 0.0f ||
                !float.IsFinite(command.Mass) || (!command.IsStatic && command.Mass <= 0.0f) ||
                !float.IsFinite(command.Restitution) || command.Restitution < 0.0f)
            {
                return RejectAndStore(command.Header, fingerprint, "body.configure", YmirCommandError.InvalidCommand);
            }

            RunNative(() =>
            {
                _native.Configure(key, command.Radius, command.Mass, command.Restitution, command.IsStatic);
                _bodies[command.BodyId] = body with
                {
                    Radius = command.Radius,
                    Mass = command.Mass,
                    IsStatic = command.IsStatic,
                    Restitution = command.Restitution
                };
                UpdateBodyProjection(_native.CopyBodies());
            });
            return AcceptAndStore(command.Header, fingerprint, "body.configure");
        }
    }

    public YmirCommandReceipt ApplyForce(YmirApplyForceCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "body.apply_force", writer =>
            {
                WriteString(writer, command.BodyId);
                Write(writer, command.Force);
            });
            if (Preflight(command.Header, fingerprint, "body.apply_force") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "body.apply_force", fingerprint);
            }
            if (!TryBody(command.BodyId, out var key, out var body))
            {
                return RejectAndStore(command.Header, fingerprint, "body.apply_force", YmirCommandError.UnknownBody);
            }
            if (body.IsStatic)
            {
                return RejectAndStore(command.Header, fingerprint, "body.apply_force", YmirCommandError.StaticBody);
            }
            if (!Finite(command.Force))
            {
                return RejectAndStore(command.Header, fingerprint, "body.apply_force", YmirCommandError.InvalidCommand);
            }

            RunNative(() => _native.ApplyForce(key, command.Force.X, command.Force.Y));
            return AcceptAndStore(command.Header, fingerprint, "body.apply_force");
        }
    }

    public YmirCommandReceipt ApplyTorque(YmirApplyTorqueCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "body.apply_torque", writer =>
            {
                WriteString(writer, command.BodyId);
                writer.Write(command.Torque);
            });
            if (Preflight(command.Header, fingerprint, "body.apply_torque") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "body.apply_torque", fingerprint);
            }
            if (!TryBody(command.BodyId, out var key, out var body))
            {
                return RejectAndStore(command.Header, fingerprint, "body.apply_torque", YmirCommandError.UnknownBody);
            }
            if (body.IsStatic)
            {
                return RejectAndStore(command.Header, fingerprint, "body.apply_torque", YmirCommandError.StaticBody);
            }
            if (!float.IsFinite(command.Torque))
            {
                return RejectAndStore(command.Header, fingerprint, "body.apply_torque", YmirCommandError.InvalidCommand);
            }

            RunNative(() => _native.ApplyTorque(key, command.Torque));
            return AcceptAndStore(command.Header, fingerprint, "body.apply_torque");
        }
    }

    public YmirSessionStepResult Step(YmirStepSessionCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var submittedFields = command.Fields;
            var fieldsValid = submittedFields != null && FieldsAreValid(submittedFields);
            var orderedFields = fieldsValid
                ? submittedFields!.OrderBy(field => field.Id, StringComparer.Ordinal).ToArray()
                : [];
            var fingerprint = Fingerprint(command.Header, "session.step", writer =>
            {
                writer.Write(command.DeltaTime);
                writer.Write(command.SubstepCount);
                if (submittedFields == null)
                {
                    writer.Write(-1);
                    return;
                }
                var fields = fieldsValid ? orderedFields : submittedFields;
                writer.Write(fields.Count);
                foreach (var field in fields)
                {
                    WriteField(writer, field);
                }
            });
            if (Preflight(command.Header, fingerprint, "session.step") is { } previous)
            {
                if (previous.Result is YmirSessionStepResult replay)
                {
                    return replay;
                }
                var rejected = ReceiptFrom(previous, command.Header, "session.step", fingerprint);
                return new YmirSessionStepResult(rejected, SnapshotCore(), _revision, _stepIndex, []);
            }
            if (!float.IsFinite(command.DeltaTime) || command.DeltaTime <= 0.0f ||
                command.SubstepCount <= 0 || !fieldsValid)
            {
                var rejected = RejectAndStore(command.Header, fingerprint, "session.step", YmirCommandError.InvalidCommand);
                return new YmirSessionStepResult(rejected, SnapshotCore(), _revision, _stepIndex, []);
            }

            var nativeFields = orderedFields
                .Select(field => new Box3DField(field.Position.X, field.Position.Y, field.Strength, field.Radius))
                .ToArray();
            YmirContactFact[] facts;
            try
            {
                _native.Step(command.DeltaTime, nativeFields, command.SubstepCount);
                UpdateBodyProjection(_native.CopyBodies());
                _time += command.DeltaTime;
                _fields = orderedFields;
                _stepIndex += 1;
                _revision += 1;
                facts = ProjectFacts(_native.CopyContactEvents()).ToArray();
                PruneRetiredIdentity();
            }
            catch
            {
                Poison();
                throw;
            }
            try
            {
                var receipt = AcceptedReceipt(command.Header, "session.step", _revision - 1, _revision,
                    facts.Select(fact => fact.FactId).ToArray());
                var result = new YmirSessionStepResult(receipt, SnapshotCore(), _revision, _stepIndex, facts);
                _ledger.Add(command.Header.CommandId, new LedgerEntry(fingerprint, result));
                return result;
            }
            catch
            {
                Poison();
                throw;
            }
        }
    }

    public YmirWorld Snapshot()
    {
        lock (_gate)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);
            return SnapshotCore();
        }
    }

    public YmirCommandReceipt Dispose(YmirDisposeSessionCommand command)
    {
        ArgumentNullException.ThrowIfNull(command);
        lock (_gate)
        {
            var fingerprint = Fingerprint(command.Header, "session.dispose", _ => { });
            if (Preflight(command.Header, fingerprint, "session.dispose") is { } previous)
            {
                return ReceiptFrom(previous, command.Header, "session.dispose", fingerprint);
            }

            var before = _revision;
            _native.Dispose();
            _disposed = true;
            _revision += 1;
            var receipt = AcceptedReceipt(command.Header, "session.dispose", before, _revision, []);
            _ledger.Add(command.Header.CommandId, new LedgerEntry(fingerprint, receipt));
            return receipt;
        }
    }

    public void Dispose()
    {
        lock (_gate)
        {
            if (_disposed)
            {
                return;
            }
            _native.Dispose();
            _disposed = true;
        }
    }

    private void SpawnCore(PhysicsBody body)
    {
        var key = NextKey();
        var generation = _nextBodyGeneration++;
        var canonicalBody = body with { Torque = 0.0f, Direction = DirectionOrDefault(body.Direction) };
        RunNative(() =>
        {
            _native.Spawn(ToNative(key, canonicalBody));
            _activeKeysById.Add(canonicalBody.Id, key);
            _idsByKey.Add(key, canonicalBody.Id);
            _generationsByKey.Add(key, generation);
            _bodies.Add(canonicalBody.Id, canonicalBody);
        });
    }

    private IReadOnlyList<YmirContactFact> ProjectFacts(IReadOnlyList<Box3DContactFact> nativeFacts)
    {
        var projected = new List<ProjectedFact>(nativeFacts.Count);
        var ephemeralEpisodes = new Dictionary<Box3DContactKey, ActiveEpisode>();
        var newEpisodeOrdinals = new Dictionary<ContactPair, long>();
        var factOrdinals = new Dictionary<(
            ContactPair Pair,
            long StartStep,
            long EpisodeOrdinal,
            YmirContactFactKind Kind), int>();
        foreach (var native in nativeFacts)
        {
            var canonical = CanonicalizeContact(native);
            var pair = new ContactPair(
                canonical.BodyA,
                canonical.BodyAGeneration,
                canonical.BodyB,
                canonical.BodyBGeneration);
            ActiveEpisode episode;
            if (native.Kind == Box3DContactKind.Begin)
            {
                if (!_episodesByNativeContact.TryGetValue(native.ContactKey, out episode!))
                {
                    episode = NewEpisode(pair, newEpisodeOrdinals);
                    _episodesByNativeContact[native.ContactKey] = episode;
                }
            }
            else if (!_episodesByNativeContact.TryGetValue(native.ContactKey, out episode!) &&
                     !ephemeralEpisodes.TryGetValue(native.ContactKey, out episode!))
            {
                episode = NewEpisode(pair, newEpisodeOrdinals);
                ephemeralEpisodes.Add(native.ContactKey, episode);
            }

            var kind = native.Kind switch
            {
                Box3DContactKind.Begin => YmirContactFactKind.Begin,
                Box3DContactKind.Hit => YmirContactFactKind.Hit,
                Box3DContactKind.End => YmirContactFactKind.End,
                _ => throw new InvalidOperationException($"Unknown Box3D contact fact kind {native.Kind}.")
            };
            var ordinalKey = (pair, episode.StartStepIndex, episode.Ordinal, kind);
            var ordinal = factOrdinals.GetValueOrDefault(ordinalKey);
            factOrdinals[ordinalKey] = ordinal + 1;
            var pairId = PairId(pair);
            var contactId = $"{_sessionGeneration}:contact:{pairId}:{episode.StartStepIndex}:{episode.Ordinal}";
            var factId = $"{_sessionGeneration}:step:{_stepIndex}:contact:{pairId}:{episode.StartStepIndex}:{episode.Ordinal}:{kind.ToString().ToLowerInvariant()}:{ordinal}";
            var hasDetails = native.HasDetails;
            var fact = new YmirContactFact(
                factId,
                contactId,
                kind,
                _sessionId,
                _sessionGeneration,
                _stepIndex,
                canonical.BodyA,
                canonical.BodyAGeneration,
                canonical.BodyB,
                canonical.BodyBGeneration,
                hasDetails ? canonical.Point : null,
                hasDetails ? canonical.Normal : null,
                hasDetails ? native.Penetration : null,
                hasDetails ? native.RelativeSpeed : null);
            projected.Add(new ProjectedFact(fact, episode.StartStepIndex, episode.Ordinal, ordinal));

            if (native.Kind == Box3DContactKind.End)
            {
                _episodesByNativeContact.Remove(native.ContactKey);
                ephemeralEpisodes.Remove(native.ContactKey);
            }
        }

        return projected
            .OrderBy(item => item.Fact.BodyA, StringComparer.Ordinal)
            .ThenBy(item => item.Fact.BodyAGeneration)
            .ThenBy(item => item.Fact.BodyB, StringComparer.Ordinal)
            .ThenBy(item => item.Fact.BodyBGeneration)
            .ThenBy(item => item.EpisodeStartStep)
            .ThenBy(item => item.EpisodeOrdinal)
            .ThenBy(item => FactKindOrder(item.Fact.Kind))
            .ThenBy(item => item.Ordinal)
            .Select(item => item.Fact)
            .ToArray();
    }

    private CanonicalContact CanonicalizeContact(Box3DContactFact fact)
    {
        var idA = ResolveId(fact.StableIdA);
        var idB = ResolveId(fact.StableIdB);
        var generationA = _generationsByKey[fact.StableIdA];
        var generationB = _generationsByKey[fact.StableIdB];
        var swap = StringComparer.Ordinal.Compare(idA, idB) > 0 ||
                   (StringComparer.Ordinal.Equals(idA, idB) && generationA > generationB);
        return swap
            ? new CanonicalContact(
                idB, generationB, idA, generationA,
                new Vec2(fact.PointX, fact.PointZ),
                new Vec2(-fact.NormalX, -fact.NormalZ))
            : new CanonicalContact(
                idA, generationA, idB, generationB,
                new Vec2(fact.PointX, fact.PointZ),
                new Vec2(fact.NormalX, fact.NormalZ));
    }

    private void UpdateBodyProjection(IReadOnlyList<Box3DBodyState> states)
    {
        foreach (var state in states)
        {
            var id = ResolveId(state.StableId);
            if (!_bodies.TryGetValue(id, out var body) || _activeKeysById.GetValueOrDefault(id) != state.StableId)
            {
                throw new InvalidOperationException($"Box3D returned inactive body key {state.StableId}.");
            }
            _bodies[id] = body with
            {
                Position = new Vec2(state.PositionX, state.PositionZ),
                Velocity = new Vec2(state.VelocityX, state.VelocityZ),
                Direction = new Vec2(state.DirectionX, state.DirectionZ),
                AngularVelocity = state.AngularVelocity,
                Torque = 0.0f
            };
        }
    }

    private LedgerEntry? Preflight(YmirCommandHeader header, string fingerprint, string kind)
    {
        ValidateHeader(header);
        if (_ledger.TryGetValue(header.CommandId, out var existing))
        {
            return existing.Fingerprint == fingerprint
                ? existing
                : new LedgerEntry(fingerprint, ConflictReceipt(header, kind, fingerprint));
        }
        if (_faulted)
        {
            var rejected = RejectedReceipt(header, kind, YmirCommandError.SessionFaulted);
            var entry = new LedgerEntry(fingerprint, rejected);
            _ledger.Add(header.CommandId, entry);
            return entry;
        }
        if (_disposed)
        {
            var rejected = RejectedReceipt(header, kind, YmirCommandError.SessionDisposed);
            var entry = new LedgerEntry(fingerprint, rejected);
            _ledger.Add(header.CommandId, entry);
            return entry;
        }
        if (header.ExpectedRevision != _revision)
        {
            var rejected = RejectedReceipt(header, kind, YmirCommandError.StaleRevision);
            var entry = new LedgerEntry(fingerprint, rejected);
            _ledger.Add(header.CommandId, entry);
            return entry;
        }
        return null;
    }

    private YmirCommandReceipt ReceiptFrom(
        LedgerEntry entry,
        YmirCommandHeader header,
        string kind,
        string fingerprint)
    {
        if (entry.Fingerprint != fingerprint)
        {
            return RejectedReceipt(header, kind, YmirCommandError.CommandIdConflict);
        }
        return entry.Result switch
        {
            YmirCommandReceipt receipt => receipt,
            YmirSessionStepResult step => step.Receipt,
            _ => throw new InvalidOperationException("Unknown Ymir receipt ledger entry.")
        };
    }

    private YmirCommandReceipt AcceptAndStore(YmirCommandHeader header, string fingerprint, string kind)
    {
        try
        {
            var before = _revision;
            _revision += 1;
            var receipt = AcceptedReceipt(header, kind, before, _revision, []);
            _ledger.Add(header.CommandId, new LedgerEntry(fingerprint, receipt));
            return receipt;
        }
        catch
        {
            Poison();
            throw;
        }
    }

    private YmirCommandReceipt RejectAndStore(
        YmirCommandHeader header,
        string fingerprint,
        string kind,
        YmirCommandError error)
    {
        var receipt = RejectedReceipt(header, kind, error);
        _ledger.Add(header.CommandId, new LedgerEntry(fingerprint, receipt));
        return receipt;
    }

    private YmirCommandReceipt AcceptedReceipt(
        YmirCommandHeader header,
        string kind,
        long before,
        long after,
        IReadOnlyList<string> facts) =>
        new(
            $"{_sessionGeneration}:receipt:{header.CommandId}",
            _sessionId,
            _sessionGeneration,
            header.CommandId,
            kind,
            YmirCommandOutcome.Accepted,
            YmirCommandError.None,
            before,
            after,
            _stepIndex,
            facts);

    private YmirCommandReceipt RejectedReceipt(
        YmirCommandHeader header,
        string kind,
        YmirCommandError error) =>
        new(
            $"{_sessionGeneration}:receipt:{header.CommandId}",
            _sessionId,
            _sessionGeneration,
            header.CommandId,
            kind,
            YmirCommandOutcome.Rejected,
            error,
            _revision,
            _revision,
            _stepIndex,
            []);

    private YmirCommandReceipt ConflictReceipt(
        YmirCommandHeader header,
        string kind,
        string fingerprint)
    {
        var conflictId = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(fingerprint)))
            .ToLowerInvariant();
        return new YmirCommandReceipt(
            $"{_sessionGeneration}:receipt-conflict:{header.CommandId}:{conflictId}",
            _sessionId,
            _sessionGeneration,
            header.CommandId,
            kind,
            YmirCommandOutcome.Rejected,
            YmirCommandError.CommandIdConflict,
            _revision,
            _revision,
            _stepIndex,
            []);
    }

    private YmirWorld SnapshotCore() => new(
        _time,
        _bodies.Values.OrderBy(body => body.Id, StringComparer.Ordinal).ToArray(),
        _fields.ToArray());

    private bool TryBody(string? id, out ulong key, out PhysicsBody body)
    {
        if (!string.IsNullOrWhiteSpace(id) &&
            _activeKeysById.TryGetValue(id, out key) &&
            _bodies.TryGetValue(id, out body!))
        {
            return true;
        }
        key = 0;
        body = null!;
        return false;
    }

    private string ResolveId(ulong key) => _idsByKey.TryGetValue(key, out var id)
        ? id
        : throw new InvalidOperationException($"Box3D returned unknown stable body key {key}.");

    private ActiveEpisode NewEpisode(ContactPair pair, IDictionary<ContactPair, long> ordinals)
    {
        var ordinal = (ordinals.TryGetValue(pair, out var previous) ? previous : 0) + 1;
        ordinals[pair] = ordinal;
        return new ActiveEpisode(_stepIndex, ordinal, pair);
    }

    private void PruneRetiredIdentity()
    {
        var activeKeys = _activeKeysById.Values.ToHashSet();
        foreach (var key in _idsByKey.Keys.Where(key => !activeKeys.Contains(key)).ToArray())
        {
            var id = _idsByKey[key];
            var generation = _generationsByKey[key];
            if (_episodesByNativeContact.Values.Any(active =>
                    (active.Pair.BodyA == id && active.Pair.BodyAGeneration == generation) ||
                    (active.Pair.BodyB == id && active.Pair.BodyBGeneration == generation)))
            {
                throw new InvalidOperationException(
                    $"Box3D did not close every contact episode for removed body '{id}' generation {generation}.");
            }
            _idsByKey.Remove(key);
            _generationsByKey.Remove(key);
        }
    }

    private void RunNative(Action operation)
    {
        try
        {
            operation();
        }
        catch
        {
            Poison();
            throw;
        }
    }

    private void Poison()
    {
        _native.Dispose();
        _disposed = true;
        _faulted = true;
    }

    private ulong NextKey()
    {
        if (_nextNativeKey == 0)
        {
            throw new InvalidOperationException("Ymir exhausted its native body key space.");
        }
        return _nextNativeKey++;
    }

    private static Box3DBody ToNative(ulong key, PhysicsBody body)
    {
        var direction = DirectionOrDefault(body.Direction);
        return new Box3DBody(
            key,
            body.Position.X,
            body.Position.Y,
            body.Velocity.X,
            body.Velocity.Y,
            direction.X,
            direction.Y,
            body.AngularVelocity,
            0.0f,
            body.Radius,
            body.Mass,
            body.Restitution,
            body.IsStatic);
    }

    private static Vec2 DirectionOrDefault(Vec2? direction) =>
        direction is { } value && ValidDirection(value) ? Normalize(value) : new Vec2(0.0f, 1.0f);

    private static Vec2 Normalize(Vec2 value)
    {
        var inverseLength = 1.0f / MathF.Sqrt(value.X * value.X + value.Y * value.Y);
        return new Vec2(value.X * inverseLength, value.Y * inverseLength);
    }

    private static void ValidateHeader(YmirCommandHeader header)
    {
        ArgumentNullException.ThrowIfNull(header);
        if (string.IsNullOrWhiteSpace(header.CommandId))
        {
            throw new ArgumentException("Command id is required.", nameof(header));
        }
        if (header.ExpectedRevision < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(header), "Expected revision must be non-negative.");
        }
    }

    private static void ValidateUniqueBodyIds(IReadOnlyList<PhysicsBody> bodies)
    {
        var ids = new HashSet<string>(StringComparer.Ordinal);
        foreach (var body in bodies)
        {
            if (!ids.Add(body.Id))
            {
                throw new ArgumentException($"Ymir body id '{body.Id}' occurs more than once.", nameof(bodies));
            }
        }
    }

    private static bool TryValidateBody(PhysicsBody body, bool allowTorque)
    {
        try
        {
            ValidateBody(body, allowTorque);
            return true;
        }
        catch (ArgumentException)
        {
            return false;
        }
    }

    private static void ValidateBody(PhysicsBody body, bool allowTorque)
    {
        ArgumentNullException.ThrowIfNull(body);
        if (string.IsNullOrWhiteSpace(body.Id) || !Finite(body.Position) || !Finite(body.Velocity) ||
            !float.IsFinite(body.Radius) || body.Radius <= 0.0f ||
            !float.IsFinite(body.Mass) || (!body.IsStatic && body.Mass <= 0.0f) ||
            !float.IsFinite(body.Restitution) || body.Restitution < 0.0f ||
            (body.Direction is { } direction && !ValidDirection(direction)) ||
            !float.IsFinite(body.AngularVelocity) || !float.IsFinite(body.Torque) ||
            (!allowTorque && body.Torque != 0.0f))
        {
            throw new ArgumentException("Body definition is not valid for a retained Ymir session.", nameof(body));
        }
    }

    private static bool FieldsAreValid(IReadOnlyList<RadialField> fields)
    {
        var ids = new HashSet<string>(StringComparer.Ordinal);
        return fields.All(field => field != null &&
            !string.IsNullOrWhiteSpace(field.Id) && ids.Add(field.Id) && Finite(field.Position) &&
            float.IsFinite(field.Strength) && float.IsFinite(field.Radius) && field.Radius >= 0.0f);
    }

    private static bool Finite(Vec2 value) => float.IsFinite(value.X) && float.IsFinite(value.Y);

    private static bool ValidDirection(Vec2 value) =>
        Finite(value) && value.X * value.X + value.Y * value.Y > 1.0e-8f;

    private static string Fingerprint(
        YmirCommandHeader header,
        string kind,
        Action<BinaryWriter> writePayload)
    {
        using var stream = new MemoryStream();
        using (var writer = new BinaryWriter(stream, Encoding.UTF8, leaveOpen: true))
        {
            writer.Write(1);
            WriteString(writer, kind);
            writer.Write(header.ExpectedRevision);
            writePayload(writer);
        }
        return Convert.ToHexString(SHA256.HashData(stream.GetBuffer().AsSpan(0, checked((int)stream.Length))))
            .ToLowerInvariant();
    }

    private static void WriteBody(BinaryWriter writer, PhysicsBody? body)
    {
        writer.Write(body != null);
        if (body == null)
        {
            return;
        }
        WriteString(writer, body.Id);
        Write(writer, body.Position);
        Write(writer, body.Velocity);
        writer.Write(body.Radius);
        writer.Write(body.Mass);
        writer.Write(body.IsStatic);
        writer.Write(body.Restitution);
        writer.Write(body.Direction.HasValue);
        if (body.Direction is { } direction)
        {
            Write(writer, direction);
        }
        writer.Write(body.AngularVelocity);
        writer.Write(body.Torque);
    }

    private static void WriteField(BinaryWriter writer, RadialField? field)
    {
        writer.Write(field != null);
        if (field == null)
        {
            return;
        }
        WriteString(writer, field.Id);
        Write(writer, field.Position);
        writer.Write(field.Strength);
        writer.Write(field.Radius);
    }

    private static void Write(BinaryWriter writer, Vec2 value)
    {
        writer.Write(value.X);
        writer.Write(value.Y);
    }

    private static void WriteString(BinaryWriter writer, string? value)
    {
        writer.Write(value != null);
        if (value != null)
        {
            writer.Write(value);
        }
    }

    private static string PairId(ContactPair pair)
    {
        var source = $"{pair.BodyA.Length}:{pair.BodyA}:{pair.BodyAGeneration}:{pair.BodyB.Length}:{pair.BodyB}:{pair.BodyBGeneration}";
        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(source))).ToLowerInvariant();
    }

    private static int FactKindOrder(Box3DContactKind kind) => kind switch
    {
        Box3DContactKind.Begin => 0,
        Box3DContactKind.Hit => 1,
        Box3DContactKind.End => 2,
        _ => 3
    };

    private static int FactKindOrder(YmirContactFactKind kind) => kind switch
    {
        YmirContactFactKind.Begin => 0,
        YmirContactFactKind.Hit => 1,
        YmirContactFactKind.End => 2,
        _ => 3
    };

    private sealed record LedgerEntry(string Fingerprint, object Result);

    private sealed record ContactPair(string BodyA, long BodyAGeneration, string BodyB, long BodyBGeneration);

    private sealed record ActiveEpisode(long StartStepIndex, long Ordinal, ContactPair Pair);

    private sealed record ProjectedFact(
        YmirContactFact Fact,
        long EpisodeStartStep,
        long EpisodeOrdinal,
        int Ordinal);

    private sealed record CanonicalContact(
        string BodyA,
        long BodyAGeneration,
        string BodyB,
        long BodyBGeneration,
        Vec2 Point,
        Vec2 Normal);
}
