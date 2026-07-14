using Ymir.Box3D;

namespace Ymir.Core;

/// <summary>
/// Owns one retained Box3D world and its stable GameCult identity projection.
/// Callers own the session lifetime; body snapshots do not imply session identity.
/// </summary>
internal sealed class YmirWorldSession : IDisposable
{
    private readonly object _gate = new();
    private readonly Box3DSession _session = new();
    private readonly Dictionary<string, ulong> _keysById = new(StringComparer.Ordinal);
    private readonly Dictionary<ulong, string> _idsByKey = new();
    private ulong _nextKey = 1;
    private bool _disposed;

    public SimulationStepResult Step(SimulationStepRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        var result = Step(new SoASimulationStepRequest(
            request.DeltaTime,
            YmirSoAWorld.FromWorld(request.World)));

        return new SimulationStepResult(result.World.ToWorld(), result.Contacts);
    }

    public SoASimulationStepResult Step(SoASimulationStepRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        if (!float.IsFinite(request.DeltaTime) || request.DeltaTime <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "DeltaTime must be finite and positive.");
        }

        lock (_gate)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);
            var world = request.World.Clone();
            world.Validate();
            ValidateStableIds(world.BodyIds);

            var indicesByKey = new Dictionary<ulong, int>(world.BodyCount);
            foreach (var id in world.BodyIds.OrderBy(id => id, StringComparer.Ordinal))
            {
                GetOrCreateKey(id);
            }

            var bodies = new Box3DBody[world.BodyCount];
            for (var i = 0; i < world.BodyCount; i++)
            {
                var key = GetOrCreateKey(world.BodyIds[i]);
                indicesByKey.Add(key, i);
                bodies[i] = new Box3DBody(
                    key,
                    world.PositionX[i],
                    world.PositionY[i],
                    world.VelocityX[i],
                    world.VelocityY[i],
                    world.DirectionX[i],
                    world.DirectionY[i],
                    world.AngularVelocity[i],
                    world.Torque[i],
                    world.Radius[i],
                    world.Mass[i],
                    world.Restitution[i],
                    world.DynamicMask[i] <= 0.0f);
            }

            Array.Sort(bodies, static (left, right) => left.StableId.CompareTo(right.StableId));

            var fields = Enumerable.Range(0, world.FieldCount)
                .OrderBy(index => world.FieldIds[index], StringComparer.Ordinal)
                .Select(index => new Box3DField(
                    world.FieldPositionX[index],
                    world.FieldPositionY[index],
                    world.FieldStrength[index],
                    world.FieldRadius[index]))
                .ToArray();

            _session.SyncBodies(bodies);
            _session.Step(request.DeltaTime, fields);

            foreach (var body in _session.CopyBodies())
            {
                if (!indicesByKey.TryGetValue(body.StableId, out var index))
                {
                    throw new InvalidOperationException($"Box3D returned unknown stable body key {body.StableId}.");
                }

                world.PositionX[index] = body.PositionX;
                world.PositionY[index] = body.PositionZ;
                world.VelocityX[index] = body.VelocityX;
                world.VelocityY[index] = body.VelocityZ;
                world.DirectionX[index] = body.DirectionX;
                world.DirectionY[index] = body.DirectionZ;
                world.AngularVelocity[index] = body.AngularVelocity;
                world.Torque[index] = 0.0f;
            }

            var contacts = _session.CopyContacts()
                .Select(ProjectContact)
                .OrderBy(contact => contact.BodyA, StringComparer.Ordinal)
                .ThenBy(contact => contact.BodyB, StringComparer.Ordinal)
                .ThenBy(contact => contact.Point.X)
                .ThenBy(contact => contact.Point.Y)
                .ToArray();

            PruneInactiveIds(world.BodyIds);

            world.Time += request.DeltaTime;
            return new SoASimulationStepResult(world, contacts);
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

            _session.Dispose();
            _disposed = true;
        }
    }

    private ulong GetOrCreateKey(string id)
    {
        if (_keysById.TryGetValue(id, out var existing))
        {
            return existing;
        }

        if (_nextKey == 0)
        {
            throw new InvalidOperationException("Ymir exhausted its stable native body key space.");
        }

        var key = _nextKey++;
        _keysById.Add(id, key);
        _idsByKey.Add(key, id);
        return key;
    }

    private ContactEvent ProjectContact(Box3DContact contact)
    {
        var bodyA = ResolveId(contact.StableIdA);
        var bodyB = ResolveId(contact.StableIdB);
        var normal = new Vec2(contact.NormalX, contact.NormalZ);
        if (StringComparer.Ordinal.Compare(bodyA, bodyB) > 0)
        {
            (bodyA, bodyB) = (bodyB, bodyA);
            normal = new Vec2(-normal.X, -normal.Y);
        }

        return new ContactEvent(
            bodyA,
            bodyB,
            new Vec2(contact.PointX, contact.PointZ),
            normal,
            contact.Penetration,
            contact.RelativeSpeed);
    }

    private string ResolveId(ulong key)
    {
        if (!_idsByKey.TryGetValue(key, out var id))
        {
            throw new InvalidOperationException($"Box3D returned unknown stable body key {key}.");
        }

        return id;
    }

    private void PruneInactiveIds(IReadOnlyList<string> activeIds)
    {
        var active = activeIds.ToHashSet(StringComparer.Ordinal);
        foreach (var id in _keysById.Keys.Where(id => !active.Contains(id)).ToArray())
        {
            var key = _keysById[id];
            _keysById.Remove(id);
            _idsByKey.Remove(key);
        }
    }

    private static void ValidateStableIds(IReadOnlyList<string> bodyIds)
    {
        var unique = new HashSet<string>(StringComparer.Ordinal);
        foreach (var id in bodyIds)
        {
            if (!unique.Add(id))
            {
                throw new ArgumentException($"Ymir body id '{id}' occurs more than once.", nameof(bodyIds));
            }
        }
    }
}
