using System.Security.Cryptography;
using System.Text;

namespace Ymir.Core;

public static class YmirSessionCheckpointCodec
{
    private const uint Magic = 0x43534D59; // YMSC
    private const int Version = 1;
    private const int MaximumPayloadBytes = 256 * 1024 * 1024;
    private const int MaximumCollectionCount = 4 * 1024 * 1024;
    private const int MaximumStringBytes = 1024 * 1024;
    private const int ChecksumBytes = 32;

    public static byte[] Encode(YmirSessionCheckpoint checkpoint)
    {
        ArgumentNullException.ThrowIfNull(checkpoint);
        using var stream = new MemoryStream();
        using var writer = new BinaryWriter(stream, Encoding.UTF8, leaveOpen: true);
        writer.Write(Magic);
        writer.Write(Version);
        WriteString(writer, checkpoint.FormatId);
        WriteString(writer, checkpoint.RuntimeFingerprint);
        WriteString(writer, checkpoint.SessionId);
        WriteString(writer, checkpoint.SessionGeneration);
        writer.Write(checkpoint.Revision);
        writer.Write(checkpoint.StepIndex);
        writer.Write(checkpoint.Time);
        writer.Write(checkpoint.NextBodyGeneration);
        WriteBodies(writer, checkpoint.InitialBodies);
        writer.Write(checkpoint.InitialTime);
        WriteJournal(writer, checkpoint.Journal);
        WriteString(writer, checkpoint.JournalDigest);
        WriteWorld(writer, checkpoint.World);
        WriteEpisodes(writer, checkpoint.ActiveContactEpisodes);
        writer.Flush();
        if (stream.Length > MaximumPayloadBytes - ChecksumBytes)
            throw new InvalidOperationException("Ymir checkpoint exceeds the supported payload size.");
        var content = stream.ToArray();
        var checksum = SHA256.HashData(content);
        return [.. content, .. checksum];
    }

    public static YmirSessionCheckpoint Decode(ReadOnlySpan<byte> payload)
    {
        if (payload.Length <= ChecksumBytes || payload.Length > MaximumPayloadBytes)
            throw new InvalidDataException("Ymir checkpoint payload size is invalid.");
        var content = payload[..^ChecksumBytes];
        var expectedChecksum = payload[^ChecksumBytes..];
        Span<byte> actualChecksum = stackalloc byte[ChecksumBytes];
        SHA256.HashData(content, actualChecksum);
        if (!CryptographicOperations.FixedTimeEquals(actualChecksum, expectedChecksum))
            throw new InvalidDataException("Ymir checkpoint checksum is invalid.");
        try
        {
            using var stream = new MemoryStream(content.ToArray(), writable: false);
            using var reader = new BinaryReader(stream, Encoding.UTF8, leaveOpen: true);
            if (reader.ReadUInt32() != Magic || reader.ReadInt32() != Version)
                throw new InvalidDataException("Ymir checkpoint codec header is incompatible.");
            var checkpoint = new YmirSessionCheckpoint(
                ReadString(reader),
                ReadString(reader),
                ReadString(reader),
                ReadString(reader),
                reader.ReadInt64(),
                reader.ReadInt64(),
                reader.ReadSingle(),
                reader.ReadInt64(),
                ReadBodies(reader),
                reader.ReadSingle(),
                ReadJournal(reader),
                ReadString(reader),
                ReadWorld(reader),
                ReadEpisodes(reader));
            if (stream.Position != stream.Length)
                throw new InvalidDataException("Ymir checkpoint contains trailing data.");
            return checkpoint;
        }
        catch (InvalidDataException)
        {
            throw;
        }
        catch (Exception error) when (error is EndOfStreamException or IOException or ArgumentException or OverflowException)
        {
            throw new InvalidDataException("Ymir checkpoint payload is corrupt.", error);
        }
    }

    private static void WriteJournal(BinaryWriter writer, IReadOnlyList<YmirSessionJournalEntry> entries)
    {
        WriteCount(writer, entries?.Count ?? -1);
        foreach (var entry in entries!)
        {
            switch (entry)
            {
                case YmirSpawnBodyJournalEntry value:
                    writer.Write((byte)1); WriteHeader(writer, value.Command.Header); WriteBody(writer, value.Command.Body); break;
                case YmirRemoveBodyJournalEntry value:
                    writer.Write((byte)2); WriteHeader(writer, value.Command.Header); WriteString(writer, value.Command.BodyId); break;
                case YmirTeleportBodyJournalEntry value:
                    writer.Write((byte)3); WriteHeader(writer, value.Command.Header); WriteString(writer, value.Command.BodyId); WriteVec2(writer, value.Command.Position); WriteVec2(writer, value.Command.Direction); break;
                case YmirSetBodyVelocityJournalEntry value:
                    writer.Write((byte)4); WriteHeader(writer, value.Command.Header); WriteString(writer, value.Command.BodyId); WriteVec2(writer, value.Command.LinearVelocity); writer.Write(value.Command.AngularVelocity); break;
                case YmirConfigureBodyJournalEntry value:
                    writer.Write((byte)5); WriteHeader(writer, value.Command.Header); WriteString(writer, value.Command.BodyId); writer.Write(value.Command.Radius); writer.Write(value.Command.Mass); writer.Write(value.Command.IsStatic); writer.Write(value.Command.Restitution); writer.Write(value.Command.IsKinematic); break;
                case YmirApplyForceJournalEntry value:
                    writer.Write((byte)6); WriteHeader(writer, value.Command.Header); WriteString(writer, value.Command.BodyId); WriteVec2(writer, value.Command.Force); break;
                case YmirApplyTorqueJournalEntry value:
                    writer.Write((byte)7); WriteHeader(writer, value.Command.Header); WriteString(writer, value.Command.BodyId); writer.Write(value.Command.Torque); break;
                case YmirStepSessionJournalEntry value:
                    writer.Write((byte)8); WriteHeader(writer, value.Command.Header); writer.Write(value.Command.DeltaTime); WriteFields(writer, value.Command.Fields); writer.Write(value.Command.SubstepCount); break;
                default:
                    throw new InvalidOperationException($"Unknown Ymir checkpoint journal entry '{entry?.GetType().FullName ?? "<null>"}'.");
            }
        }
    }

    private static YmirSessionJournalEntry[] ReadJournal(BinaryReader reader)
    {
        var count = ReadCount(reader);
        var values = new YmirSessionJournalEntry[count];
        for (var index = 0; index < count; index++)
        {
            values[index] = reader.ReadByte() switch
            {
                1 => new YmirSpawnBodyJournalEntry(new YmirSpawnBodyCommand(ReadHeader(reader), ReadBody(reader)!)),
                2 => new YmirRemoveBodyJournalEntry(new YmirRemoveBodyCommand(ReadHeader(reader), ReadString(reader))),
                3 => new YmirTeleportBodyJournalEntry(new YmirTeleportBodyCommand(ReadHeader(reader), ReadString(reader), ReadVec2(reader), ReadVec2(reader))),
                4 => new YmirSetBodyVelocityJournalEntry(new YmirSetBodyVelocityCommand(ReadHeader(reader), ReadString(reader), ReadVec2(reader), reader.ReadSingle())),
                5 => new YmirConfigureBodyJournalEntry(new YmirConfigureBodyCommand(ReadHeader(reader), ReadString(reader), reader.ReadSingle(), reader.ReadSingle(), reader.ReadBoolean(), reader.ReadSingle(), reader.ReadBoolean())),
                6 => new YmirApplyForceJournalEntry(new YmirApplyForceCommand(ReadHeader(reader), ReadString(reader), ReadVec2(reader))),
                7 => new YmirApplyTorqueJournalEntry(new YmirApplyTorqueCommand(ReadHeader(reader), ReadString(reader), reader.ReadSingle())),
                8 => new YmirStepSessionJournalEntry(new YmirStepSessionCommand(ReadHeader(reader), reader.ReadSingle(), ReadFields(reader)!, reader.ReadInt32())),
                var kind => throw new InvalidDataException($"Unknown Ymir checkpoint journal kind {kind}.")
            };
        }
        return values;
    }

    private static void WriteWorld(BinaryWriter writer, YmirWorld world)
    {
        ArgumentNullException.ThrowIfNull(world);
        writer.Write(world.Time);
        WriteBodies(writer, world.Bodies);
        WriteFields(writer, world.Fields);
    }

    private static YmirWorld ReadWorld(BinaryReader reader)
    {
        var time = reader.ReadSingle();
        var bodies = ReadBodies(reader);
        var fields = ReadFields(reader) ?? throw new InvalidDataException("Ymir checkpoint world fields cannot be null.");
        return new YmirWorld(time, bodies, fields);
    }

    private static void WriteBodies(BinaryWriter writer, IReadOnlyList<PhysicsBody> bodies)
    {
        WriteCount(writer, bodies?.Count ?? -1);
        foreach (var body in bodies!) WriteBody(writer, body);
    }

    private static PhysicsBody[] ReadBodies(BinaryReader reader)
    {
        var count = ReadCount(reader);
        var values = new PhysicsBody[count];
        for (var index = 0; index < count; index++)
            values[index] = ReadBody(reader) ?? throw new InvalidDataException("Ymir body list contains null.");
        return values;
    }

    private static void WriteBody(BinaryWriter writer, PhysicsBody? body)
    {
        writer.Write(body != null);
        if (body == null) return;
        WriteString(writer, body.Id);
        WriteVec2(writer, body.Position);
        WriteVec2(writer, body.Velocity);
        writer.Write(body.Radius);
        writer.Write(body.Mass);
        writer.Write(body.IsStatic);
        writer.Write(body.Restitution);
        writer.Write(body.Direction.HasValue);
        if (body.Direction is { } direction) WriteVec2(writer, direction);
        writer.Write(body.AngularVelocity);
        writer.Write(body.Torque);
        writer.Write(body.IsKinematic);
        writer.Write(body.IsBullet);
        writer.Write(body.ParticipatesInFields);
        writer.Write(body.CollisionCategoryBits);
        writer.Write(body.CollisionMaskBits);
        writer.Write(body.CollisionGroupIndex);
    }

    private static PhysicsBody? ReadBody(BinaryReader reader)
    {
        if (!reader.ReadBoolean()) return null;
        var id = ReadString(reader);
        var position = ReadVec2(reader);
        var velocity = ReadVec2(reader);
        var radius = reader.ReadSingle();
        var mass = reader.ReadSingle();
        var isStatic = reader.ReadBoolean();
        var restitution = reader.ReadSingle();
        var direction = reader.ReadBoolean() ? ReadVec2(reader) : (Vec2?)null;
        return new PhysicsBody(
            id, position, velocity, radius, mass, isStatic, restitution, direction,
            reader.ReadSingle(), reader.ReadSingle(), reader.ReadBoolean(), reader.ReadBoolean(),
            reader.ReadBoolean(), reader.ReadUInt64(), reader.ReadUInt64(), reader.ReadInt32());
    }

    private static void WriteFields(BinaryWriter writer, IReadOnlyList<RadialField>? fields)
    {
        writer.Write(fields != null);
        if (fields == null) return;
        WriteCount(writer, fields.Count);
        foreach (var field in fields)
        {
            writer.Write(field != null);
            if (field == null) continue;
            WriteString(writer, field.Id);
            WriteVec2(writer, field.Position);
            writer.Write(field.Strength);
            writer.Write(field.Radius);
        }
    }

    private static RadialField[]? ReadFields(BinaryReader reader)
    {
        if (!reader.ReadBoolean()) return null;
        var count = ReadCount(reader);
        var values = new RadialField[count];
        for (var index = 0; index < count; index++)
        {
            if (!reader.ReadBoolean())
            {
                values[index] = null!;
                continue;
            }
            values[index] = new RadialField(ReadString(reader), ReadVec2(reader), reader.ReadSingle(), reader.ReadSingle());
        }
        return values;
    }

    private static void WriteEpisodes(BinaryWriter writer, IReadOnlyList<YmirContactEpisodeCheckpoint> episodes)
    {
        WriteCount(writer, episodes?.Count ?? -1);
        foreach (var value in episodes!)
        {
            WriteString(writer, value.BodyA); writer.Write(value.BodyAGeneration);
            WriteString(writer, value.BodyB); writer.Write(value.BodyBGeneration);
            writer.Write(value.StartStepIndex); writer.Write(value.Ordinal);
        }
    }

    private static YmirContactEpisodeCheckpoint[] ReadEpisodes(BinaryReader reader)
    {
        var count = ReadCount(reader);
        var values = new YmirContactEpisodeCheckpoint[count];
        for (var index = 0; index < count; index++)
            values[index] = new YmirContactEpisodeCheckpoint(
                ReadString(reader), reader.ReadInt64(), ReadString(reader), reader.ReadInt64(),
                reader.ReadInt64(), reader.ReadInt64());
        return values;
    }

    private static void WriteHeader(BinaryWriter writer, YmirCommandHeader header)
    {
        WriteString(writer, header.CommandId);
        writer.Write(header.ExpectedRevision);
    }

    private static YmirCommandHeader ReadHeader(BinaryReader reader) => new(ReadString(reader), reader.ReadInt64());

    private static void WriteVec2(BinaryWriter writer, Vec2 value)
    {
        writer.Write(value.X);
        writer.Write(value.Y);
    }

    private static Vec2 ReadVec2(BinaryReader reader) => new(reader.ReadSingle(), reader.ReadSingle());

    private static void WriteString(BinaryWriter writer, string value)
    {
        if (value == null) throw new InvalidOperationException("Ymir checkpoint strings cannot be null.");
        var bytes = Encoding.UTF8.GetBytes(value);
        if (bytes.Length > MaximumStringBytes)
            throw new InvalidOperationException("Ymir checkpoint string exceeds the supported size.");
        writer.Write(bytes.Length);
        writer.Write(bytes);
    }

    private static string ReadString(BinaryReader reader)
    {
        var length = reader.ReadInt32();
        if (length < 0 || length > MaximumStringBytes)
            throw new InvalidDataException("Ymir checkpoint string length is invalid.");
        var bytes = reader.ReadBytes(length);
        if (bytes.Length != length)
            throw new EndOfStreamException();
        return Encoding.UTF8.GetString(bytes);
    }

    private static void WriteCount(BinaryWriter writer, int count)
    {
        if (count < 0 || count > MaximumCollectionCount)
            throw new InvalidOperationException("Ymir checkpoint collection count is invalid.");
        writer.Write(count);
    }

    private static int ReadCount(BinaryReader reader)
    {
        var count = reader.ReadInt32();
        return count is < 0 or > MaximumCollectionCount
            ? throw new InvalidDataException("Ymir checkpoint collection count is invalid.")
            : count;
    }
}
