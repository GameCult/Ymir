using GameCult.Caching;
using GameCult.Caching.MessagePack;
using MessagePack;

namespace Ymir.Core;

[CultDocument("gamecult.ymir.world_state", "gamecult.ymir.world_state.v0")]
[MessagePackObject]
public sealed class YmirWorldStateDocumentV0
{
    [Key(0)]
    [CultName]
    public string WorldId { get; set; } = "default";

    [Key(1)]
    public float Time { get; set; }

    [Key(2)]
    public string[] BodyIds { get; set; } = [];

    [Key(3)]
    public float[] PositionX { get; set; } = [];

    [Key(4)]
    public float[] PositionY { get; set; } = [];

    [Key(5)]
    public float[] VelocityX { get; set; } = [];

    [Key(6)]
    public float[] VelocityY { get; set; } = [];

    [Key(7)]
    public float[] Radius { get; set; } = [];

    [Key(8)]
    public float[] Mass { get; set; } = [];

    [Key(9)]
    public float[] DynamicMask { get; set; } = [];

    [Key(10)]
    public float[] Restitution { get; set; } = [];

    [Key(11)]
    public string[] FieldIds { get; set; } = [];

    [Key(12)]
    public float[] FieldPositionX { get; set; } = [];

    [Key(13)]
    public float[] FieldPositionY { get; set; } = [];

    [Key(14)]
    public float[] FieldStrength { get; set; } = [];

    [Key(15)]
    public float[] FieldRadius { get; set; } = [];

    public static YmirWorldStateDocumentV0 FromSoA(string worldId, YmirSoAWorld world) => new()
    {
        WorldId = worldId,
        Time = world.Time,
        BodyIds = (string[])world.BodyIds.Clone(),
        PositionX = (float[])world.PositionX.Clone(),
        PositionY = (float[])world.PositionY.Clone(),
        VelocityX = (float[])world.VelocityX.Clone(),
        VelocityY = (float[])world.VelocityY.Clone(),
        Radius = (float[])world.Radius.Clone(),
        Mass = (float[])world.Mass.Clone(),
        DynamicMask = (float[])world.DynamicMask.Clone(),
        Restitution = (float[])world.Restitution.Clone(),
        FieldIds = (string[])world.FieldIds.Clone(),
        FieldPositionX = (float[])world.FieldPositionX.Clone(),
        FieldPositionY = (float[])world.FieldPositionY.Clone(),
        FieldStrength = (float[])world.FieldStrength.Clone(),
        FieldRadius = (float[])world.FieldRadius.Clone()
    };

    public YmirSoAWorld ToSoA() => new()
    {
        Time = Time,
        BodyIds = (string[])BodyIds.Clone(),
        PositionX = (float[])PositionX.Clone(),
        PositionY = (float[])PositionY.Clone(),
        VelocityX = (float[])VelocityX.Clone(),
        VelocityY = (float[])VelocityY.Clone(),
        Radius = (float[])Radius.Clone(),
        Mass = (float[])Mass.Clone(),
        DynamicMask = (float[])DynamicMask.Clone(),
        Restitution = (float[])Restitution.Clone(),
        FieldIds = (string[])FieldIds.Clone(),
        FieldPositionX = (float[])FieldPositionX.Clone(),
        FieldPositionY = (float[])FieldPositionY.Clone(),
        FieldStrength = (float[])FieldStrength.Clone(),
        FieldRadius = (float[])FieldRadius.Clone()
    };
}

[CultDocument("gamecult.ymir.world_state", "gamecult.ymir.world_state.v1")]
[MessagePackObject]
public sealed class YmirWorldStateDocumentV1
{
    [Key(0), CultName] public string WorldId { get; set; } = "default";
    [Key(1)] public float Time { get; set; }
    [Key(2)] public string[] BodyIds { get; set; } = [];
    [Key(3)] public float[] PositionX { get; set; } = [];
    [Key(4)] public float[] PositionY { get; set; } = [];
    [Key(5)] public float[] VelocityX { get; set; } = [];
    [Key(6)] public float[] VelocityY { get; set; } = [];
    [Key(7)] public float[] Radius { get; set; } = [];
    [Key(8)] public float[] Mass { get; set; } = [];
    [Key(9)] public float[] DynamicMask { get; set; } = [];
    [Key(10)] public float[] Restitution { get; set; } = [];
    [Key(11)] public string[] FieldIds { get; set; } = [];
    [Key(12)] public float[] FieldPositionX { get; set; } = [];
    [Key(13)] public float[] FieldPositionY { get; set; } = [];
    [Key(14)] public float[] FieldStrength { get; set; } = [];
    [Key(15)] public float[] FieldRadius { get; set; } = [];
    [Key(16)] public float[] DirectionX { get; set; } = [];
    [Key(17)] public float[] DirectionY { get; set; } = [];
    [Key(18)] public float[] AngularVelocity { get; set; } = [];

    public static YmirWorldStateDocumentV1 FromSoA(string worldId, YmirSoAWorld world) => new()
    {
        WorldId = worldId,
        Time = world.Time,
        BodyIds = (string[])world.BodyIds.Clone(),
        PositionX = (float[])world.PositionX.Clone(),
        PositionY = (float[])world.PositionY.Clone(),
        VelocityX = (float[])world.VelocityX.Clone(),
        VelocityY = (float[])world.VelocityY.Clone(),
        Radius = (float[])world.Radius.Clone(),
        Mass = (float[])world.Mass.Clone(),
        DynamicMask = (float[])world.DynamicMask.Clone(),
        Restitution = (float[])world.Restitution.Clone(),
        FieldIds = (string[])world.FieldIds.Clone(),
        FieldPositionX = (float[])world.FieldPositionX.Clone(),
        FieldPositionY = (float[])world.FieldPositionY.Clone(),
        FieldStrength = (float[])world.FieldStrength.Clone(),
        FieldRadius = (float[])world.FieldRadius.Clone(),
        DirectionX = (float[])world.DirectionX.Clone(),
        DirectionY = (float[])world.DirectionY.Clone(),
        AngularVelocity = (float[])world.AngularVelocity.Clone()
    };

    public YmirSoAWorld ToSoA() => new()
    {
        Time = Time,
        BodyIds = (string[])BodyIds.Clone(),
        PositionX = (float[])PositionX.Clone(),
        PositionY = (float[])PositionY.Clone(),
        VelocityX = (float[])VelocityX.Clone(),
        VelocityY = (float[])VelocityY.Clone(),
        Radius = (float[])Radius.Clone(),
        Mass = (float[])Mass.Clone(),
        DynamicMask = (float[])DynamicMask.Clone(),
        Restitution = (float[])Restitution.Clone(),
        FieldIds = (string[])FieldIds.Clone(),
        FieldPositionX = (float[])FieldPositionX.Clone(),
        FieldPositionY = (float[])FieldPositionY.Clone(),
        FieldStrength = (float[])FieldStrength.Clone(),
        FieldRadius = (float[])FieldRadius.Clone(),
        DirectionX = (float[])DirectionX.Clone(),
        DirectionY = (float[])DirectionY.Clone(),
        AngularVelocity = (float[])AngularVelocity.Clone()
    };
}

public static class YmirCultCacheStore
{
    public static async Task SaveWorldAsync(string path, string worldId, YmirSoAWorld world)
    {
        var cache = await CultCacheMessagePack.OpenAsync(
            path,
            new CultCacheOpenOptions
            {
                UseDirectoryStore = true,
                FlushOnDispose = true,
                StoreFlushOnDispose = true
            }).ConfigureAwait(false);

        using (cache)
        {
        await cache.UpsertAsync(
            YmirWorldStateDocumentV1.FromSoA(worldId, world),
            new CultRecordHandle<YmirWorldStateDocumentV1>(new CultRecordKey($"ymir:world:{worldId}")))
            .ConfigureAwait(false);
        await cache.FlushAsync().ConfigureAwait(false);
        }
    }

    public static async Task<YmirSoAWorld?> LoadWorldAsync(string path, string worldId)
    {
        using var cache = await CultCacheMessagePack.OpenAsync(
            path,
            new CultCacheOpenOptions { UseDirectoryStore = true }).ConfigureAwait(false);
        var key = new CultRecordKey($"ymir:world:{worldId}");
        return cache.Get<YmirWorldStateDocumentV1>(key)?.ToSoA()
            ?? cache.Get<YmirWorldStateDocumentV0>(key)?.ToSoA();
    }
}
