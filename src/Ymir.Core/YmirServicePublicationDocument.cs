using GameCult.Caching;
using GameCult.Caching.MessagePack;
using GameCult.Eve.Surface;
using MessagePack;

namespace Ymir.Core;

[CultDocument("gamecult.ymir.provider_advertisement", "gamecult.ymir.provider_advertisement.v1")]
[MessagePackObject]
public sealed class YmirProviderAdvertisementDocument
{
    [Key(0)]
    [CultName]
    public string ProviderId { get; set; } = "ymir.physics";

    [Key(1)]
    public string Title { get; set; } = "Ymir Physics";

    [Key(2)]
    public string Description { get; set; } = "GameCult Box3D retained-session library and physics-daemon cutover surface.";

    [Key(3)]
    public string[] Owns { get; set; } = [];

    [Key(4)]
    public string[] DoesNotOwn { get; set; } = [];

    [Key(5)]
    public YmirTransportLoweringDocument[] CommandLowerings { get; set; } = [];

    [Key(6)]
    public string UpdatedAtUtc { get; set; } = "";
}

[CultDocument("gamecult.ymir.operator_state", "gamecult.ymir.operator_state.v1")]
[MessagePackObject]
public sealed class YmirOperatorStateDocument
{
    [Key(0)]
    [CultName]
    public string ProviderId { get; set; } = "ymir.physics";

    [Key(1)]
    public string Status { get; set; } = "mvp";

    [Key(2)]
    public string StateOwner { get; set; } = "In-process retained Box3D sessions, isolated snapshot steps, and diagnostic projections";

    [Key(3)]
    public string NumericSubstrate { get; set; } = "Box3D v0.1.0 (C17)";

    [Key(4)]
    public string BatchKernel { get; set; } = "";

    [Key(5)]
    public string Persistence { get; set; } = "CultCache gamecult.ymir.world_state.v1; explicit v0 read migration";

    [Key(6)]
    public string UpdatedAtUtc { get; set; } = "";
}

[MessagePackObject]
public sealed class YmirTransportLoweringDocument
{
    [Key(0)]
    public string Label { get; set; } = "";

    [Key(1)]
    public string Method { get; set; } = "POST";

    [Key(2)]
    public string Path { get; set; } = "";

    [Key(3)]
    public string Purpose { get; set; } = "";
}

public static class YmirServicePublication
{
    public static YmirProviderAdvertisementDocument ProviderAdvertisement(DateTimeOffset now) => new()
    {
        ProviderId = "ymir.physics",
        Title = "Ymir Physics",
        Description = "GameCult Box3D retained-session library and physics-daemon cutover surface.",
        Owns =
        [
            "Box3D-only isolated snapshot stepping",
            "public in-process retained sessions with explicit mutation receipts",
            "typed Begin, Hit, and End contact facts for retained steps",
            "stable GameCult id to transient Box3D handle projection",
            "Box3D-backed circle overlap and cast queries over submitted bodies",
            "deterministic result ordering",
            "CultCache world snapshot v1 with explicit v0 read migration",
            "diagnostic Eve publication"
        ],
        DoesNotOwn =
        [
            "daemon-owned named session registry or CultMesh command lowering yet",
            "complete checkpoint reconstruction yet",
            "physics algorithm invention",
            "Box3D solver and collision semantics",
            "Unity scene truth",
            "rendering",
            "gameplay damage policy",
            "editor authoring truth"
        ],
        CommandLowerings = [],
        UpdatedAtUtc = FormatUtc(now)
    };

    public static YmirOperatorStateDocument OperatorState(DateTimeOffset now) => new()
    {
        ProviderId = "ymir.physics",
        Status = "mvp",
        StateOwner = "In-process retained Box3D sessions, isolated snapshot steps, and diagnostic projections",
        NumericSubstrate = "Box3D v0.1.0 (C17)",
        BatchKernel = "Box3D native solver; Ymir does not own physics algorithms",
        Persistence = "CultCache gamecult.ymir.world_state.v1; explicit v0 read migration",
        UpdatedAtUtc = FormatUtc(now)
    };

    public static EveSurfaceDocument EveSurface(YmirOperatorStateDocument diagnostics) =>
        GameCult.Eve.Surface.EveSurface.Create("ymir.physics.operator")
            .Provider(diagnostics.ProviderId, "physics.operator")
            .Title("Ymir Physics")
            .UpdatedAtUtc(diagnostics.UpdatedAtUtc)
            .RootColumn("ymir.physics.operator.root", root => root
                .Metric("ymir.physics.operator.status", "Status", diagnostics.Status)
                .Metric("ymir.physics.operator.owner", "State owner", diagnostics.StateOwner)
                .Metric("ymir.physics.operator.math", "Numeric substrate", diagnostics.NumericSubstrate)
                .Metric("ymir.physics.operator.batch", "Batch kernel", diagnostics.BatchKernel)
                .Metric("ymir.physics.operator.persistence", "Persistence", diagnostics.Persistence))
            .Build();

    public static EveSurfaceDocument EveSurface(DateTimeOffset now)
    {
        return EveSurface(OperatorState(now));
    }

    private static string FormatUtc(DateTimeOffset value) => value.UtcDateTime.ToString("O");
}

public static class YmirServicePublicationStore
{
    public const string SurfaceRecordKey = "surface:ymir.physics.operator";

    public static async Task RegenerateDerivedStoreAsync(string path, DateTimeOffset now)
    {
        DeleteDerivedStore(path);
        await PublishAsync(path, now).ConfigureAwait(false);
    }

    public static async Task PublishAsync(string path, DateTimeOffset now)
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
            var operatorState = YmirServicePublication.OperatorState(now);
            await cache.UpsertAsync(
                YmirServicePublication.ProviderAdvertisement(now),
                new CultRecordHandle<YmirProviderAdvertisementDocument>(new CultRecordKey("ymir:provider:ymir.physics")))
                .ConfigureAwait(false);
            await cache.UpsertAsync(
                operatorState,
                new CultRecordHandle<YmirOperatorStateDocument>(new CultRecordKey("ymir:operator:ymir.physics")))
                .ConfigureAwait(false);
            await cache.UpsertAsync(
                YmirServicePublication.EveSurface(operatorState),
                new CultRecordHandle<EveSurfaceDocument>(new CultRecordKey(SurfaceRecordKey)))
                .ConfigureAwait(false);
            await cache.FlushAsync().ConfigureAwait(false);
        }
    }

    private static void DeleteDerivedStore(string path)
    {
        if (File.Exists(path))
            File.Delete(path);

        var recordsPath = DirectoryMessagePackBackingStore.DefaultRecordDirectoryPath(path);
        if (Directory.Exists(recordsPath))
            Directory.Delete(recordsPath, recursive: true);
    }
}
