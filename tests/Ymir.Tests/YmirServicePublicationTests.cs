using GameCult.Caching;
using GameCult.Caching.MessagePack;
using GameCult.Eve.Surface;
using Ymir.Core;

namespace Ymir.Tests;

public sealed class YmirServicePublicationTests
{
    [Fact]
    public void PublicationAdvertisesBox3DOwnershipBoundary()
    {
        var now = DateTimeOffset.Parse("2026-07-14T12:00:00Z");

        var advertisement = YmirServicePublication.ProviderAdvertisement(now);
        var diagnostics = YmirServicePublication.OperatorState(now);

        Assert.Contains("Box3D", advertisement.Description, StringComparison.Ordinal);
        Assert.Contains("Box3D-only isolated snapshot stepping", advertisement.Owns);
        Assert.Contains("public retained session lifecycle yet", advertisement.DoesNotOwn);
        Assert.Empty(advertisement.CommandLowerings);
        Assert.Contains("Box3D solver and collision semantics", advertisement.DoesNotOwn);
        Assert.Equal("Box3D v0.1.0 (C17)", diagnostics.NumericSubstrate);
        Assert.Contains("does not own physics algorithms", diagnostics.BatchKernel, StringComparison.Ordinal);
        Assert.Contains("world_state.v1", diagnostics.Persistence, StringComparison.Ordinal);
        Assert.Contains("v0 read migration", diagnostics.Persistence, StringComparison.Ordinal);
    }

    [Fact]
    public void EveSurfaceUsesCanonicalDocumentAndOperatorMetrics()
    {
        var diagnostics = new YmirOperatorStateDocument
        {
            ProviderId = "ymir.physics",
            Status = "ready",
            StateOwner = "diagnostic-owner",
            NumericSubstrate = "diagnostic-math",
            BatchKernel = "diagnostic-batch",
            Persistence = "diagnostic-persistence",
            UpdatedAtUtc = "2026-07-13T12:00:00.0000000Z"
        };

        EveSurfaceDocument surface = YmirServicePublication.EveSurface(diagnostics);

        Assert.Equal(EveSurfaceDocument.SchemaId, surface.Schema);
        Assert.Equal("ymir.physics.operator", surface.Surface.Id);
        Assert.Equal("partition", surface.Surface.Root.Kind);
        Assert.Equal(diagnostics.UpdatedAtUtc, surface.UpdatedAtUtc);
        Assert.Equal(
            new[]
            {
                ("Status", diagnostics.Status),
                ("State owner", diagnostics.StateOwner),
                ("Numeric substrate", diagnostics.NumericSubstrate),
                ("Batch kernel", diagnostics.BatchKernel),
                ("Persistence", diagnostics.Persistence)
            },
            surface.Surface.Root.Children
                .Select(child => (child.GetProp("label"), child.GetProp("value")))
                .ToArray());
    }

    [Fact]
    public void CombinedRegistryContainsExactlyOneCanonicalSurfaceDescriptor()
    {
        var registry = new CultDocumentRegistry();

        var descriptors = registry.AllDescriptors
            .Where(descriptor => descriptor.SchemaVersion == EveSurfaceDocument.SchemaId)
            .ToArray();

        var descriptor = Assert.Single(descriptors);
        Assert.Equal(typeof(EveSurfaceDocument), descriptor.DocumentType);
    }

    [Fact]
    public async Task PublicationPersistsAndReopensWithCombinedRegistry()
    {
        var path = TemporaryStorePath();
        try
        {
            await YmirServicePublicationStore.RegenerateDerivedStoreAsync(path, DateTimeOffset.UtcNow);

            using var cache = await CultCacheMessagePack.OpenAsync(path, new CultCacheOpenOptions
            {
                Registry = new CultDocumentRegistry(),
                UseDirectoryStore = true
            });

            var surface = cache.Get<EveSurfaceDocument>(new CultRecordKey(YmirServicePublicationStore.SurfaceRecordKey));
            Assert.NotNull(surface);
            Assert.Equal("ymir.physics.operator", surface.Surface.Id);
            Assert.Single(cache.GetAll<EveSurfaceDocument>());
        }
        finally
        {
            DeleteStore(path);
        }
    }

    [Fact]
    public async Task RegenerationReplacesUnreadableLegacyDerivedStoreBeforeOpen()
    {
        var path = TemporaryStorePath();
        var recordsPath = DirectoryMessagePackBackingStore.DefaultRecordDirectoryPath(path);
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            Directory.CreateDirectory(recordsPath);
            await File.WriteAllBytesAsync(path, [0xc1]);
            await File.WriteAllBytesAsync(Path.Combine(recordsPath, "legacy.msgpack"), [0xc1]);

            await YmirServicePublicationStore.RegenerateDerivedStoreAsync(path, DateTimeOffset.UtcNow);

            using var cache = await CultCacheMessagePack.OpenAsync(path, new CultCacheOpenOptions
            {
                Registry = new CultDocumentRegistry(),
                UseDirectoryStore = true
            });
            Assert.NotNull(cache.Get<EveSurfaceDocument>(new CultRecordKey(YmirServicePublicationStore.SurfaceRecordKey)));
            Assert.False(File.Exists(Path.Combine(recordsPath, "legacy.msgpack")));
        }
        finally
        {
            DeleteStore(path);
        }
    }

    private static string TemporaryStorePath() =>
        Path.Combine(Path.GetTempPath(), "ymir-tests", Guid.NewGuid().ToString("N"), "service.cc");

    private static void DeleteStore(string path)
    {
        var directory = Path.GetDirectoryName(path);
        if (directory is not null && Directory.Exists(directory))
            Directory.Delete(directory, recursive: true);
    }
}
