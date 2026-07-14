using Ymir.Core;
using GameCult.Caching;
using GameCult.Caching.MessagePack;

namespace Ymir.Tests;

public sealed class YmirSimulatorTests
{
    [Fact]
    public void StepIntegratesDynamicBodies()
    {
        var result = Step(new PhysicsBody("projectile", Vec2.Zero, new Vec2(10.0f, 0.0f), 0.1f, 1.0f));

        Assert.Equal(1.0f, result.World.Bodies[0].Position.X, precision: 5);
        Assert.Equal(0.0f, result.World.Bodies[0].Position.Y, precision: 5);
    }

    [Fact]
    public void RadialFieldAcceleratesBodyTowardWell()
    {
        var simulator = new YmirSimulator();
        var request = new SimulationStepRequest(
            1.0f,
            new YmirWorld(
                0.0f,
                new[] { new PhysicsBody("ship", Vec2.Zero, Vec2.Zero, 0.5f, 1.0f) },
                new[] { new RadialField("well", new Vec2(10.0f, 0.0f), 10.0f, 20.0f) }));

        var result = simulator.Step(request);

        Assert.True(result.World.Bodies[0].Velocity.X > 0.0f);
    }

    [Fact]
    public void OverlappingCirclesEmitContactAndSeparate()
    {
        var simulator = new YmirSimulator();
        var request = new SimulationStepRequest(
            0.1f,
            new YmirWorld(
                0.0f,
                new[]
                {
                    new PhysicsBody("a", new Vec2(0.0f, 0.0f), new Vec2(1.0f, 0.0f), 1.0f, 1.0f),
                    new PhysicsBody("b", new Vec2(1.5f, 0.0f), new Vec2(-1.0f, 0.0f), 1.0f, 1.0f)
                },
                Array.Empty<RadialField>()));

        var result = simulator.Step(request);

        var contact = Assert.Single(result.Contacts);
        Assert.Equal("a", contact.BodyA);
        Assert.Equal("b", contact.BodyB);
        Assert.True(contact.Penetration > 0.0f);
        Assert.True(result.World.Bodies[0].Position.X < 0.1f);
        Assert.True(result.World.Bodies[1].Position.X > 1.4f);
    }

    [Fact]
    public void ContactOrderingIsStableByBodyId()
    {
        var simulator = new YmirSimulator();
        var request = new SimulationStepRequest(
            0.1f,
            new YmirWorld(
                0.0f,
                new[]
                {
                    new PhysicsBody("zeta", new Vec2(0.0f, 0.0f), Vec2.Zero, 1.0f, 1.0f),
                    new PhysicsBody("alpha", new Vec2(1.0f, 0.0f), Vec2.Zero, 1.0f, 1.0f)
                },
                Array.Empty<RadialField>()));

        var contact = Assert.Single(simulator.Step(request).Contacts);

        Assert.Equal("alpha", contact.BodyA);
        Assert.Equal("zeta", contact.BodyB);
    }

    [Fact]
    public void ExplicitWorldSessionRetainsBox3DStateAcrossSteps()
    {
        using var session = new YmirWorldSession();
        var first = session.Step(new SimulationStepRequest(
            0.1f,
            new YmirWorld(
                0.0f,
                [new PhysicsBody("ship", Vec2.Zero, new Vec2(10.0f, 0.0f), 0.5f, 1.0f)],
                [])));

        var second = session.Step(new SimulationStepRequest(0.1f, first.World));

        Assert.Equal(2.0f, second.World.Bodies[0].Position.X, precision: 5);
        Assert.Equal(0.2f, second.World.Time, precision: 5);
    }

    [Fact]
    public void Box3DOwnsPlanarOrientationAndConsumesTorqueOnce()
    {
        using var session = new YmirWorldSession();
        var first = session.Step(new SimulationStepRequest(
            0.1f,
            new YmirWorld(
                0.0f,
                [new PhysicsBody(
                    "ship",
                    Vec2.Zero,
                    Vec2.Zero,
                    0.5f,
                    1.0f,
                    Direction: new Vec2(0.0f, 1.0f),
                    Torque: 1.0f)],
                [])));

        var firstBody = first.World.Bodies[0];
        Assert.NotNull(firstBody.Direction);
        Assert.NotEqual(0.0f, firstBody.AngularVelocity);
        Assert.Equal(0.0f, firstBody.Torque);

        var second = session.Step(new SimulationStepRequest(0.1f, first.World));
        Assert.Equal(firstBody.AngularVelocity, second.World.Bodies[0].AngularVelocity, precision: 4);
        Assert.NotEqual(firstBody.Direction, second.World.Bodies[0].Direction);
    }

    [Fact]
    public void CircleCastUsesPinnedBox3DEdgeSemantics()
    {
        var result = YmirQueries.CastCircle(new CircleCastQueryRequest(
            Vec2.Zero,
            new Vec2(1.0f, 0.0f),
            10.0f,
            1.0f,
            [new PhysicsBody("target", new Vec2(30.0f, 0.0f), Vec2.Zero, 20.0f, 1.0f, IsStatic: true)]));

        var hit = Assert.Single(result.Hits);
        Assert.Equal("target", hit.BodyId);
        Assert.InRange(hit.Distance, 8.99f, 9.01f);
    }

    [Fact]
    public void CircleOverlapUsesPinnedBox3DTangentSemantics()
    {
        var result = YmirQueries.OverlapCircle(new CircleOverlapQueryRequest(
            Vec2.Zero,
            1.0f,
            [new PhysicsBody("target", new Vec2(2.0f, 0.0f), Vec2.Zero, 1.0f, 1.0f, IsStatic: true)]));

        Assert.Equal("target", Assert.Single(result.Hits).BodyId);
    }

    [Fact]
    public void SoAStepProjectsBox3DStateIntoContractBuffers()
    {
        var simulator = new YmirSimulator();
        var world = new YmirSoAWorld
        {
            BodyIds = ["ship", "anchor"],
            PositionX = [0.0f, 10.0f],
            PositionY = [0.0f, 0.0f],
            VelocityX = [1.0f, 0.0f],
            VelocityY = [0.0f, 0.0f],
            Radius = [0.5f, 0.5f],
            Mass = [1.0f, 1.0f],
            DynamicMask = [1.0f, 0.0f],
            Restitution = [0.2f, 0.2f],
            FieldIds = ["well"],
            FieldPositionX = [10.0f],
            FieldPositionY = [0.0f],
            FieldStrength = [10.0f],
            FieldRadius = [20.0f]
        };

        var result = simulator.Step(new SoASimulationStepRequest(1.0f, world));

        Assert.True(result.World.PositionX[0] > 1.0f);
        Assert.Equal(10.0f, result.World.PositionX[1], precision: 5);
        Assert.Equal(1.0f, result.World.Time, precision: 5);
    }

    [Fact]
    public async Task CultCacheStoreRoundTripsSoAWorldState()
    {
        var path = Path.Combine(Path.GetTempPath(), $"ymir-tests-{Guid.NewGuid():N}.cc");
        var recordsPath = $"{path}.records";

        try
        {
            var world = YmirSoAWorld.FromWorld(new YmirWorld(
                2.0f,
                new[] { new PhysicsBody("ship", new Vec2(1.0f, 2.0f), new Vec2(3.0f, 4.0f), 0.5f, 1.0f, Direction: new Vec2(1.0f, 0.0f), AngularVelocity: 2.0f) },
                new[] { new RadialField("well", new Vec2(5.0f, 6.0f), 7.0f, 8.0f) }));

            await YmirCultCacheStore.SaveWorldAsync(path, "test", world);
            var loaded = await YmirCultCacheStore.LoadWorldAsync(path, "test");

            Assert.NotNull(loaded);
            Assert.Equal(["ship"], loaded!.BodyIds);
            Assert.Equal(1.0f, loaded.PositionX[0], precision: 5);
            Assert.Equal(7.0f, loaded.FieldStrength[0], precision: 5);
            Assert.Equal(1.0f, loaded.DirectionX[0], precision: 5);
            Assert.Equal(2.0f, loaded.AngularVelocity[0], precision: 5);
        }
        finally
        {
            if (File.Exists(path))
            {
                File.Delete(path);
            }

            if (Directory.Exists(recordsPath))
            {
                Directory.Delete(recordsPath, recursive: true);
            }
        }
    }

    [Fact]
    public async Task CultCacheStoreReadsV0WithDefaultAngularState()
    {
        var path = Path.Combine(Path.GetTempPath(), $"ymir-v0-tests-{Guid.NewGuid():N}.cc");
        var recordsPath = $"{path}.records";
        try
        {
            var world = YmirSoAWorld.FromWorld(new YmirWorld(
                2.0f,
                [new PhysicsBody("ship", Vec2.Zero, Vec2.Zero, 0.5f, 1.0f)],
                []));
            using (var cache = await CultCacheMessagePack.OpenAsync(
                       path,
                       new CultCacheOpenOptions { UseDirectoryStore = true, FlushOnDispose = true, StoreFlushOnDispose = true }))
            {
                await cache.UpsertAsync(
                    YmirWorldStateDocumentV0.FromSoA("test", world),
                    new CultRecordHandle<YmirWorldStateDocumentV0>(new CultRecordKey("ymir:world:test")));
                await cache.FlushAsync();
            }

            var loaded = await YmirCultCacheStore.LoadWorldAsync(path, "test");

            Assert.NotNull(loaded);
            loaded!.Validate();
            Assert.Equal(0.0f, loaded.DirectionX[0]);
            Assert.Equal(1.0f, loaded.DirectionY[0]);
            Assert.Equal(0.0f, loaded.AngularVelocity[0]);
        }
        finally
        {
            if (File.Exists(path)) File.Delete(path);
            if (Directory.Exists(recordsPath)) Directory.Delete(recordsPath, recursive: true);
        }
    }

    private static SimulationStepResult Step(params PhysicsBody[] bodies)
    {
        var simulator = new YmirSimulator();
        return simulator.Step(new SimulationStepRequest(
            0.1f,
            new YmirWorld(0.0f, bodies, Array.Empty<RadialField>())));
    }
}
