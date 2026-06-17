using Ymir.Core;

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

    private static SimulationStepResult Step(params PhysicsBody[] bodies)
    {
        var simulator = new YmirSimulator();
        return simulator.Step(new SimulationStepRequest(
            0.1f,
            new YmirWorld(0.0f, bodies, Array.Empty<RadialField>())));
    }
}
