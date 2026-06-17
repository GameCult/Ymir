using CultMath;
using static CultMath.math;

namespace Ymir.Core;

public sealed class YmirSimulator
{
    private const float MinimumMass = 0.0001f;
    private const float MinimumDistance = 0.0001f;

    public SimulationStepResult Step(SimulationStepRequest request)
    {
        var result = Step(new SoASimulationStepRequest(
            request.DeltaTime,
            YmirSoAWorld.FromWorld(request.World)));

        return new SimulationStepResult(result.World.ToWorld(), result.Contacts);
    }

    public SoASimulationStepResult Step(SoASimulationStepRequest request)
    {
        if (request.DeltaTime <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "DeltaTime must be positive.");
        }

        var world = request.World.Clone();
        world.Validate();

        var accelerationX = new float[world.BodyCount];
        var accelerationY = new float[world.BodyCount];
        for (var i = 0; i < world.FieldCount; i++)
        {
            BatchMath.AddRadialFalloffAcceleration2D(
                world.PositionX,
                world.PositionY,
                world.FieldPositionX[i],
                world.FieldPositionY[i],
                world.FieldStrength[i],
                world.FieldRadius[i],
                accelerationX,
                accelerationY);
        }

        BatchMath.IntegrateSemiImplicitEuler2D(
            request.DeltaTime,
            world.DynamicMask,
            world.PositionX,
            world.PositionY,
            world.VelocityX,
            world.VelocityY,
            accelerationX,
            accelerationY);

        var contacts = new List<ContactEvent>();
        ResolveCircleContacts(world, contacts);

        world.Time += request.DeltaTime;
        return new SoASimulationStepResult(
            world,
            contacts);
    }

    private static void ResolveCircleContacts(YmirSoAWorld world, List<ContactEvent> contacts)
    {
        for (var i = 0; i < world.BodyCount; i++)
        {
            for (var j = i + 1; j < world.BodyCount; j++)
            {
                var delta = world.PositionAt(j) - world.PositionAt(i);
                var distance = length(delta);
                var combinedRadius = world.Radius[i] + world.Radius[j];
                var penetration = combinedRadius - distance;

                if (penetration <= 0.0f)
                {
                    continue;
                }

                var normal = distance <= MinimumDistance ? float2(1.0f, 0.0f) : delta / distance;
                var point = world.PositionAt(i) + normal * (world.Radius[i] - penetration * 0.5f);
                var relativeVelocity = world.VelocityAt(j) - world.VelocityAt(i);
                var relativeSpeed = dot(relativeVelocity, normal);

                contacts.Add(new ContactEvent(
                    world.BodyIds[i],
                    world.BodyIds[j],
                    Vec2.FromCultMath(point),
                    Vec2.FromCultMath(normal),
                    penetration,
                    relativeSpeed));

                ResolveImpulse(world, i, j, normal, penetration, relativeSpeed);
            }
        }
    }

    private static void ResolveImpulse(
        YmirSoAWorld world,
        int left,
        int right,
        float2 normal,
        float penetration,
        float relativeSpeed)
    {
        var leftInvMass = world.DynamicMask[left] <= 0.0f ? 0.0f : 1.0f / max(world.Mass[left], MinimumMass);
        var rightInvMass = world.DynamicMask[right] <= 0.0f ? 0.0f : 1.0f / max(world.Mass[right], MinimumMass);
        var invMassSum = leftInvMass + rightInvMass;
        if (invMassSum <= 0.0f)
        {
            return;
        }

        var correction = normal * (penetration / invMassSum);
        if (leftInvMass > 0.0f)
        {
            world.SetPosition(left, world.PositionAt(left) - correction * leftInvMass);
        }

        if (rightInvMass > 0.0f)
        {
            world.SetPosition(right, world.PositionAt(right) + correction * rightInvMass);
        }

        if (relativeSpeed >= 0.0f)
        {
            return;
        }

        var restitution = min(world.Restitution[left], world.Restitution[right]);
        var impulseMagnitude = -(1.0f + restitution) * relativeSpeed / invMassSum;
        var impulse = normal * impulseMagnitude;

        if (leftInvMass > 0.0f)
        {
            world.SetVelocity(left, world.VelocityAt(left) - impulse * leftInvMass);
        }

        if (rightInvMass > 0.0f)
        {
            world.SetVelocity(right, world.VelocityAt(right) + impulse * rightInvMass);
        }
    }
}
