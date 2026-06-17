using CultMath;
using static CultMath.math;

namespace Ymir.Core;

public sealed class YmirSimulator
{
    private const float MinimumMass = 0.0001f;
    private const float MinimumDistance = 0.0001f;

    public SimulationStepResult Step(SimulationStepRequest request)
    {
        if (request.DeltaTime <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "DeltaTime must be positive.");
        }

        var bodies = request.World.Bodies
            .OrderBy(body => body.Id, StringComparer.Ordinal)
            .Select(body => Integrate(body, request.World.Fields, request.DeltaTime))
            .ToList();

        var contacts = new List<ContactEvent>();
        ResolveCircleContacts(bodies, contacts);

        return new SimulationStepResult(
            request.World with
            {
                Time = request.World.Time + request.DeltaTime,
                Bodies = bodies
            },
            contacts);
    }

    private static PhysicsBody Integrate(PhysicsBody body, IReadOnlyList<RadialField> fields, float deltaTime)
    {
        ValidateBody(body);

        if (body.IsStatic)
        {
            return body;
        }

        var position = body.Position.ToCultMath();
        var velocity = body.Velocity.ToCultMath();
        var acceleration = float2(0.0f);

        foreach (var field in fields.OrderBy(field => field.Id, StringComparer.Ordinal))
        {
            if (field.Radius <= 0.0f || field.Strength == 0.0f)
            {
                continue;
            }

            var offset = field.Position.ToCultMath() - position;
            var distance = max(length(offset), MinimumDistance);
            if (distance > field.Radius)
            {
                continue;
            }

            var direction = offset / distance;
            var falloff = saturate(1.0f - distance / field.Radius);
            acceleration += direction * field.Strength * falloff;
        }

        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;

        return body with
        {
            Position = Vec2.FromCultMath(position),
            Velocity = Vec2.FromCultMath(velocity)
        };
    }

    private static void ResolveCircleContacts(List<PhysicsBody> bodies, List<ContactEvent> contacts)
    {
        for (var i = 0; i < bodies.Count; i++)
        {
            for (var j = i + 1; j < bodies.Count; j++)
            {
                var left = bodies[i];
                var right = bodies[j];
                var delta = right.Position.ToCultMath() - left.Position.ToCultMath();
                var distance = length(delta);
                var combinedRadius = left.Radius + right.Radius;
                var penetration = combinedRadius - distance;

                if (penetration <= 0.0f)
                {
                    continue;
                }

                var normal = distance <= MinimumDistance ? float2(1.0f, 0.0f) : delta / distance;
                var point = left.Position.ToCultMath() + normal * (left.Radius - penetration * 0.5f);
                var relativeVelocity = right.Velocity.ToCultMath() - left.Velocity.ToCultMath();
                var relativeSpeed = dot(relativeVelocity, normal);

                contacts.Add(new ContactEvent(
                    left.Id,
                    right.Id,
                    Vec2.FromCultMath(point),
                    Vec2.FromCultMath(normal),
                    penetration,
                    relativeSpeed));

                ResolveImpulse(ref left, ref right, normal, penetration, relativeSpeed);
                bodies[i] = left;
                bodies[j] = right;
            }
        }
    }

    private static void ResolveImpulse(
        ref PhysicsBody left,
        ref PhysicsBody right,
        float2 normal,
        float penetration,
        float relativeSpeed)
    {
        var leftInvMass = left.IsStatic ? 0.0f : 1.0f / max(left.Mass, MinimumMass);
        var rightInvMass = right.IsStatic ? 0.0f : 1.0f / max(right.Mass, MinimumMass);
        var invMassSum = leftInvMass + rightInvMass;
        if (invMassSum <= 0.0f)
        {
            return;
        }

        var correction = normal * (penetration / invMassSum);
        if (!left.IsStatic)
        {
            left = left with { Position = Vec2.FromCultMath(left.Position.ToCultMath() - correction * leftInvMass) };
        }

        if (!right.IsStatic)
        {
            right = right with { Position = Vec2.FromCultMath(right.Position.ToCultMath() + correction * rightInvMass) };
        }

        if (relativeSpeed >= 0.0f)
        {
            return;
        }

        var restitution = min(left.Restitution, right.Restitution);
        var impulseMagnitude = -(1.0f + restitution) * relativeSpeed / invMassSum;
        var impulse = normal * impulseMagnitude;

        if (!left.IsStatic)
        {
            left = left with { Velocity = Vec2.FromCultMath(left.Velocity.ToCultMath() - impulse * leftInvMass) };
        }

        if (!right.IsStatic)
        {
            right = right with { Velocity = Vec2.FromCultMath(right.Velocity.ToCultMath() + impulse * rightInvMass) };
        }
    }

    private static void ValidateBody(PhysicsBody body)
    {
        if (string.IsNullOrWhiteSpace(body.Id))
        {
            throw new ArgumentException("Physics bodies must have stable ids.");
        }

        if (body.Radius <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(body), "Body radius must be positive.");
        }

        if (!body.IsStatic && body.Mass <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(body), "Dynamic body mass must be positive.");
        }
    }
}
