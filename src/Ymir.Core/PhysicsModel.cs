using CultMath;

namespace Ymir.Core;

public readonly record struct Vec2(float X, float Y)
{
    public static Vec2 Zero => new(0.0f, 0.0f);

    public float2 ToCultMath() => new(X, Y);

    public static Vec2 FromCultMath(float2 value) => new(value.x, value.y);
}

public sealed record PhysicsBody(
    string Id,
    Vec2 Position,
    Vec2 Velocity,
    float Radius,
    float Mass,
    bool IsStatic = false,
    float Restitution = 0.2f);

public sealed record RadialField(
    string Id,
    Vec2 Position,
    float Strength,
    float Radius);

public sealed record YmirWorld(
    float Time,
    IReadOnlyList<PhysicsBody> Bodies,
    IReadOnlyList<RadialField> Fields);

public sealed record SimulationStepRequest(
    float DeltaTime,
    YmirWorld World);

public sealed record ContactEvent(
    string BodyA,
    string BodyB,
    Vec2 Point,
    Vec2 Normal,
    float Penetration,
    float RelativeSpeed);

public sealed record SimulationStepResult(
    YmirWorld World,
    IReadOnlyList<ContactEvent> Contacts);
