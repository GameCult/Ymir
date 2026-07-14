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
    float Restitution = 0.2f,
    Vec2? Direction = null,
    float AngularVelocity = 0.0f,
    float Torque = 0.0f,
    bool IsKinematic = false,
    bool IsBullet = false,
    bool ParticipatesInFields = true,
    ulong CollisionCategoryBits = 1UL,
    ulong CollisionMaskBits = ulong.MaxValue,
    int CollisionGroupIndex = 0);

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

public sealed record CircleCastQueryRequest(
    Vec2 Origin,
    Vec2 Direction,
    float Distance,
    float Radius,
    IReadOnlyList<PhysicsBody> Bodies);

public sealed record CircleCastHit(
    string BodyId,
    Vec2 Point,
    Vec2 Normal,
    float Distance);

public sealed record CircleCastQueryResult(IReadOnlyList<CircleCastHit> Hits);

public sealed record CircleOverlapQueryRequest(
    Vec2 Center,
    float Radius,
    IReadOnlyList<PhysicsBody> Bodies);

public sealed record CircleOverlapHit(
    string BodyId,
    Vec2 Point,
    Vec2 Normal,
    float Distance);

public sealed record CircleOverlapQueryResult(IReadOnlyList<CircleOverlapHit> Hits);
