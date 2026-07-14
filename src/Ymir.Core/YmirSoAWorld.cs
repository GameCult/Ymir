using CultMath;

namespace Ymir.Core;

public sealed class YmirSoAWorld
{
    public float Time { get; set; }
    public string[] BodyIds { get; set; } = [];
    public float[] PositionX { get; set; } = [];
    public float[] PositionY { get; set; } = [];
    public float[] VelocityX { get; set; } = [];
    public float[] VelocityY { get; set; } = [];
    public float[] DirectionX { get; set; } = [];
    public float[] DirectionY { get; set; } = [];
    public float[] AngularVelocity { get; set; } = [];
    public float[] Torque { get; set; } = [];
    public float[] Radius { get; set; } = [];
    public float[] Mass { get; set; } = [];
    public float[] DynamicMask { get; set; } = [];
    public float[] Restitution { get; set; } = [];
    public float[] KinematicMask { get; set; } = [];
    public float[] BulletMask { get; set; } = [];
    public float[] FieldParticipationMask { get; set; } = [];
    public ulong[] CollisionCategoryBits { get; set; } = [];
    public ulong[] CollisionMaskBits { get; set; } = [];
    public int[] CollisionGroupIndex { get; set; } = [];
    public string[] FieldIds { get; set; } = [];
    public float[] FieldPositionX { get; set; } = [];
    public float[] FieldPositionY { get; set; } = [];
    public float[] FieldStrength { get; set; } = [];
    public float[] FieldRadius { get; set; } = [];

    public int BodyCount => BodyIds.Length;

    public int FieldCount => FieldIds.Length;

    public static YmirSoAWorld FromWorld(YmirWorld world)
    {
        var bodies = world.Bodies.OrderBy(body => body.Id, StringComparer.Ordinal).ToArray();
        var fields = world.Fields.OrderBy(field => field.Id, StringComparer.Ordinal).ToArray();

        return new YmirSoAWorld
        {
            Time = world.Time,
            BodyIds = bodies.Select(body => body.Id).ToArray(),
            PositionX = bodies.Select(body => body.Position.X).ToArray(),
            PositionY = bodies.Select(body => body.Position.Y).ToArray(),
            VelocityX = bodies.Select(body => body.Velocity.X).ToArray(),
            VelocityY = bodies.Select(body => body.Velocity.Y).ToArray(),
            DirectionX = bodies.Select(body => DirectionOrDefault(body.Direction).X).ToArray(),
            DirectionY = bodies.Select(body => DirectionOrDefault(body.Direction).Y).ToArray(),
            AngularVelocity = bodies.Select(body => body.AngularVelocity).ToArray(),
            Torque = bodies.Select(body => body.Torque).ToArray(),
            Radius = bodies.Select(body => body.Radius).ToArray(),
            Mass = bodies.Select(body => body.Mass).ToArray(),
            DynamicMask = bodies.Select(body => body.IsStatic ? 0.0f : 1.0f).ToArray(),
            Restitution = bodies.Select(body => body.Restitution).ToArray(),
            KinematicMask = bodies.Select(body => body.IsKinematic ? 1.0f : 0.0f).ToArray(),
            BulletMask = bodies.Select(body => body.IsBullet ? 1.0f : 0.0f).ToArray(),
            FieldParticipationMask = bodies.Select(body => body.ParticipatesInFields ? 1.0f : 0.0f).ToArray(),
            CollisionCategoryBits = bodies.Select(body => body.CollisionCategoryBits).ToArray(),
            CollisionMaskBits = bodies.Select(body => body.CollisionMaskBits).ToArray(),
            CollisionGroupIndex = bodies.Select(body => body.CollisionGroupIndex).ToArray(),
            FieldIds = fields.Select(field => field.Id).ToArray(),
            FieldPositionX = fields.Select(field => field.Position.X).ToArray(),
            FieldPositionY = fields.Select(field => field.Position.Y).ToArray(),
            FieldStrength = fields.Select(field => field.Strength).ToArray(),
            FieldRadius = fields.Select(field => field.Radius).ToArray()
        };
    }

    public YmirWorld ToWorld() => new(
        Time,
        Enumerable.Range(0, BodyCount)
            .Select(i => new PhysicsBody(
                BodyIds[i],
                new Vec2(PositionX[i], PositionY[i]),
                new Vec2(VelocityX[i], VelocityY[i]),
                Radius[i],
                Mass[i],
                DynamicMask[i] <= 0.0f,
                Restitution[i],
                new Vec2(DirectionX[i], DirectionY[i]),
                AngularVelocity[i],
                Torque[i],
                KinematicMask[i] > 0.0f,
                BulletMask[i] > 0.0f,
                FieldParticipationMask[i] > 0.0f,
                CollisionCategoryBits[i],
                CollisionMaskBits[i],
                CollisionGroupIndex[i]))
            .ToArray(),
        Enumerable.Range(0, FieldCount)
            .Select(i => new RadialField(
                FieldIds[i],
                new Vec2(FieldPositionX[i], FieldPositionY[i]),
                FieldStrength[i],
                FieldRadius[i]))
            .ToArray());

    public YmirSoAWorld Clone() => new()
    {
        Time = Time,
        BodyIds = (string[])BodyIds.Clone(),
        PositionX = (float[])PositionX.Clone(),
        PositionY = (float[])PositionY.Clone(),
        VelocityX = (float[])VelocityX.Clone(),
        VelocityY = (float[])VelocityY.Clone(),
        DirectionX = (float[])DirectionX.Clone(),
        DirectionY = (float[])DirectionY.Clone(),
        AngularVelocity = (float[])AngularVelocity.Clone(),
        Torque = (float[])Torque.Clone(),
        Radius = (float[])Radius.Clone(),
        Mass = (float[])Mass.Clone(),
        DynamicMask = (float[])DynamicMask.Clone(),
        Restitution = (float[])Restitution.Clone(),
        KinematicMask = (float[])KinematicMask.Clone(),
        BulletMask = (float[])BulletMask.Clone(),
        FieldParticipationMask = (float[])FieldParticipationMask.Clone(),
        CollisionCategoryBits = (ulong[])CollisionCategoryBits.Clone(),
        CollisionMaskBits = (ulong[])CollisionMaskBits.Clone(),
        CollisionGroupIndex = (int[])CollisionGroupIndex.Clone(),
        FieldIds = (string[])FieldIds.Clone(),
        FieldPositionX = (float[])FieldPositionX.Clone(),
        FieldPositionY = (float[])FieldPositionY.Clone(),
        FieldStrength = (float[])FieldStrength.Clone(),
        FieldRadius = (float[])FieldRadius.Clone()
    };

    public void Validate()
    {
        ValidateBodyLengths();
        ValidateFieldLengths();

        for (var i = 0; i < BodyCount; i++)
        {
            if (string.IsNullOrWhiteSpace(BodyIds[i]))
            {
                throw new ArgumentException("Ymir bodies must have stable ids.");
            }

            if (Radius[i] <= 0.0f)
            {
                throw new ArgumentOutOfRangeException(nameof(Radius), "Body radius must be positive.");
            }

            if (DynamicMask[i] > 0.0f && Mass[i] <= 0.0f)
            {
                throw new ArgumentOutOfRangeException(nameof(Mass), "Dynamic body mass must be positive.");
            }

            if (DynamicMask[i] <= 0.0f && KinematicMask[i] > 0.0f)
            {
                throw new ArgumentException("A body cannot be both static and kinematic.");
            }

            if (CollisionCategoryBits[i] == 0)
            {
                throw new ArgumentOutOfRangeException(nameof(CollisionCategoryBits), "Collision category bits must be non-zero.");
            }

            var directionLengthSquared = DirectionX[i] * DirectionX[i] + DirectionY[i] * DirectionY[i];
            if (!float.IsFinite(directionLengthSquared) || directionLengthSquared <= 1.0e-8f)
            {
                throw new ArgumentOutOfRangeException(nameof(DirectionX), "Body direction must be finite and non-zero.");
            }

            if (!float.IsFinite(AngularVelocity[i]) || !float.IsFinite(Torque[i]))
            {
                throw new ArgumentOutOfRangeException(nameof(AngularVelocity), "Body angular state must be finite.");
            }
        }
    }

    public float2 PositionAt(int index) => new(PositionX[index], PositionY[index]);

    public float2 VelocityAt(int index) => new(VelocityX[index], VelocityY[index]);

    public void SetPosition(int index, float2 value)
    {
        PositionX[index] = value.x;
        PositionY[index] = value.y;
    }

    public void SetVelocity(int index, float2 value)
    {
        VelocityX[index] = value.x;
        VelocityY[index] = value.y;
    }

    private void ValidateBodyLengths()
    {
        var expected = BodyIds.Length;
        FillMissingAngularState(expected);
        FillMissingCollisionState(expected);
        foreach (var length in new[]
                 {
                     PositionX.Length,
                     PositionY.Length,
                     VelocityX.Length,
                     VelocityY.Length,
                     DirectionX.Length,
                     DirectionY.Length,
                     AngularVelocity.Length,
                     Torque.Length,
                     Radius.Length,
                     Mass.Length,
                     DynamicMask.Length,
                     Restitution.Length,
                     KinematicMask.Length,
                     BulletMask.Length,
                     FieldParticipationMask.Length,
                     CollisionCategoryBits.Length,
                     CollisionMaskBits.Length,
                     CollisionGroupIndex.Length
                 })
        {
            if (length != expected)
            {
                throw new ArgumentException("Ymir body SoA arrays must have equal length.");
            }
        }
    }

    private void FillMissingCollisionState(int expected)
    {
        if (KinematicMask.Length == 0 && expected > 0) KinematicMask = new float[expected];
        if (BulletMask.Length == 0 && expected > 0) BulletMask = new float[expected];
        if (FieldParticipationMask.Length == 0 && expected > 0)
            FieldParticipationMask = Enumerable.Repeat(1.0f, expected).ToArray();
        if (CollisionCategoryBits.Length == 0 && expected > 0)
            CollisionCategoryBits = Enumerable.Repeat(1UL, expected).ToArray();
        if (CollisionMaskBits.Length == 0 && expected > 0)
            CollisionMaskBits = Enumerable.Repeat(ulong.MaxValue, expected).ToArray();
        if (CollisionGroupIndex.Length == 0 && expected > 0) CollisionGroupIndex = new int[expected];
    }

    private void FillMissingAngularState(int expected)
    {
        if (DirectionX.Length == 0 && expected > 0)
        {
            DirectionX = new float[expected];
        }

        if (DirectionY.Length == 0 && expected > 0)
        {
            DirectionY = Enumerable.Repeat(1.0f, expected).ToArray();
        }

        if (AngularVelocity.Length == 0 && expected > 0)
        {
            AngularVelocity = new float[expected];
        }

        if (Torque.Length == 0 && expected > 0)
        {
            Torque = new float[expected];
        }
    }

    private static Vec2 DirectionOrDefault(Vec2? direction)
    {
        if (direction is not { } value ||
            value.X * value.X + value.Y * value.Y <= 1.0e-8f)
        {
            return new Vec2(0.0f, 1.0f);
        }

        return value;
    }

    private void ValidateFieldLengths()
    {
        var expected = FieldIds.Length;
        foreach (var length in new[]
                 {
                     FieldPositionX.Length,
                     FieldPositionY.Length,
                     FieldStrength.Length,
                     FieldRadius.Length
                 })
        {
            if (length != expected)
            {
                throw new ArgumentException("Ymir field SoA arrays must have equal length.");
            }
        }
    }
}

public sealed record SoASimulationStepRequest(float DeltaTime, YmirSoAWorld World);

public sealed record SoASimulationStepResult(YmirSoAWorld World, IReadOnlyList<ContactEvent> Contacts);
