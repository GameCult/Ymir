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
    public float[] Radius { get; set; } = [];
    public float[] Mass { get; set; } = [];
    public float[] DynamicMask { get; set; } = [];
    public float[] Restitution { get; set; } = [];
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
            Radius = bodies.Select(body => body.Radius).ToArray(),
            Mass = bodies.Select(body => body.Mass).ToArray(),
            DynamicMask = bodies.Select(body => body.IsStatic ? 0.0f : 1.0f).ToArray(),
            Restitution = bodies.Select(body => body.Restitution).ToArray(),
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
                Restitution[i]))
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
        foreach (var length in new[]
                 {
                     PositionX.Length,
                     PositionY.Length,
                     VelocityX.Length,
                     VelocityY.Length,
                     Radius.Length,
                     Mass.Length,
                     DynamicMask.Length,
                     Restitution.Length
                 })
        {
            if (length != expected)
            {
                throw new ArgumentException("Ymir body SoA arrays must have equal length.");
            }
        }
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
