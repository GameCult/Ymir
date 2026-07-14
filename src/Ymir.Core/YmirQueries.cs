using Ymir.Box3D;

namespace Ymir.Core;

public static class YmirQueries
{
    public static CircleOverlapQueryResult OverlapCircle(CircleOverlapQueryRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        if (!float.IsFinite(request.Radius) || request.Radius <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "Overlap radius must be finite and positive.");
        }
        if (!float.IsFinite(request.Center.X) || !float.IsFinite(request.Center.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(request), "Overlap center must be finite.");
        }

        var hits = new List<CircleOverlapHit>();
        foreach (var body in request.Bodies)
        {
            ValidateQueryBody(body, request);
            var hit = Box3DQueries.OverlapSpheres(
                request.Center.X,
                request.Center.Y,
                request.Radius,
                body.Position.X,
                body.Position.Y,
                body.Radius);
            if (hit is not { } value)
            {
                continue;
            }

            hits.Add(new CircleOverlapHit(
                body.Id,
                request.Center,
                new Vec2(value.NormalX, value.NormalZ),
                value.Distance));
        }

        return new CircleOverlapQueryResult(hits
            .OrderBy(hit => hit.Distance)
            .ThenBy(hit => hit.BodyId, StringComparer.Ordinal)
            .ToArray());
    }

    public static CircleCastQueryResult CastCircle(CircleCastQueryRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        if (!float.IsFinite(request.Distance) || request.Distance < 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "Cast distance must be finite and non-negative.");
        }
        if (!float.IsFinite(request.Radius) || request.Radius <= 0.0f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "Cast radius must be finite and positive.");
        }
        if (!float.IsFinite(request.Origin.X) || !float.IsFinite(request.Origin.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(request), "Cast origin must be finite.");
        }

        var directionLengthSquared = request.Direction.X * request.Direction.X + request.Direction.Y * request.Direction.Y;
        if (!float.IsFinite(directionLengthSquared) || directionLengthSquared <= 1.0e-8f)
        {
            throw new ArgumentOutOfRangeException(nameof(request), "Cast direction must be finite and non-zero.");
        }

        var inverseLength = 1.0f / MathF.Sqrt(directionLengthSquared);
        var translationX = request.Direction.X * inverseLength * request.Distance;
        var translationY = request.Direction.Y * inverseLength * request.Distance;
        var hits = new List<CircleCastHit>();
        foreach (var body in request.Bodies)
        {
            ValidateQueryBody(body, request);

            var hit = Box3DQueries.CastSphere(
                request.Origin.X,
                request.Origin.Y,
                translationX,
                translationY,
                request.Radius,
                body.Position.X,
                body.Position.Y,
                body.Radius);
            if (hit is not { } value)
            {
                continue;
            }

            hits.Add(new CircleCastHit(
                body.Id,
                new Vec2(value.PointX, value.PointZ),
                new Vec2(value.NormalX, value.NormalZ),
                value.Fraction * request.Distance));
        }

        return new CircleCastQueryResult(hits
            .OrderBy(hit => hit.Distance)
            .ThenBy(hit => hit.BodyId, StringComparer.Ordinal)
            .ToArray());
    }

    private static void ValidateQueryBody(PhysicsBody body, object request)
    {
        if (string.IsNullOrWhiteSpace(body.Id) ||
            !float.IsFinite(body.Position.X) ||
            !float.IsFinite(body.Position.Y) ||
            !float.IsFinite(body.Radius) ||
            body.Radius <= 0.0f)
        {
            throw new ArgumentException("Query bodies require stable ids and finite positive radii.", nameof(request));
        }
    }
}
