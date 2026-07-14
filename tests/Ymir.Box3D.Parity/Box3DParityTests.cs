namespace Ymir.Box3D.Parity;

public sealed class Box3DParityTests
{
    [Fact]
    public void OracleIsPinnedBox3DRelease()
    {
        var doublePrecision = Box3DOracle.ymir_box3d_get_version(out var major, out var minor, out var revision);

        Assert.Equal((0, 1, 0), (major, minor, revision));
        Assert.Equal(0, doublePrecision);
    }

    [Theory]
    [InlineData(1.9f, true)]
    [InlineData(2.0f, true)]
    [InlineData(2.00049f, true)]
    [InlineData(2.00051f, false)]
    public void SphereOverlapClassificationMatchesBox3DSlop(float centerDistance, bool expected)
    {
        var result = Box3DOracle.ymir_box3d_overlap_spheres(0.0f, 0.0f, 1.0f, centerDistance, 0.0f, 1.0f);

        Assert.Equal(expected, result.IsHit);
    }

    [Theory]
    [InlineData(-1.00049f, true)]
    [InlineData(-1.00051f, false)]
    [InlineData(5.0f, true)]
    [InlineData(11.00049f, true)]
    [InlineData(11.00051f, false)]
    public void TractorCapsuleMembershipUsesBox3DRoundedCapsAndSlop(float pickupX, bool expected)
    {
        var result = Box3DOracle.ymir_box3d_overlap_capsule_sphere(
            0.0f,
            0.0f,
            10.0f,
            0.0f,
            0.5f,
            pickupX,
            0.0f,
            0.5f);

        Assert.Equal(expected, result.IsHit);
    }

    [Fact]
    public void SphereCastReturnsBox3DFractionAndNormal()
    {
        var result = Box3DOracle.ymir_box3d_cast_sphere(
            -5.0f,
            0.0f,
            10.0f,
            0.0f,
            0.5f,
            0.0f,
            0.0f,
            1.0f,
            0);

        Assert.True(result.IsHit);
        Assert.Equal(0.3505f, result.Fraction, precision: 4);
        Assert.Equal(-1.0f, result.NormalX, precision: 4);
        Assert.Equal(0.0f, result.NormalY, precision: 4);
    }

    [Theory]
    [InlineData(-1.5f)]
    [InlineData(-1.4f)]
    [InlineData(0.0f)]
    public void SphereCastInitiallyTouchingOrOverlappingIsAHit(float originX)
    {
        var result = Box3DOracle.ymir_box3d_cast_sphere(
            originX,
            0.0f,
            5.0f,
            0.0f,
            0.5f,
            0.0f,
            0.0f,
            1.0f,
            0);

        Assert.True(result.IsHit);
    }
}
