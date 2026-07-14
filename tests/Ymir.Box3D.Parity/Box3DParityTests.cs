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

    [Fact]
    public void PairStepEmitsBeginContactAndSeparatesOverlappingSpheres()
    {
        var bodyA = new Box3DBodyInput(-0.9f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0);
        var bodyB = new Box3DBodyInput(0.9f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0);

        Box3DOracle.ymir_box3d_step_pair(in bodyA, in bodyB, 1.0f / 60.0f, 4, 0.0f, out var result);

        Assert.Equal(1, result.BeginContactCount);
        Assert.True(result.BodyA.PositionX < -0.9f);
        Assert.True(result.BodyB.PositionX > 0.9f);
    }

    [Fact]
    public void PairStepUsesMaximumRestitution()
    {
        var bodyA = new Box3DBodyInput(-1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.1f, 0);
        var bodyB = new Box3DBodyInput(1.0f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f, 0.8f, 0);

        Box3DOracle.ymir_box3d_step_pair(in bodyA, in bodyB, 1.0f / 60.0f, 4, 0.0f, out var result);

        Assert.True(result.BodyA.VelocityX < -0.7f);
        Assert.True(result.BodyB.VelocityX > 0.7f);
    }

    [Fact]
    public void TorqueIsClearedAfterEachWorldStep()
    {
        var result = Box3DOracle.ymir_box3d_torque_lifetime(0.4f, 0.1f, 4);

        Assert.True(result.AngularVelocityAfterAppliedStep > 0.0f);
        Assert.Equal(
            result.AngularVelocityAfterAppliedStep,
            result.AngularVelocityAfterUnforcedStep,
            precision: 5);
    }
}
