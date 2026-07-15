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

    [Fact]
    public unsafe void RetainedSessionCommandsPreserveTypedContactLifecycle()
    {
        Assert.Equal(5u, Box3DOracle.ymir_box3d_get_abi_version());
        Assert.Equal(0, Box3DOracle.ymir_box3d_session_create(out var session));
        try
        {
            var bodyA = SessionBody(1, -0.9f);
            var bodyB = SessionBody(2, 0.9f);
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_spawn(session, &bodyA));
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_spawn(session, &bodyB));
            Assert.Equal(2, Box3DOracle.ymir_box3d_session_spawn(session, &bodyA));
            Assert.Equal(1, Box3DOracle.ymir_box3d_session_remove(session, 999));

            Assert.Equal(0, Box3DOracle.ymir_box3d_session_step(session, 1.0f / 60.0f, 4, null, 0));
            var beginEvents = CopyEvents(session);
            var begin = Assert.Single(beginEvents, value => value.Kind == 1);
            Assert.Equal(new HashSet<ulong> { 1, 2 }, new HashSet<ulong> { begin.StableIdA, begin.StableIdB });
            Assert.Equal(1u, begin.HasDetails);

            Assert.Equal(0, Box3DOracle.ymir_box3d_session_remove(session, 2));
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_step(session, 1.0f / 60.0f, 4, null, 0));
            var end = Assert.Single(CopyEvents(session), value => value.Kind == 3);
            Assert.Equal(new HashSet<ulong> { 1, 2 }, new HashSet<ulong> { end.StableIdA, end.StableIdB });
            Assert.Equal(0u, end.HasDetails);
        }
        finally
        {
            Box3DOracle.ymir_box3d_session_destroy(session);
        }
    }

    [Fact]
    public unsafe void NegativeGroupAndCategoryMasksAreBox3DCollisionAuthority()
    {
        Assert.Equal(0, Box3DOracle.ymir_box3d_session_create(out var session));
        try
        {
            var source = SessionBody(1, 0.0f, category: 1, mask: 2, group: -7);
            var ownPayload = SessionBody(2, 0.0f, category: 2, mask: 1, group: -7);
            var otherPayload = SessionBody(3, 0.0f, category: 2, mask: 1, group: -8);
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_spawn(session, &source));
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_spawn(session, &ownPayload));
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_spawn(session, &otherPayload));

            Assert.Equal(0, Box3DOracle.ymir_box3d_session_step(session, 1.0f / 60.0f, 4, null, 0));
            var pairs = CopyEvents(session)
                .Where(value => value.Kind == 1)
                .Select(value => new HashSet<ulong> { value.StableIdA, value.StableIdB })
                .ToArray();

            Assert.Single(pairs);
            Assert.Equal(new HashSet<ulong> { 1, 3 }, pairs[0]);
        }
        finally
        {
            Box3DOracle.ymir_box3d_session_destroy(session);
        }
    }

    [Fact]
    public unsafe void BulletUsesBox3DContinuousCollisionAgainstKinematicTarget()
    {
        Assert.Equal(0, Box3DOracle.ymir_box3d_session_create(out var session));
        try
        {
            var bullet = SessionBody(1, -10.0f, velocityX: 100.0f, isBullet: true);
            var target = SessionBody(2, 0.0f, isKinematic: true);
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_spawn(session, &bullet));
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_spawn(session, &target));

            Assert.Equal(0, Box3DOracle.ymir_box3d_session_step(session, 0.2f, 4, null, 0));
            Assert.Empty(CopyEvents(session));
            var ccdBodies = CopyBodies(session);
            Assert.True(ccdBodies.Single(value => value.StableId == 1).PositionX < 0.0f);
            Assert.Equal(0.0f, ccdBodies.Single(value => value.StableId == 2).PositionX, precision: 5);
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_step(session, 1.0f / 60.0f, 4, null, 0));
            Assert.Contains(CopyEvents(session), value =>
                (value.Kind == 1 || value.Kind == 2) &&
                new HashSet<ulong> { value.StableIdA, value.StableIdB }.SetEquals([1UL, 2UL]));
        }
        finally
        {
            Box3DOracle.ymir_box3d_session_destroy(session);
        }
    }

    private static Box3DSessionBodyInput SessionBody(
        ulong id,
        float positionX,
        float velocityX = 0.0f,
        ulong category = 1,
        ulong mask = ulong.MaxValue,
        int group = 0,
        bool isKinematic = false,
        bool isBullet = false) => new(
            id, category, mask, positionX, 0.0f, velocityX, 0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0,
            isKinematic ? 1u : 0u, isBullet ? 1u : 0u, 1u, group);

    private static unsafe Box3DSessionContactEvent[] CopyEvents(nint session)
    {
        var count = Box3DOracle.ymir_box3d_session_get_contact_event_count(session);
        Assert.True(count >= 0);
        var events = new Box3DSessionContactEvent[count];
        fixed (Box3DSessionContactEvent* pointer = events)
        {
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_copy_contact_events(session, pointer, count, out var written));
            Assert.Equal(count, written);
        }
        return events;
    }

    private static unsafe Box3DSessionBodyOutput[] CopyBodies(nint session)
    {
        var count = Box3DOracle.ymir_box3d_session_get_body_count(session);
        Assert.True(count >= 0);
        var bodies = new Box3DSessionBodyOutput[count];
        fixed (Box3DSessionBodyOutput* pointer = bodies)
        {
            Assert.Equal(0, Box3DOracle.ymir_box3d_session_copy_bodies(session, pointer, count, out var written));
            Assert.Equal(count, written);
        }
        return bodies;
    }
}
