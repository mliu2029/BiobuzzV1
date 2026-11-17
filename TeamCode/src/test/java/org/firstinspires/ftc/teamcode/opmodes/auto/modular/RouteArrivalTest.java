package org.firstinspires.ftc.teamcode.opmodes.auto.modular;

import static org.junit.Assert.*;

import com.pedropathing.math.Pose;
import org.junit.Test;

public class RouteArrivalTest {
    private final Pose target = new Pose(53, 7, Math.toRadians(90));

    @Test
    public void exactArrivalIsAccepted() {
        assertTrue(BaseModularAuto.isAt(target, target, 3.5, 12));
    }

    @Test
    public void smallDriftCanUseExistingExtendedTolerance() {
        Pose actual = new Pose(58, 7, Math.toRadians(105));
        assertFalse(BaseModularAuto.isAt(actual, target, 3.5, 12));
        assertTrue(BaseModularAuto.isAt(actual, target, 6.5, 20));
    }

    @Test
    public void missedWaypointIsRejected() {
        assertFalse(BaseModularAuto.isAt(new Pose(60, 7, Math.toRadians(90)), target, 6.5, 20));
    }

    @Test
    public void wrongHeadingIsRejected() {
        assertFalse(BaseModularAuto.isAt(new Pose(53, 7, Math.toRadians(111)), target, 6.5, 20));
    }

    @Test
    public void headingWrapUsesShortestAngle() {
        assertTrue(BaseModularAuto.isAt(new Pose(53, 7, Math.toRadians(359)),
                new Pose(53, 7, Math.toRadians(1)), 3.5, 12));
    }

    @Test
    public void missingOrInvalidPoseIsRejected() {
        assertFalse(BaseModularAuto.isAt(null, target, 6.5, 20));
        assertFalse(BaseModularAuto.isAt(target, null, 6.5, 20));
        assertFalse(BaseModularAuto.isAt(new Pose(Double.NaN, 7, 0), target, 6.5, 20));
    }
}
