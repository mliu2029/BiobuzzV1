package org.firstinspires.ftc.teamcode.aiming;

import static org.junit.Assert.*;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.aiming.TurretCalculations.Alliance;
import org.firstinspires.ftc.teamcode.aiming.TurretCalculations.HiveTarget;

public class TurretCalculationsTest {
    private static final double EPS = 1e-8;

    @Test public void preservesAllFourSuppliedTargets() {
        assertEquals(58, HiveTarget.RED_CLOSE.x, EPS);
        assertEquals(59, HiveTarget.RED_CLOSE.y, EPS);
        assertEquals(84, HiveTarget.BLUE_CLOSE.x, EPS);
        assertEquals(59, HiveTarget.BLUE_CLOSE.y, EPS);
        assertEquals(58, HiveTarget.RED_FAR.x, EPS);
        assertEquals(85, HiveTarget.RED_FAR.y, EPS);
        assertEquals(84, HiveTarget.BLUE_FAR.x, EPS);
        assertEquals(85, HiveTarget.BLUE_FAR.y, EPS);
    }

    @Test public void bearingSubtractsHeadingAndWraps() {
        assertEquals(0, TurretCalculations.bearing(58, 9, Math.PI / 2, 0, 0,
                HiveTarget.RED_CLOSE), EPS);
        assertEquals(90, TurretCalculations.bearing(58, 9, 0, 0, 0,
                HiveTarget.RED_CLOSE), EPS);
        assertEquals(-90, TurretCalculations.bearing(58, 9, Math.PI, 0, 0,
                HiveTarget.RED_CLOSE), EPS);
        assertEquals(-179, TurretCalculations.normalize(181), EPS);
        assertEquals(179, TurretCalculations.normalize(-181), EPS);
        assertTrue(Double.isNaN(TurretCalculations.bearing(Double.POSITIVE_INFINITY, 9,
                0, 0, 0, HiveTarget.RED_CLOSE)));
    }

    @Test public void turretPivotOffsetRotatesWithRobot() {
        // Facing north, a pivot 10 inches left lies at (48, 49).
        assertEquals(-45, TurretCalculations.bearing(58, 49, Math.PI / 2, 0, 10,
                HiveTarget.RED_CLOSE), EPS);
        assertTrue(Double.isNaN(TurretCalculations.bearing(58, 59, 0, 0, 0,
                HiveTarget.RED_CLOSE)));
    }

    @Test public void choosesEquivalentInsideLimitsWithoutCrossingStop() {
        assertEquals(181, TurretCalculations.reachableAngle(-179, 175, -200, 200), EPS);
        assertEquals(-179, TurretCalculations.reachableAngle(-179, 175, -180, 180), EPS);
        assertTrue(Double.isNaN(TurretCalculations.reachableAngle(150, 0, -90, 90)));
        assertTrue(Double.isNaN(TurretCalculations.reachableAngle(Double.NaN, 0, -90, 90)));
        assertTrue(Double.isNaN(TurretCalculations.reachableAngle(0, 0, 0, 0)));
    }

    @Test public void visionRequiresSelectedTagFreshFrameAndMatchingPipeline() {
        HiveTarget target = HiveTarget.RED_CLOSE;
        assertTrue(TurretCalculations.usableVision(target, 34, 0, 0, 50, 100, 3, 10));
        assertFalse(TurretCalculations.usableVision(target, 38, 0, 0, 50, 100, 3, 10));
        assertFalse(TurretCalculations.usableVision(target, 30, 0, 0, 50, 100, 3, 10));
        assertFalse(TurretCalculations.usableVision(null, 34, 0, 0, 50, 100, 3, 10));
        assertFalse(TurretCalculations.usableVision(target, 34, 1, 0, 50, 100, 3, 10));
        assertFalse(TurretCalculations.usableVision(target, 34, 0, 0, 101, 100, 3, 10));
        assertFalse(TurretCalculations.usableVision(target, 34, 0, 0, -1, 100, 3, 10));
        assertFalse(TurretCalculations.usableVision(target, 34, 0, 0, 50, 100, 11, 10));
        assertFalse(TurretCalculations.usableVision(target, 34, 0, 0, 50, 100, Double.NaN, 10));
    }

    @Test public void changesTargetAtHalfwayAndNeverChangesAlliance() {
        double halfway = 70.75;
        assertSame(HiveTarget.RED_CLOSE, Alliance.RED.targetForY(halfway - .01, halfway));
        assertSame(HiveTarget.RED_FAR, Alliance.RED.targetForY(halfway, halfway));
        assertSame(HiveTarget.RED_FAR, Alliance.RED.targetForY(halfway + .01, halfway));
        assertSame(HiveTarget.RED_CLOSE, Alliance.RED.targetForY(20, halfway));
        assertSame(HiveTarget.BLUE_CLOSE, Alliance.BLUE.targetForY(20, halfway));
        assertSame(HiveTarget.BLUE_FAR, Alliance.BLUE.targetForY(100, halfway));
        assertFalse(Alliance.RED.correctTarget(HiveTarget.BLUE_CLOSE));
        assertFalse(Alliance.BLUE.correctTarget(HiveTarget.RED_FAR));
        assertNull(Alliance.RED.targetForY(Double.NaN, halfway));
        assertNull(Alliance.BLUE.targetForY(20, Double.NaN));
    }

    @Test public void acceptsEveryTagInOnlyTheSelectedCellCluster() {
        HiveTarget[] expected = {
                HiveTarget.RED_FAR, HiveTarget.RED_FAR, HiveTarget.RED_FAR, HiveTarget.RED_FAR,
                HiveTarget.RED_CLOSE, HiveTarget.RED_CLOSE, HiveTarget.RED_CLOSE, HiveTarget.RED_CLOSE,
                HiveTarget.BLUE_CLOSE, HiveTarget.BLUE_CLOSE, HiveTarget.BLUE_CLOSE, HiveTarget.BLUE_CLOSE,
                HiveTarget.BLUE_FAR, HiveTarget.BLUE_FAR, HiveTarget.BLUE_FAR, HiveTarget.BLUE_FAR};
        for (int id = 30; id <= 45; id++) {
            for (HiveTarget target : HiveTarget.values()) {
                assertEquals(expected[id - 30] == target, target.acceptsTag(id));
            }
        }
        assertFalse(HiveTarget.RED_FAR.acceptsTag(29));
        assertFalse(HiveTarget.BLUE_FAR.acceptsTag(46));
    }
}
