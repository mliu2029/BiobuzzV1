package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.aiming.TurretConfig;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class TurretHoldingTest {
    private double[] saved;

    @Before public void configureController() {
        saved = new double[] {TurretConfig.p, TurretConfig.i, TurretConfig.d,
                TurretConfig.staticPower, TurretConfig.maxPower,
                TurretConfig.ticksPerDegree, TurretConfig.toleranceDegrees};
        TurretConfig.p = 0.05;
        TurretConfig.i = 0;
        TurretConfig.d = 0;
        TurretConfig.staticPower = 0;
        TurretConfig.maxPower = 0.25;
        TurretConfig.ticksPerDegree = 10;
        TurretConfig.toleranceDegrees = 2;
    }

    @After public void restoreController() {
        TurretConfig.p = saved[0];
        TurretConfig.i = saved[1];
        TurretConfig.d = saved[2];
        TurretConfig.staticPower = saved[3];
        TurretConfig.maxPower = saved[4];
        TurretConfig.ticksPerDegree = saved[5];
        TurretConfig.toleranceDegrees = saved[6];
    }

    @Test public void correctsErrorsInsideAlignmentTolerance() {
        Turret turret = new Turret();
        assertEquals(0.05, turret.calculatePower(1), 1e-8);
        assertEquals(-0.05, turret.calculatePower(-1), 1e-8);
    }

    @Test public void staticFrictionDoesNotPushAtExactlyZeroError() {
        TurretConfig.staticPower = 0.08;
        Turret turret = new Turret();
        assertEquals(0, turret.calculatePower(0), 1e-8);
        assertEquals(0.13, turret.calculatePower(1), 1e-8);
        assertEquals(-0.13, turret.calculatePower(-1), 1e-8);
    }

    @Test public void respectsPowerCapAndReversedEncoderConversion() {
        TurretConfig.ticksPerDegree = -10;
        Turret turret = new Turret();
        assertEquals(-0.25, turret.calculatePower(100), 1e-8);
        assertEquals(0.25, turret.calculatePower(-100), 1e-8);
        assertEquals(-0.05, turret.calculatePower(1), 1e-8);
    }
}
