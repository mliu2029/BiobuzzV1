package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.reflect.Proxy;
import org.junit.Before;
import org.junit.Test;

public class ShooterTest {
    private final Shooter shooter = new Shooter();
    private double power;
    private double measuredVelocity;
    private DcMotor.RunMode mode;

    @Before
    public void initialize() {
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[] {DcMotorEx.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower":
                            power = (double) args[0];
                            return null;
                        case "getPower":
                            return power;
                        case "setMode":
                            mode = (DcMotor.RunMode) args[0];
                            return null;
                        case "getVelocity":
                            return measuredVelocity;
                        default:
                            throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
        shooter.initializeMotor(motor, null);
    }

    @Test
    public void usesSoftwareControlAndStartsStopped() {
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, mode);
        assertFalse(shooter.isEnabled());
        assertEquals(1375, shooter.getTargetVelocity(), 0);
        assertEquals(1275, shooter.getMinimumVelocity(), 0);
        shooter.setVelocity(1375);
        shooter.periodic();
        assertEquals(0, power, 0); // Old REV Hub gains are not reused.
    }

    @Test
    public void combinesPidAndFeedforwardAndRespondsEveryLoop() {
        shooter.setPIDFCoefficients(0.001, 0, 0, 0.0002);
        shooter.setVelocity(1000);
        measuredVelocity = 500;
        shooter.periodic();
        assertEquals(0.7, power, 1e-8);

        // The setpoint command has finished; periodic control must keep running.
        measuredVelocity = 900;
        shooter.periodic();
        assertEquals(0.3, power, 1e-8);
    }

    @Test
    public void clampsPowerWithoutReversingTheFlywheel() {
        shooter.setPIDFCoefficients(0.01, 0, 0, 0);
        shooter.setVelocity(1000);
        shooter.periodic();
        assertEquals(1, power, 0);
        measuredVelocity = 2000;
        shooter.periodic();
        assertEquals(0, power, 0);
    }

    @Test
    public void zeroTargetAndStopRemovePowerImmediately() {
        shooter.setPIDFCoefficients(0.001, 0, 0, 0);
        shooter.setVelocity(500);
        shooter.periodic();
        assertEquals(0.5, power, 1e-8);
        shooter.setVelocity(0);
        assertEquals(0, power, 0);
        assertFalse(shooter.isEnabled());

        shooter.setVelocity(500);
        shooter.periodic();
        shooter.stop();
        assertEquals(0, power, 0);
        shooter.periodic();
        assertEquals(0, power, 0);
    }

    @Test
    public void invalidGainsStopPowerUntilValidGainsAreRestored() {
        shooter.setPIDFCoefficients(0.001, 0, 0, 0);
        shooter.setVelocity(500);
        shooter.periodic();
        shooter.setPIDFCoefficients(Double.NaN, 0, 0, 0);
        assertEquals(0, power, 0);
        shooter.periodic();
        assertEquals(0, power, 0);
        assertFalse(shooter.isReady());

        shooter.setPIDFCoefficients(0.001, 0, 0, 0);
        shooter.periodic();
        assertEquals(0.5, power, 1e-8);
    }

    @Test
    public void invalidTargetsStopAndInvalidMeasurementsCannotReportReady() {
        shooter.setPIDFCoefficients(0.001, 0, 0, 0);
        for (double target : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
            shooter.setVelocity(500);
            shooter.periodic();
            shooter.setVelocity(target);
            assertEquals(0, power, 0);
            assertFalse(shooter.isEnabled());
        }
        shooter.setVelocity(500);
        for (double velocity : new double[] {Double.NaN, Double.POSITIVE_INFINITY}) {
            measuredVelocity = velocity;
            shooter.periodic();
            assertEquals(0, power, 0);
            assertFalse(shooter.isReady());
        }
    }

    @Test
    public void preservesReadinessThresholdAndTargetAdjustments() {
        shooter.setPIDFCoefficients(0.001, 0, 0, 0);
        shooter.setVelocity(1375);
        measuredVelocity = 1274;
        assertFalse(shooter.isReady());
        measuredVelocity = 1275;
        assertTrue(shooter.isReady());
        shooter.adjustVelocity(25);
        assertEquals(1400, shooter.getTargetVelocity(), 0);
        assertFalse(shooter.isReady());
        measuredVelocity = 1300;
        assertTrue(shooter.isReady());
        shooter.stopImmediately();
        assertFalse(shooter.isReady());
    }

    @Test
    public void reinitializationClearsTargetsGainsAndEnabledState() {
        shooter.setPIDFCoefficients(0.001, 0, 0, 0);
        shooter.setVelocity(500);
        shooter.periodic();
        initialize();
        assertFalse(shooter.isEnabled());
        assertEquals(1375, shooter.getTargetVelocity(), 0);
        shooter.setVelocity(1375);
        shooter.periodic();
        assertEquals(0, power, 0);
    }
}
