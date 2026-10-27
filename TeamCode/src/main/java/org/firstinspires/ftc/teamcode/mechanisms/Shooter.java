package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Shooter implements Mechanism {

    // TODO: retune ts PID.
    public static final double DEFAULT_P = 0.001;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0;
    public static final double DEFAULT_F = 0.000357;

    private static final double DEFAULT_TARGET_VELOCITY = 1550;

    private final NextMotor flywheel = new NextMotor("flywheel");
    private final NextMotor roller = new NextMotor("rollers");

    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean velocityGainsValid = true;

    public final Command runFlywheel = instant(() -> setVelocity(targetVelocity));
    public final Command stopFlywheel = instant(this::stopImmediately);

    public void initialize() {
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        stopImmediately();
        flywheel.setDirection(NextMotor.Direction.REVERSE);
        roller.setDirection(NextMotor.Direction.FORWARD);
        velocityGainsValid = false;
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
    }

    public void adjustVelocity(int change) {
        double target = Math.max(0, targetVelocity + change);
        if (enabled) {
            setVelocity(target);
        } else {
            targetVelocity = target;
        }
    }

    public double getTargetVelocity() { return targetVelocity; }

    public void setVelocity(double velocity) {
        double target = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
        boolean targetChanged = targetVelocity != target;
        targetVelocity = target;
        if (target == 0 || !velocityGainsValid) {
            stopImmediately();
            return;
        }
        if (!enabled || targetChanged) {
            resetControllers();
        }
        enabled = true;
        flywheel.setVelocitySetpoint(Units.RadiansPerSecond.of(target));
        roller.setVelocitySetpoint(Units.RadiansPerSecond.of(target));
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        if (!Double.isFinite(p) || !Double.isFinite(i)
                || !Double.isFinite(d) || !Double.isFinite(f)) {
            velocityGainsValid = false;
            stopImmediately();
            return;
        }

        MotorVelocityConstants gains = flywheel.getVelocityConstants();
        if (velocityGainsValid && gains.getKP() == p && gains.getKI() == i
                && gains.getKD() == d && gains.getKV() == f) {
            return;
        }

        for (NextMotor motor : new NextMotor[]{flywheel, roller}) {
            motor.getVelocityConstants().setKP(p);
            motor.getVelocityConstants().setKI(i);
            motor.getVelocityConstants().setKD(d);
            motor.getVelocityConstants().setKV(f);
        }
        resetControllers();
        velocityGainsValid = true;
    }

    public double getFlywheelVelocity() { return flywheel.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getRollerVelocity() { return roller.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getFlywheelPower() { return Math.max(-1, Math.min(1, flywheel.getThrottle())); }
    public double getRollerPower() { return Math.max(-1, Math.min(1, roller.getThrottle())); }

    public boolean isEnabled() { return enabled; }

    public void stopImmediately() {
        enabled = false;
        resetControllers();
        flywheel.setThrottle(0);
        roller.setThrottle(0);
        flywheel.update();
        roller.update();
    }

    private void resetControllers() {
        flywheel.getVelocityPID().reset();
        roller.getVelocityPID().reset();
    }
}
