package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Shooter implements Mechanism {

    public static final double DEFAULT_P = 0.004;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0001;
    public static final double DEFAULT_F = 0.000458;

    private static final double DEFAULT_TARGET_VELOCITY = 1625;

    private final NextMotor flywheel;

    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean velocityGainsValid = true;

    public final Command runFlywheel = instant(() -> setVelocity(targetVelocity));
    public final Command stopFlywheel = instant(this::stopImmediately);

    public Shooter() {
        this(new NextMotor("flywheel"));
    }

    Shooter(NextMotor flywheel) {
        this.flywheel = flywheel;
    }

    public void initialize() {
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        stopImmediately();
        flywheel.setDirection(NextMotor.Direction.REVERSE);
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

        gains.setKP(p);
        gains.setKI(i);
        gains.setKD(d);
        gains.setKV(f);
        resetControllers();
        velocityGainsValid = true;
    }

    public double getFlywheelVelocity() { return flywheel.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getFlywheelPower() { return Math.max(-1, Math.min(1, flywheel.getThrottle())); }

    public boolean isEnabled() { return enabled; }

    public String getControlStatus() {
        if (!velocityGainsValid) return "INVALID PIDF";
        return enabled ? "VELOCITY CONTROL" : "STOPPED";
    }

    public void stopImmediately() {
        enabled = false;
        resetControllers();
        flywheel.setThrottle(0);
        flywheel.update();
    }

    private void resetControllers() {
        flywheel.getVelocityPID().reset();
    }
}
