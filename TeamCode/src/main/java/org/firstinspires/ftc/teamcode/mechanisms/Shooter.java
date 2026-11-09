package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    public static final double DEFAULT_P = 0.004;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0001;
    public static final double DEFAULT_F = 0.000458;

    private static final double DEFAULT_TARGET_VELOCITY = 1625;

    private final NextMotor flywheel = new NextMotor("flywheel");
    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;

    public final Command runFlywheel = instant(() -> setVelocity(targetVelocity));

    public void initialize() {
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        stopImmediately();
        flywheel.setDirection(NextMotor.Direction.REVERSE);
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
    }

    public void setVelocity(double velocity) {
        targetVelocity = Math.max(0, velocity);
        enabled = targetVelocity > 0;
        if (enabled) {
            VelocityMotors.setVelocity(flywheel, targetVelocity);
        } else {
            stopImmediately();
        }
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        VelocityMotors.setPIDF(flywheel, p, i, d, f);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getFlywheelVelocity() { return VelocityMotors.velocity(flywheel); }
    public double getFlywheelPower() { return flywheel.getThrottle(); }
    public boolean isEnabled() { return enabled; }

    public String getControlStatus() {
        return enabled ? "VELOCITY CONTROL" : "STOPPED";
    }

    /** Cuts power but keeps the target, so runFlywheel resumes at the same speed. */
    public void stopImmediately() {
        enabled = false;
        VelocityMotors.stop(flywheel);
    }
}
