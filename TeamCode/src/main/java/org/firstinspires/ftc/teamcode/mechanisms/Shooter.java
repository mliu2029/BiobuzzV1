package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Shooter implements Mechanism {
    public static final double DEFAULT_P = 0.004;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.000;
    public static final double DEFAULT_F = 0.00034;

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
        double requested = Math.max(0, velocity);
        if (targetVelocity == requested && enabled == (requested > 0)) return;
        targetVelocity = requested;
        enabled = targetVelocity > 0;
        if (enabled) {
            flywheel.setVelocitySetpoint(Units.RadiansPerSecond.of(targetVelocity));
        } else {
            stopImmediately();
        }
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        flywheel.getVelocityConstants().withP(p).withI(i).withD(d).withV(f);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getFlywheelVelocity() { return flywheel.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getFlywheelPower() { return flywheel.getThrottle(); }
    public boolean isEnabled() { return enabled; }

    public String getControlStatus() {
        return enabled ? "VELOCITY CONTROL" : "STOPPED";
    }

    public void stopImmediately() {
        enabled = false;
        flywheel.getVelocityPID().reset();
        flywheel.setThrottle(0);
        flywheel.update();
    }
}
