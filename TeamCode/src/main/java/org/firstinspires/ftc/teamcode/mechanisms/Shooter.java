package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Configurable
public class Shooter implements Mechanism {
    public static final double TICKS_PER_REVOLUTION = 28.0;

    // NextFTC Velocity PID & Feedforward Constants
    public static volatile double kP = 0.001867;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0.000;
    public static volatile double kV = 0.0001587;
    public static volatile double toleranceRPM = 160.0;

    public static volatile double defaultRPM = 3480.0;

    private final NextMotor flywheel = new NextMotor("flywheel", Units.Rotations.of(1.0 / TICKS_PER_REVOLUTION));
    private boolean enabled;
    private boolean initialized;
    private double targetRPM = defaultRPM;

    public final Command runFlywheel = instant(() -> setVelocity(targetRPM > 0 ? targetRPM : defaultRPM));
    public final Command stopFlywheel = instant(this::stopImmediately);

    /** Returns a command to adjust the target flywheel velocity. */
    public Command adjustRPM(double delta) {
        return instant(() -> adjustVelocity(delta));
    }

    public void adjustVelocity(double delta) {
        double base = enabled && targetRPM > 0 ? targetRPM : defaultRPM;
        setVelocity(Math.max(0, base + delta));
    }

    public void initialize() {
        targetRPM = defaultRPM;
        stopImmediately();
        flywheel.setDirection(NextMotor.Direction.REVERSE);
        setVelocityConstants(kP, kI, kD, kV);
        initialized = true;
    }

    /** Set flywheel target velocity in RPM. */
    public void setVelocity(double rpm) {
        double requested = Math.max(0, rpm);
        if (targetRPM == requested && enabled == (requested > 0)) return;
        targetRPM = requested;
        enabled = targetRPM > 0;
        if (enabled) {
            flywheel.setVelocitySetpoint(Units.RotationsPerMinute.of(targetRPM));
        } else {
            stopImmediately();
        }
    }

    public void setVelocityConstants(double kP, double kI, double kD, double kV) {
        flywheel.getVelocityConstants()
                .withP(kP)
                .withI(kI)
                .withD(kD)
                .withV(kV);
    }

    public double getTargetVelocity() { return targetRPM; }

    public double getFlywheelVelocity() {
        return flywheel.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    public double getFlywheelPower() { return flywheel.getThrottle(); }
    public boolean isEnabled() { return enabled; }

    public boolean isReady() {
        return enabled && targetRPM > 0
                && Math.abs(getFlywheelVelocity() - targetRPM) <= toleranceRPM;
    }

    public void stopImmediately() {
        enabled = false;
        flywheel.getVelocityPID().reset();
        flywheel.setThrottle(0);
        if (initialized) {
            flywheel.update();
        }
    }

    public void prepareForInit() {
        initialized = false;
        targetRPM = defaultRPM;
        stopImmediately();
    }
}
