package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Configurable
public class Transfer implements Mechanism {
    public static final double DEFAULT_P = 0.001;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0;
    public static final double DEFAULT_F = 0.0004;
    public static final double DEFAULT_TARGET_VELOCITY = 1550;

    public static volatile double openPosition = 0.5;
    public static volatile double blockedPosition = 0.5;

    private final NextServo transfer;
    private final NextMotor roller;
    private boolean blockerOpen;
    private boolean enabled;
    private boolean velocityGainsValid = true;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;

    public final Command openBlocker = instant(this::openImmediately);
    public final Command closeBlocker = instant(this::closeImmediately);

    public Transfer() {
        this(new NextMotor("rollers"), new NextServo("transfer"));
    }

    Transfer(NextMotor roller, NextServo transfer) {
        this.roller = roller;
        this.transfer = transfer;
    }

    public void initialize() {
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        transfer.setDirection(NextMotor.Direction.FORWARD);
        roller.setDirection(NextMotor.Direction.FORWARD);
        closeImmediately();
        velocityGainsValid = false;
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
    }

    public double getPosition() { return transfer.getPosition(); }
    public double getTargetVelocity() { return targetVelocity; }
    public double getRollerVelocity() { return roller.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getRollerPower() { return Math.max(-1, Math.min(1, roller.getThrottle())); }
    public boolean isEnabled() { return enabled; }

    public void setVelocity(double velocity) {
        double target = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
        boolean targetChanged = targetVelocity != target;
        targetVelocity = target;
        if (!blockerOpen || target == 0 || !velocityGainsValid) {
            stopRollerImmediately();
            return;
        }
        if (!enabled || targetChanged) {
            roller.getVelocityPID().reset();
        }
        enabled = true;
        roller.setVelocitySetpoint(Units.RadiansPerSecond.of(target));
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        if (!Double.isFinite(p) || !Double.isFinite(i)
                || !Double.isFinite(d) || !Double.isFinite(f)) {
            velocityGainsValid = false;
            stopRollerImmediately();
            return;
        }
        MotorVelocityConstants gains = roller.getVelocityConstants();
        if (velocityGainsValid && gains.getKP() == p && gains.getKI() == i
                && gains.getKD() == d && gains.getKV() == f) {
            return;
        }
        gains.setKP(p);
        gains.setKI(i);
        gains.setKD(d);
        gains.setKV(f);
        roller.getVelocityPID().reset();
        velocityGainsValid = true;
    }

    public void openImmediately() {
        blockerOpen = true;
        transfer.setPosition(openPosition);
        setVelocity(targetVelocity);
    }

    public void closeImmediately() {
        blockerOpen = false;
        stopRollerImmediately();
        transfer.setPosition(blockedPosition);
    }

    private void stopRollerImmediately() {
        enabled = false;
        roller.getVelocityPID().reset();
        roller.setThrottle(0);
        roller.update();
    }
}
