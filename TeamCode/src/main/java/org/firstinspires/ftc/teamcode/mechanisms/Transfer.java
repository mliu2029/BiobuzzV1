package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/** Owns the feeding rollers and blocker; the shooter flywheel runs independently. */
@Configurable
public class Transfer implements Mechanism {
    public static volatile double openPosition = 0.13;
    public static volatile double blockedPosition = 0.7;

    public static final double DEFAULT_ROLLER_P = 0.0001;
    public static final double DEFAULT_ROLLER_I = 0.0;
    public static final double DEFAULT_ROLLER_D = 0.0;
    public static final double DEFAULT_ROLLER_F = 0.0004279;
    public static final double DEFAULT_ROLLER_TARGET_VELOCITY = 1200;

    private final NextServo transfer;
    private final NextMotor roller;
    private boolean feeding;
    private boolean rollerGainsValid = true;
    private double rollerTargetVelocity = DEFAULT_ROLLER_TARGET_VELOCITY;

    public final Command feed = instant(this::openImmediately);
    public final Command stopFeeding = instant(this::stopImmediately);

    public Transfer() {
        this(new NextServo("transfer"), new NextMotor("rollers"));
    }

    Transfer(NextServo transfer, NextMotor roller) {
        this.transfer = transfer;
        this.roller = roller;
    }

    public void initialize() {
        transfer.setDirection(NextMotor.Direction.FORWARD);
        roller.setDirection(NextMotor.Direction.FORWARD);
        rollerTargetVelocity = DEFAULT_ROLLER_TARGET_VELOCITY;
        stopImmediately();
        rollerGainsValid = false;
        setRollerPIDFCoefficients(DEFAULT_ROLLER_P, DEFAULT_ROLLER_I,
                DEFAULT_ROLLER_D, DEFAULT_ROLLER_F);
        openImmediately();
        closeImmediately();
    }

    public double getPosition() { return transfer.getPosition(); }

    public void openImmediately() {
        transfer.setPosition(openPosition);
        if (!feeding) roller.getVelocityPID().reset();
        feeding = true;
        updateRollerVelocity();
    }

    public void closeImmediately() {
        feeding = false;
        stopRollerImmediately();
        transfer.setPosition(blockedPosition);
    }

    public void stopImmediately() {
        closeImmediately();
    }

     public void setRollerVelocity(double velocity) {
        double target = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
        if (rollerTargetVelocity != target) roller.getVelocityPID().reset();
        rollerTargetVelocity = target;
        updateRollerVelocity();
    }

    public void setRollerPIDFCoefficients(double p, double i, double d, double f) {
        if (!Double.isFinite(p) || !Double.isFinite(i)
                || !Double.isFinite(d) || !Double.isFinite(f)) {
            rollerGainsValid = false;
            stopRollerImmediately();
            return;
        }
        MotorVelocityConstants gains = roller.getVelocityConstants();
        if (rollerGainsValid && gains.getKP() == p && gains.getKI() == i
                && gains.getKD() == d && gains.getKV() == f) {
            return;
        }
        gains.setKP(p);
        gains.setKI(i);
        gains.setKD(d);
        gains.setKV(f);
        roller.getVelocityPID().reset();
        rollerGainsValid = true;
        updateRollerVelocity();
    }

    public boolean isFeeding() {
        return feeding && rollerGainsValid && rollerTargetVelocity > 0;
    }

    public double getRollerTargetVelocity() { return rollerTargetVelocity; }
    public double getRollerVelocity() { return roller.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getRollerPower() { return Math.max(-1, Math.min(1, roller.getThrottle())); }

    private void updateRollerVelocity() {
        if (!isFeeding()) {
            stopRollerImmediately();
            return;
        }
        roller.setVelocitySetpoint(Units.RadiansPerSecond.of(rollerTargetVelocity));
    }

    private void stopRollerImmediately() {
        roller.getVelocityPID().reset();
        roller.setThrottle(0);
        roller.update();
    }
}
