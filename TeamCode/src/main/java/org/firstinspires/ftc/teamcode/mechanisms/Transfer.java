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
    public static volatile boolean servoFreeMode = false;
    /** Positive reverse-hold magnitude; the current default encoder scale makes this ticks/s. */
    public static volatile double closedReverseVelocity = 500;

    public static final double DEFAULT_ROLLER_P = 0.0001;
    public static final double DEFAULT_ROLLER_I = 0.0;
    public static final double DEFAULT_ROLLER_D = 0.0;
    public static final double DEFAULT_ROLLER_F = 0.0004279;
    public static final double DEFAULT_ROLLER_TARGET_VELOCITY = 1200;

    private final NextServo transfer;
    private final NextMotor roller;
    private enum State { STOPPED, CLOSED, OPEN }
    private State state = State.STOPPED;
    private boolean initialized;
    private boolean servoUsed;
    private boolean servoDisabled;
    private boolean rollerGainsValid = true;
    private double rollerTargetVelocity = DEFAULT_ROLLER_TARGET_VELOCITY;
    private double appliedRollerVelocity;

    public final Command feed = instant(this::openImmediately);
    public final Command stopFeeding = instant(this::closeImmediately);

    public Transfer() {
        this(new NextServo("transfer"), new NextMotor("rollers"));
    }

    Transfer(NextServo transfer, NextMotor roller) {
        this.transfer = transfer;
        this.roller = roller;
    }

    public void initialize() {
        initialized = false;
        if (!servoFreeMode) transfer.setDirection(NextMotor.Direction.FORWARD);
        roller.setDirection(NextMotor.Direction.FORWARD);
        rollerTargetVelocity = DEFAULT_ROLLER_TARGET_VELOCITY;
        stopImmediately();
        rollerGainsValid = false;
        setRollerPIDFCoefficients(DEFAULT_ROLLER_P, DEFAULT_ROLLER_I,
                DEFAULT_ROLLER_D, DEFAULT_ROLLER_F);
        initialized = true;
        if (!servoFreeMode) openImmediately();
        closeImmediately();
    }

    public double getPosition() { return servoFreeMode ? Double.NaN : transfer.getPosition(); }

    public void openImmediately() {
        state = State.OPEN;
        applyState();
    }

    public void closeImmediately() {
        state = State.CLOSED;
        applyState();
    }

    public void stopImmediately() {
        state = State.STOPPED;
        stopRollerImmediately();
        updateServo();
    }

    /** Stop the rollers during INIT while leaving the blocker servo position alone. */
    public void prepareForInit() {
        initialized = false;
        state = State.STOPPED;
        stopRollerImmediately();
    }

    @Override
    public void periodic() {
        if (initialized) applyState();
    }

    public void setRollerVelocity(double velocity) {
        double target = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
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
        return state == State.OPEN && rollerGainsValid && rollerTargetVelocity > 0;
    }

    /** Actual signed target, including reverse hold; STOP always returns zero. */
    public double getCommandedRollerVelocity() {
        if (!rollerGainsValid || state == State.STOPPED) return 0;
        if (state == State.OPEN) return rollerTargetVelocity;
        return servoFreeMode && Double.isFinite(closedReverseVelocity)
                ? -Math.abs(closedReverseVelocity) : 0;
    }

    public String getStatus() {
        if (state == State.STOPPED) return "Stopped";
        if (!rollerGainsValid) return "Roller PIDF invalid";
        if (state == State.OPEN) return isFeeding()
                ? (servoFreeMode ? "Feeding (servo off)" : "Feeding") : "Open";
        return getCommandedRollerVelocity() < 0 ? "Reverse hold (servo off)" : "Closed";
    }

    public double getRollerTargetVelocity() { return rollerTargetVelocity; }
    public double getRollerVelocity() { return roller.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getRollerPower() { return Math.max(-1, Math.min(1, roller.getThrottle())); }

    private void updateRollerVelocity() {
        double velocity = getCommandedRollerVelocity();
        if (velocity == 0) {
            stopRollerImmediately();
            return;
        }
        if (appliedRollerVelocity != velocity) roller.getVelocityPID().reset();
        appliedRollerVelocity = velocity;
        roller.setVelocitySetpoint(Units.RadiansPerSecond.of(velocity));
    }

    private void applyState() {
        updateServo();
        updateRollerVelocity();
    }

    private void updateServo() {
        if (servoFreeMode) {
            // Skip servo lookup entirely if this OpMode never used it. This also works
            // with the broken servo removed from the robot configuration.
            if (servoUsed && !servoDisabled) {
                transfer.disable();
                servoDisabled = true;
            }
            return;
        }
        if (servoDisabled) {
            transfer.enable();
            servoDisabled = false;
        }
        transfer.setPosition(state == State.OPEN ? openPosition : blockedPosition);
        servoUsed = true;
    }

    private void stopRollerImmediately() {
        appliedRollerVelocity = 0;
        roller.getVelocityPID().reset();
        roller.setThrottle(0);
        roller.update();
    }
}
