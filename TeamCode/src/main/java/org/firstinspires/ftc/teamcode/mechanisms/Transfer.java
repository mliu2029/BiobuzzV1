package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

@Configurable
public class Transfer implements Mechanism {
    public static volatile double openPosition = 0.13;
    public static volatile double blockedPosition = 0.7;
    // Servo off: the gate goes limp and the rollers reverse to hold balls back instead.
    public static volatile boolean servoFreeMode = false;
    public static volatile double closedReverseVelocity = 1000  ;

    public static final double DEFAULT_ROLLER_P = 0.0001;
    public static final double DEFAULT_ROLLER_I = 0.0;
    public static final double DEFAULT_ROLLER_D = 0.0;
    public static final double DEFAULT_ROLLER_F = 0.0004279;
    public static final double DEFAULT_ROLLER_TARGET_VELOCITY = 1200;

    private enum State { STOPPED, CLOSED, OPEN }
    private enum ServoState { UNUSED, ENABLED, DISABLED }

    private final NextServo transfer = new NextServo("transfer");
    private final NextMotor roller = new NextMotor("rollers");
    private State state = State.STOPPED;
    private ServoState servoState = ServoState.UNUSED;
    private boolean initialized;
    private double rollerTargetVelocity = DEFAULT_ROLLER_TARGET_VELOCITY;

    public final Command feed = instant(() -> setFeeding(true));
    public final Command stopFeeding = instant(() -> setFeeding(false));

    public void initialize() {
        initialized = false;
        if (!servoFreeMode) transfer.setDirection(NextMotor.Direction.FORWARD);
        roller.setDirection(NextMotor.Direction.FORWARD);
        rollerTargetVelocity = DEFAULT_ROLLER_TARGET_VELOCITY;
        stopImmediately();
        setRollerPIDFCoefficients(DEFAULT_ROLLER_P, DEFAULT_ROLLER_I,
                DEFAULT_ROLLER_D, DEFAULT_ROLLER_F);
        initialized = true;
        setFeeding(false);
    }

    public double getPosition() { return servoFreeMode ? Double.NaN : transfer.getPosition(); }

    public void setFeeding(boolean feeding) {
        state = feeding ? State.OPEN : State.CLOSED;
        applyState();
    }

    public void stopImmediately() {
        state = State.STOPPED;
        VelocityMotors.stop(roller);
        updateServo();
    }

    public void prepareForInit() {
        initialized = false;
        state = State.STOPPED;
        VelocityMotors.stop(roller);
    }

    @Override
    public void periodic() {
        if (initialized) applyState();
    }

    public void setRollerVelocity(double velocity) {
        rollerTargetVelocity = Math.max(0, velocity);
        VelocityMotors.setVelocity(roller, getCommandedRollerVelocity());
    }

    public void setRollerPIDFCoefficients(double p, double i, double d, double f) {
        VelocityMotors.setPIDF(roller, p, i, d, f);
    }

    public double getCommandedRollerVelocity() {
        if (state == State.STOPPED) return 0;
        if (state == State.OPEN) return rollerTargetVelocity;
        return servoFreeMode ? -Math.abs(closedReverseVelocity) : 0;
    }

    public String getStatus() {
        if (state == State.STOPPED) return "Stopped";
        if (state == State.OPEN) {
            if (rollerTargetVelocity <= 0) return "Open";
            return servoFreeMode ? "Feeding (servo off)" : "Feeding";
        }
        return getCommandedRollerVelocity() < 0 ? "Reverse hold (servo off)" : "Closed";
    }

    public double getRollerVelocity() { return VelocityMotors.velocity(roller); }
    public double getRollerPower() { return roller.getThrottle(); }

    private void applyState() {
        updateServo();
        VelocityMotors.setVelocity(roller, getCommandedRollerVelocity());
    }

    private void updateServo() {
        if (servoFreeMode) {
            if (servoState == ServoState.ENABLED) {
                transfer.disable();
                servoState = ServoState.DISABLED;
            }
            return;
        }
        if (servoState == ServoState.DISABLED) {
            transfer.enable();
        }
        transfer.setPosition(state == State.OPEN ? openPosition : blockedPosition);
        servoState = ServoState.ENABLED;
    }
}
