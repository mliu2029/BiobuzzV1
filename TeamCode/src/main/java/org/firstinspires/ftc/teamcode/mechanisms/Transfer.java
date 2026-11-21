package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Configurable
public class Transfer implements Mechanism {
    public static final double ticksPerRevolution = 28.0;

    public static volatile double openPosition = 0.5;
    public static volatile double blockedPosition = 0.2;
    public static volatile double closedReverseRPM = 2140;

    // NextFTC Velocity PID & Feedforward Constants
    public static volatile double kP = 0.000002;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0.0;
    public static volatile double kV = 0.000204;
    public static volatile double rollerDefaultRPM = 1925;

    private final NextServo transfer = new NextServo("transfer");
    private final NextMotor roller = new NextMotor("rollers", Units.Rotations.of(1.0 / ticksPerRevolution));
    private boolean servoFreeMode = false;
    private boolean feeding;
    private boolean stopped = true;
    private boolean initialized;
    private boolean servoDisabled;
    private double rollerTarget = rollerDefaultRPM;

    public final Command feed = instant(() -> setFeeding(true));
    public final Command stopFeeding = instant(() -> setFeeding(false));
    public final Command toggleServoFree = instant(this::toggleServoFreeMode);

    // Aliases
    public final Command run = feed;
    public final Command stop = stopFeeding;

    public void initialize() {
        initialized = false;
        servoFreeMode = false;
        transfer.setDirection(NextMotor.Direction.FORWARD);
        roller.setDirection(NextMotor.Direction.FORWARD);
        rollerTarget = rollerDefaultRPM;
        stopImmediately();
        applyRollerVelocityConstants();
        initialized = true;
        setFeeding(false);
        periodic();
    }

    /** Apply NextFTC velocity constants to the roller motor */
    public void applyRollerVelocityConstants() {
        roller.getVelocityConstants()
                .withP(kP)
                .withI(kI)
                .withD(kD)
                .withV(kV);
    }

    public void setFeeding(boolean feeding) {
        this.feeding = feeding;
        stopped = false;
    }

    public boolean isFeeding() {
        return feeding && !stopped;
    }

    /** Set roller speed in RPM. */
    public void setRollerVelocity(double rpm) {
        rollerTarget = Math.max(0, rpm);
    }

    public void setServoFreeMode(boolean freeMode) {
        this.servoFreeMode = freeMode;
    }

    public boolean isServoFreeMode() {
        return servoFreeMode;
    }

    public void toggleServoFreeMode() {
        setServoFreeMode(!servoFreeMode);
    }

    @Override
    public void periodic() {
        if (!initialized) return;
        updateServo();
        double speed = getTargetVelocity();
        if (speed == 0) stopRoller();
        else {
            roller.setVelocitySetpoint(Units.RotationsPerMinute.of(speed));
        }
    }

    private void updateServo() {
        if (servoFreeMode != servoDisabled) {
            if (servoFreeMode) transfer.disable();
            else transfer.enable();
            servoDisabled = servoFreeMode;
        }
        if (!servoFreeMode) transfer.setPosition(feeding && !stopped ? openPosition : blockedPosition);
    }

    public void setRollerVelocityConstants(double kP, double kI, double kD, double kV) {
        roller.getVelocityConstants()
                .withP(kP)
                .withI(kI)
                .withD(kD)
                .withV(kV);
    }

    public MotorVelocityConstants getRollerVelocityConstants() {
        return roller.getVelocityConstants();
    }

    public NextMotor getRoller() {
        return roller;
    }

    /** Returns commanded velocity in RPM. */
    public double getTargetVelocity() {
        if (stopped) return 0;
        if (feeding) return rollerTarget;
        return servoFreeMode ? -Math.abs(closedReverseRPM) : 0;
    }

    public double getPosition() { return servoFreeMode ? Double.NaN : transfer.getPosition(); }

    /** Returns measured roller velocity in RPM. */
    public double getRollerVelocity() {
        return roller.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    public double getRollerPower() { return roller.getThrottle(); }

    public String getStatus() {
        if (stopped) return "Stopped";
        if (feeding) return rollerTarget <= 0 ? "Open"
                : servoFreeMode ? "Feeding (servo off)" : "Feeding";
        return getTargetVelocity() < 0 ? "Reverse hold (servo off)" : "Closed";
    }

    private void stopRoller() {
        roller.getVelocityPID().reset();
        roller.setThrottle(0);
    }

    public void stopImmediately() {
        stopped = true;
        feeding = false;
        stopRoller();
        if (initialized) {
            roller.update();
            updateServo();
        }
    }

    public void prepareForInit() {
        initialized = false;
        servoFreeMode = false;
        stopImmediately();
    }
}
