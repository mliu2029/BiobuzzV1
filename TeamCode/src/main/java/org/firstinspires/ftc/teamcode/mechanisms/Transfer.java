package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Configurable
public class Transfer implements Mechanism {
    public static final double ticksPerRevolution = 28.0;

    public static volatile double openPosition = 0.5;
    public static volatile double blockedPosition = 0.2;

    public static volatile double kP = 0.000002;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0.0;
    public static volatile double kV = 0.000204;
    public static volatile double rollerDefaultRPM = 1925;

    private final NextServo transfer = new NextServo("transfer");
    private final NextMotor roller = new NextMotor("rollers", Units.Rotations.of(1.0 / ticksPerRevolution));
    private boolean feeding;
    private boolean stopped = true;
    private boolean initialized;
    private double rollerTarget = rollerDefaultRPM;

    public final Command feed = instant(() -> setFeeding(true));
    public final Command stopFeeding = instant(() -> setFeeding(false));

    public void initialize() {
        initialized = false;
        transfer.setDirection(NextMotor.Direction.FORWARD);
        transfer.enable();
        roller.setDirection(NextMotor.Direction.FORWARD);
        rollerTarget = rollerDefaultRPM;
        stopImmediately();
        setRollerVelocityConstants(kP, kI, kD, kV);
        initialized = true;
        setFeeding(false);
        periodic();
    }

    public void setFeeding(boolean feeding) {
        this.feeding = feeding;
        stopped = false;
    }

    public void setRollerVelocity(double rpm) {
        rollerTarget = Math.max(0, rpm);
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
        transfer.setPosition(feeding && !stopped ? openPosition : blockedPosition);
    }

    public void setRollerVelocityConstants(double kP, double kI, double kD, double kV) {
        roller.getVelocityConstants()
                .withP(kP)
                .withI(kI)
                .withD(kD)
                .withV(kV);
    }

    /** Returns commanded velocity in RPM. */
    public double getTargetVelocity() {
        return feeding && !stopped ? rollerTarget : 0;
    }

    /** Returns measured roller velocity in RPM. */
    public double getRollerVelocity() {
        return roller.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    public double getRollerPower() { return roller.getThrottle(); }

    public String getStatus() {
        if (stopped) return "Stopped";
        if (feeding) return rollerTarget <= 0 ? "Open" : "Feeding";
        return "Closed";
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
        stopImmediately();
    }
}
