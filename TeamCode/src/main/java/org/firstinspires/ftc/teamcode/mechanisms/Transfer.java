package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Configurable
public class Transfer implements Mechanism {
    public static volatile double openPosition = 0.13;
    public static volatile double blockedPosition = 0.7;
    public static volatile boolean servoFreeMode = false;
    public static volatile double closedReverseVelocity = 1000;

    public static final double DEFAULT_ROLLER_P = 0.0001;
    public static final double DEFAULT_ROLLER_I = 0.0;
    public static final double DEFAULT_ROLLER_D = 0.0;
    public static final double DEFAULT_ROLLER_F = 0.0004279;
    public static final double DEFAULT_ROLLER_TARGET_VELOCITY = 1200;

    private final NextServo transfer = new NextServo("transfer");
    private final NextMotor roller = new NextMotor("rollers");
    private boolean feeding;
    private boolean stopped = true;
    private boolean initialized;
    private boolean servoDisabled;
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
        periodic();
    }

    public void setFeeding(boolean feeding) {
        this.feeding = feeding;
        stopped = false;
    }

    public void setRollerVelocity(double velocity) {
        rollerTargetVelocity = Math.max(0, velocity);
    }

    @Override
    public void periodic() {
        if (!initialized) return;
        updateServo();
        double velocity = getCommandedRollerVelocity();
        if (velocity == 0) stopRoller();
        else roller.setVelocitySetpoint(Units.RadiansPerSecond.of(velocity));
    }

    private void updateServo() {
        if (servoFreeMode != servoDisabled) {
            if (servoFreeMode) transfer.disable();
            else transfer.enable();
            servoDisabled = servoFreeMode;
        }
        if (!servoFreeMode) transfer.setPosition(feeding && !stopped ? openPosition : blockedPosition);
    }

    public void setRollerPIDFCoefficients(double p, double i, double d, double f) {
        roller.getVelocityConstants().withP(p).withI(i).withD(d).withV(f);
    }

    public double getCommandedRollerVelocity() {
        if (stopped) return 0;
        if (feeding) return rollerTargetVelocity;
        return servoFreeMode ? -Math.abs(closedReverseVelocity) : 0;
    }

    public double getPosition() { return servoFreeMode ? Double.NaN : transfer.getPosition(); }
    public double getRollerVelocity() { return roller.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getRollerPower() { return roller.getThrottle(); }
    public String getStatus() {
        if (stopped) return "Stopped";
        if (feeding) return rollerTargetVelocity <= 0 ? "Open"
                : servoFreeMode ? "Feeding (servo off)" : "Feeding";
        return getCommandedRollerVelocity() < 0 ? "Reverse hold (servo off)" : "Closed";
    }

    private void stopRoller() {
        roller.getVelocityPID().reset();
        roller.setThrottle(0);
        roller.update();
    }

    public void stopImmediately() {
        stopped = true;
        feeding = false;
        stopRoller();
        if (initialized) updateServo();
    }

    public void prepareForInit() {
        initialized = false;
        stopImmediately();
    }
}
