package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Configurable
public class Intake implements Mechanism {
    public static volatile double stallCurrentAmps = 4.0;
    public static volatile double stallVelocityRadPerSec = 5.0;
    public static volatile double stallDwellMs = 250.0;

    private final NextMotor intake = new NextMotor("intake");
    private boolean initialized;
    private long stallConditionStartNanos = 0;
    private boolean stalled = false;
    private boolean encoderMovementDetected = false;

    public final Command runIntake = instant(this::restart);
    public final Command reverseIntake = instant(this::reverse);
    public final Command stopIntake = instant(this::stopImmediately);

    public void initialize() {
        initialized = false;
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
        stallConditionStartNanos = 0;
        stalled = false;
        encoderMovementDetected = false;
        stopImmediately();
        initialized = true;
    }

    public void restart() {
        stallConditionStartNanos = 0;
        stalled = false;
        setPower(1);
    }

    public void reverse() {
        stallConditionStartNanos = 0;
        stalled = false;
        setPower(-1);
    }

    public void setPower(double power) {
        intake.setThrottle(power);
    }

    public double getVelocity() { return -intake.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getPower() { return intake.getThrottle(); }

    public double getCurrentAmps() {
        try {
            return intake.getCurrent().into(Units.Amperes);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    @Override
    public void periodic() {
        updateStallDetection(System.nanoTime());
    }

    public void updateStallDetection(long nowNanos) {
        double power = Math.abs(getPower());
        if (power < 0.2) {
            stallConditionStartNanos = 0;
            stalled = false;
            return;
        }

        double velocity = Math.abs(getVelocity());
        if (Double.isFinite(velocity) && velocity > 10.0) {
            encoderMovementDetected = true;
        }

        double current = getCurrentAmps();
        boolean highCurrent = Double.isFinite(current) && current >= stallCurrentAmps;
        boolean lowVelocity = encoderMovementDetected && Double.isFinite(velocity) && velocity < stallVelocityRadPerSec;

        boolean stallCondition = highCurrent || lowVelocity;

        if (stallCondition) {
            if (stallConditionStartNanos == 0) {
                stallConditionStartNanos = nowNanos;
            } else if ((nowNanos - stallConditionStartNanos) >= (long) (stallDwellMs * 1_000_000L)) {
                stalled = true;
            }
        } else {
            stallConditionStartNanos = 0;
            stalled = false;
        }
    }

    public boolean isStalled() {
        return stalled;
    }

    public String getStatus() {
        return isStalled() ? "Stalled" : getPower() > 0 ? "Intaking" : getPower() < 0 ? "Reverse" : "Off";
    }

    public void stopImmediately() {
        setPower(0);
        stallConditionStartNanos = 0;
        stalled = false;
        if (initialized) {
            intake.update();
        }
    }
}
