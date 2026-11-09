package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import java.util.function.LongSupplier;

@Configurable
public class Intake implements Mechanism {
    public static final double DEFAULT_P = 0.000001;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0;
    public static final double DEFAULT_F = 0.000323;
    public static final double DEFAULT_TARGET_VELOCITY = 2500;

    // Starting values: tune with the loaded intake on the robot.
    public static volatile boolean stallDetectionEnabled = true;
    public static volatile double stallVelocityFraction = 0.10;
    public static volatile double stallStartupDelayMs = 500;
    public static volatile double stallConfirmMs = 300;

    private final NextMotor intake;
    private final LongSupplier nanoTime;
    private double requestedVelocity;
    private double targetVelocity;
    private boolean stalled;
    private long runStartedNanos;
    private long slowSinceNanos = Long.MIN_VALUE;

    public final Command runIntake = instant(this::restart);

    public Intake() {
        this(new NextMotor("intake"), System::nanoTime);
    }

    Intake(NextMotor intake, LongSupplier nanoTime) {
        this.intake = intake;
        this.nanoTime = nanoTime;
    }

    public void initialize() {
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
        stopImmediately();
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
    }

    public void setVelocity(double velocity) {
        if (!Double.isFinite(velocity) || velocity == 0) {
            stopImmediately();
            return;
        }
        if (velocity < 0) {
            // Reverse remains available to clear a jam.
            stalled = false;
            slowSinceNanos = Long.MIN_VALUE;
        } else if (requestedVelocity <= 0) {
            runStartedNanos = nanoTime.getAsLong();
            slowSinceNanos = Long.MIN_VALUE;
        }
        requestedVelocity = velocity;
        targetVelocity = stalled ? 0 : velocity;
        VelocityMotors.setVelocity(intake, targetVelocity);
    }

    /** Explicit forward restart, including after the full/stalled latch has stopped the motor. */
    public void restart() {
        stalled = false;
        requestedVelocity = 0;
        setVelocity(DEFAULT_TARGET_VELOCITY);
    }

    @Override
    public void periodic() {
        if (!stallDetectionEnabled || requestedVelocity <= 0 || stalled) {
            slowSinceNanos = Long.MIN_VALUE;
            return;
        }
        long now = nanoTime.getAsLong();
        if ((now - runStartedNanos) / 1e6 < stallStartupDelayMs
                || Math.abs(getVelocity()) >= requestedVelocity * stallVelocityFraction) {
            slowSinceNanos = Long.MIN_VALUE;
            return;
        }
        if (slowSinceNanos == Long.MIN_VALUE) slowSinceNanos = now;
        if ((now - slowSinceNanos) / 1e6 >= stallConfirmMs) {
            stalled = true;
            targetVelocity = 0;
            // Switch out of velocity PID, so it cannot keep pushing against the blockage.
            VelocityMotors.stop(intake);
        }
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        VelocityMotors.setPIDF(intake, p, i, d, f);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getVelocity() { return -VelocityMotors.velocity(intake); }
    public double getPower() { return intake.getThrottle(); }
    public boolean isStalled() { return stalled; }
    public String getStatus() {
        return stalled ? "Full / stalled" : targetVelocity > 0 ? "Intaking"
                : targetVelocity < 0 ? "Reverse" : "Off";
    }

    public void stopImmediately() {
        requestedVelocity = 0;
        targetVelocity = 0;
        stalled = false;
        slowSinceNanos = Long.MIN_VALUE;
        VelocityMotors.stop(intake);
    }
}
