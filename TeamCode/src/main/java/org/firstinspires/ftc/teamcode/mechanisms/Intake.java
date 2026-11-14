package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Configurable
public class Intake implements Mechanism {
    public static final double DEFAULT_P = 0.000001;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0;
    public static final double DEFAULT_F = 0.000323;
    public static final double DEFAULT_TARGET_VELOCITY = 2500;

    public static volatile double stallVelocity = 250;
    public static volatile double stallSeconds = 3.0;

    private final NextMotor intake = new NextMotor("intake");
    private double targetVelocity;
    private boolean stalled;
    private long slowSinceNanos;

    public final Command runIntake = instant(this::restart);

    public void restart() {
        stalled = false;
        slowSinceNanos = 0;
        setVelocity(DEFAULT_TARGET_VELOCITY);
    }

    public void initialize() {
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
        stopImmediately();
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
    }

    public void setVelocity(double velocity) {
        if (velocity <= 0) {
            stalled = false;
            slowSinceNanos = 0;
        }
        if (stalled) return;
        targetVelocity = velocity;
        if (velocity == 0) stopImmediately();
        else intake.setVelocitySetpoint(Units.RadiansPerSecond.of(velocity));
    }

    @Override
    public void periodic() {
        if (targetVelocity <= 0 || Math.abs(getVelocity()) >= stallVelocity) {
            slowSinceNanos = 0;
            return;
        }
        long now = System.nanoTime();
        if (slowSinceNanos == 0) slowSinceNanos = now;
        if ((now - slowSinceNanos) / 1e9 >= stallSeconds) {
            stopImmediately();
            stalled = true;
        }
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        intake.getVelocityConstants().withP(p).withI(i).withD(d).withV(f);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getVelocity() { return -intake.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getPower() { return intake.getThrottle(); }
    public String getStatus() {
        return stalled ? "Stalled" : targetVelocity > 0 ? "Intaking" : targetVelocity < 0 ? "Reverse" : "Off";
    }

    public void stopImmediately() {
        targetVelocity = 0;
        stalled = false;
        slowSinceNanos = 0;
        intake.getVelocityPID().reset();
        intake.setThrottle(0);
        intake.update();
    }
}
