package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Intake implements Mechanism {
    public static final double DEFAULT_P = 0.0000001;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0;
    public static final double DEFAULT_F = 0.000299;
    public static final double DEFAULT_TARGET_VELOCITY = 2750;

    private final NextMotor intake;
    private boolean enabled;
    private boolean velocityGainsValid = true;
    private double targetVelocity;

    public final Command runIntake = instant(() -> setVelocity(DEFAULT_TARGET_VELOCITY));
    public final Command runIntakeReverse = instant(() -> setVelocity(-DEFAULT_TARGET_VELOCITY));
    public final Command stopIntake = instant(this::stopImmediately);

    public Intake() {
        this(new NextMotor("intake"));
    }

    Intake(NextMotor intake) {
        this.intake = intake;
    }

    public void initialize() {
// THE MOTOR IS REVERSE DONT CHANGE IT
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
        stopImmediately();
        velocityGainsValid = false;
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
    }

    public void setVelocity(double velocity) {
        double target = Double.isFinite(velocity) ? velocity : 0;
        boolean targetChanged = targetVelocity != target;
        targetVelocity = target;
        if (target == 0 || !velocityGainsValid) {
            stopImmediately();
            return;
        }
        if (!enabled || targetChanged) intake.getVelocityPID().reset();
        enabled = true;
        intake.setVelocitySetpoint(Units.RadiansPerSecond.of(target));
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        if (!Double.isFinite(p) || !Double.isFinite(i)
                || !Double.isFinite(d) || !Double.isFinite(f)) {
            velocityGainsValid = false;
            stopImmediately();
            return;
        }
        MotorVelocityConstants gains = intake.getVelocityConstants();
        if (velocityGainsValid && gains.getKP() == p && gains.getKI() == i
                && gains.getKD() == d && gains.getKV() == f) {
            return;
        }
        gains.setKP(p);
        gains.setKI(i);
        gains.setKD(d);
        gains.setKV(f);
        intake.getVelocityPID().reset();
        velocityGainsValid = true;
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getVelocity() { return -intake.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getPower() { return Math.max(-1, Math.min(1, intake.getThrottle())); }
    public boolean isEnabled() { return enabled; }

    public void stopImmediately() {
        enabled = false;
        targetVelocity = 0;
        intake.getVelocityPID().reset();
        intake.setThrottle(0.0);
        intake.update();
    }
}
