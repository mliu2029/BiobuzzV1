package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Shooter implements Mechanism {

    // TODO: retune this pid or fix autotune
    // Gains use NextMotor default scaling: one radian per encoder tick.
    public static final double DEFAULT_P = 0.01686;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.00116;
    public static final double DEFAULT_F = 0.000507;

    private static final double DEFAULT_TARGET_VELOCITY = 1375;

    private DcMotorEx flywheelMotor;
    private DcMotorEx rollerMotor;
    private final NextMotor flywheel;
    private final NextMotor roller;

    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean velocityGainsValid = true;

    public final Command runFlywheel = instant(() -> setVelocity(targetVelocity));
    public final Command stopFlywheel = instant(this::stopImmediately);

    public Shooter() {
        this(new NextMotor("flywheel"), new NextMotor("rollers"));
    }

    // Lets unit tests supply motors without an active Robot Controller hardware map.
    Shooter(NextMotor flywheel, NextMotor roller) {
        this.flywheel = flywheel;
        this.roller = roller;
    }

    public void initialize(HardwareMap hardwareMap) {
        initializeMotors(hardwareMap.get(DcMotorEx.class, "flywheel"),
                hardwareMap.get(DcMotorEx.class, "rollers"));
    }

    void initializeMotors(DcMotorEx flywheel, DcMotorEx roller) {
        this.flywheelMotor = flywheel;
        this.rollerMotor = roller;
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        flywheel.setPower(0);
        roller.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        roller.setDirection(DcMotor.Direction.FORWARD);
        velocityGainsValid = false;
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
        stopImmediately();
    }

    @Override
    public void periodic() {
        if (!canRun()) {
            stopMotors();
            return;
        }

        double flywheelVelocity = getVelocity1();
        double rollerVelocity = getVelocity2();
        if (!Double.isFinite(flywheelVelocity) || !Double.isFinite(rollerVelocity)) {
            stopMotors();
            return;
        }
        flywheel.setVelocitySetpoint(Units.RadiansPerSecond.of(targetVelocity));
        roller.setVelocitySetpoint(Units.RadiansPerSecond.of(targetVelocity));
    }

    public void adjustVelocity(int change) {
        updateTargetVelocity(targetVelocity + change);
    }

    public double getTargetVelocity() { return targetVelocity; }

    public void setVelocity(double velocity) {
        updateTargetVelocity(velocity);
        boolean shouldRun = targetVelocity > 0;
        if (enabled != shouldRun) {
            resetControllers();
        }
        enabled = shouldRun;
    }

    private void updateTargetVelocity(double velocity) {
        double target = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
        if (targetVelocity != target) {
            targetVelocity = target;
            resetControllers();
        }
        if (target == 0) {
            stopMotors();
        }
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        if (!Double.isFinite(p) || !Double.isFinite(i)
                || !Double.isFinite(d) || !Double.isFinite(f)) {
            velocityGainsValid = false;
            stopMotors();
            return;
        }

        MotorVelocityConstants gains = flywheel.getVelocityConstants();
        if (velocityGainsValid && gains.getKP() == p && gains.getKI() == i
                && gains.getKD() == d && gains.getKV() == f) {
            return;
        }

        for (NextMotor motor : new NextMotor[]{flywheel, roller}) {
            motor.getVelocityConstants().setKP(p);
            motor.getVelocityConstants().setKI(i);
            motor.getVelocityConstants().setKD(d);
            motor.getVelocityConstants().setKV(f);
        }
        resetControllers();
        velocityGainsValid = true;
    }

    public double getVelocity1() { return flywheelMotor == null ? 0 : flywheel.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getVelocity2() { return rollerMotor == null ? 0 : roller.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getPower1() { return flywheelMotor == null ? 0 : flywheelMotor.getPower(); }
    public double getPower2() { return rollerMotor == null ? 0 : rollerMotor.getPower(); }

    public boolean isEnabled() { return enabled; }

    public void stopImmediately() {
        enabled = false;
        stopMotors();
    }

    private boolean canRun() {
        return flywheelMotor != null && rollerMotor != null
                && enabled && velocityGainsValid && targetVelocity > 0;
    }

    private void resetControllers() {
        flywheel.getVelocityPID().reset();
        roller.getVelocityPID().reset();
    }

    // Stop output without disabling the requested speed, so a valid reading can resume control.
    private void stopMotors() {
        resetControllers();
        flywheel.setThrottle(0);
        roller.setThrottle(0);
        if (flywheelMotor != null) {
            flywheel.update();
            flywheelMotor.setPower(0);
        }
        if (rollerMotor != null) {
            roller.update();
            rollerMotor.setPower(0);
        }
    }
}
