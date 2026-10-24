package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.feedforward.SimpleFFCoefficients;
import dev.nextftc.control.feedforward.SimpleFeedforward;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    // Software gains produce motor power from encoder ticks/second. Tune on the robot.
    public static final double DEFAULT_P = 0.01686;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.00116;
    public static final double DEFAULT_F = 0.000507;

    private static final double DEFAULT_TARGET_VELOCITY = 1375;
    private static final double READY_VELOCITY_MARGIN = 100;

    private final PIDCoefficients gains = new PIDCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D);
    private final PIDController controller = new PIDController(gains);
    // Same gains, separate integral/derivative history for each encoder.
    private final PIDController controller2 = new PIDController(gains);
    private final SimpleFFCoefficients feedforwardGains =
            new SimpleFFCoefficients(0.0, DEFAULT_F, 0.0);
    private final SimpleFeedforward feedforward = new SimpleFeedforward(feedforwardGains);

    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean velocityGainsValid = true;

    public final Command runFlywheel = instant(() -> setVelocity(targetVelocity));
    public final Command stopFlywheel = instant(this::stopImmediately);

    public void initialize(HardwareMap hardwareMap) {
        initializeMotors(hardwareMap.get(DcMotorEx.class, "flywheel"),
                hardwareMap.get(DcMotorEx.class, "rollers"));
    }

    void initializeMotors(DcMotorEx shooter, DcMotorEx shooter2) {
        this.shooter = shooter;
        this.shooter2 = shooter2;
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        shooter.setPower(0);
        shooter2.setPower(0);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reverse the appropriate motor here if the flywheels are mounted opposite ways.
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
        stopImmediately();
    }

    @Override
    public void periodic() {
        if (shooter == null || shooter2 == null) {
            stopMotor();
            return;
        }
        if (!canRun()) {
            stopMotor();
            return;
        }

        double measuredVelocity = getVelocity1();
        double measuredVelocity2 = getVelocity2();
        if (!Double.isFinite(measuredVelocity) || !Double.isFinite(measuredVelocity2)) {
            stopMotor();
            return;
        }
        double feedforwardPower = feedforward.calculate(targetVelocity);
        double power = controller.calculateFromReference(targetVelocity, measuredVelocity)
                + feedforwardPower;
        double power2 = controller2.calculateFromReference(targetVelocity, measuredVelocity2)
                + feedforwardPower;
        if (!Double.isFinite(power) || !Double.isFinite(power2)) {
            stopMotor();
            return;
        }
        shooter.setPower(Math.max(0, Math.min(1, power)));
        shooter2.setPower(Math.max(0, Math.min(1, power2)));
    }

    public void adjustVelocity(int change) {
        updateTargetVelocity(targetVelocity + change);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getMinimumVelocity() {
        return Math.max(0, targetVelocity - READY_VELOCITY_MARGIN);
    }

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
            stopMotor();
        }
    }

    /** Software PID plus velocity feedforward: f is motor power per tick/second. */
    public void setPIDFCoefficients(double p, double i, double d, double f) {
        if (!Double.isFinite(p) || !Double.isFinite(i)
                || !Double.isFinite(d) || !Double.isFinite(f)) {
            velocityGainsValid = false;
            stopMotor();
            return;
        }

        // The tuner sends gains every loop. Keep PID history until they change.
        if (velocityGainsValid && gains.kP == p && gains.kI == i && gains.kD == d
                && feedforwardGains.kV == f) {
            return;
        }

        gains.kP = p;
        gains.kI = i;
        gains.kD = d;
        feedforwardGains.kV = f;
        resetControllers();
        velocityGainsValid = true;
    }

    public Command runFlywheel(double velocity) {
        return instant(() -> setVelocity(velocity));
    }

    /** Mean measured motor velocity in encoder ticks/second. */
    public double getVelocity() {
        return (getVelocity1() + getVelocity2()) / 2.0;
    }

    /** Mean motor power; use the individual getters when tuning. */
    public double getPower() {
        return (getPower1() + getPower2()) / 2.0;
    }

    public double getVelocity1() { return shooter == null ? 0 : shooter.getVelocity(); }
    public double getVelocity2() { return shooter2 == null ? 0 : shooter2.getVelocity(); }
    public double getPower1() { return shooter == null ? 0 : shooter.getPower(); }
    public double getPower2() { return shooter2 == null ? 0 : shooter2.getPower(); }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isReady() {
        if (!canRun()) {
            return false;
        }
        double velocity = getVelocity1();
        double velocity2 = getVelocity2();
        return Double.isFinite(velocity) && Double.isFinite(velocity2)
                && velocity >= getMinimumVelocity() && velocity2 >= getMinimumVelocity();
    }

    public void stopImmediately() {
        enabled = false;
        stopMotor();
    }

    private boolean canRun() {
        return shooter != null && shooter2 != null
                && enabled && velocityGainsValid && targetVelocity > 0;
    }

    private void resetControllers() {
        controller.reset();
        controller2.reset();
    }

    // Stop output without disabling the requested speed, so a valid reading can resume control.
    private void stopMotor() {
        resetControllers();
        if (shooter != null) shooter.setPower(0);
        if (shooter2 != null) shooter2.setPower(0);
    }
}
