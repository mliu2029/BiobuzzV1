package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public static final double DEFAULT_P = 0.00998;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.00037;
    public static final double DEFAULT_F = 0.000409;

    private static final double DEFAULT_TARGET_VELOCITY = 1375;
    private static final double READY_VELOCITY_MARGIN = 100;

    private final PIDCoefficients gains = new PIDCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D);
    private final PIDController controller = new PIDController(gains);
    private final SimpleFFCoefficients feedforwardGains =
            new SimpleFFCoefficients(0.0, DEFAULT_F, 0.0);
    private final SimpleFeedforward feedforward = new SimpleFeedforward(feedforwardGains);

    private DcMotorEx flywheel;
    // Rollers are configured but not controlled yet.
    private DcMotorEx rollers;
    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean velocityGainsValid = true;

    public final Command runFlywheel = instant(() -> setVelocity(targetVelocity));
    public final Command stopFlywheel = instant(this::stopImmediately);

    public void initialize(HardwareMap hardwareMap) {
        initializeMotors(hardwareMap.get(DcMotorEx.class, "flywheel"),
                hardwareMap.get(DcMotorEx.class, "rollers"));
    }

    void initializeMotors(DcMotorEx flywheel, DcMotorEx rollers) {
        this.flywheel = flywheel;
        this.rollers = rollers;
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        flywheel.setPower(0);
        rollers.setPower(0);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        rollers.setDirection(DcMotor.Direction.FORWARD);
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
        stopImmediately();
    }

    @Override
    public void periodic() {
        if (!canRun()) {
            stopMotor();
            return;
        }

        double measuredVelocity = getVelocity();
        if (!Double.isFinite(measuredVelocity)) {
            stopMotor();
            return;
        }
        double power = controller.calculateFromReference(targetVelocity, measuredVelocity)
                + feedforward.calculate(targetVelocity);
        if (!Double.isFinite(power)) {
            stopMotor();
            return;
        }
        flywheel.setPower(Math.max(0, Math.min(1, power)));
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

    /** Measured flywheel velocity in encoder ticks/second. */
    public double getVelocity() {
        return flywheel == null ? 0 : flywheel.getVelocity();
    }

    public double getPower() {
        return flywheel == null ? 0 : flywheel.getPower();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isReady() {
        if (!canRun()) {
            return false;
        }
        double velocity = getVelocity();
        return Double.isFinite(velocity) && velocity >= getMinimumVelocity();
    }

    public void stopImmediately() {
        enabled = false;
        stopMotor();
    }

    private boolean canRun() {
        return flywheel != null && enabled && velocityGainsValid && targetVelocity > 0;
    }

    private void resetControllers() {
        controller.reset();
    }

    // Stop output without disabling the requested speed, so a valid reading can resume control.
    private void stopMotor() {
        resetControllers();
        if (flywheel != null) flywheel.setPower(0);
    }
}
