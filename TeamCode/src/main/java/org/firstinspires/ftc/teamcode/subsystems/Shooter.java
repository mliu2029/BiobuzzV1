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
    public static final double DEFAULT_P = 0.0;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0;
    public static final double DEFAULT_F = 0.0;

    private static final double DEFAULT_TARGET_VELOCITY = 1375;
    private static final double READY_VELOCITY_MARGIN = 100;

    private final PIDCoefficients gains = new PIDCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D);
    private final PIDController controller = new PIDController(gains);
    private final SimpleFFCoefficients feedforwardGains =
            new SimpleFFCoefficients(0.0, DEFAULT_F, 0.0);
    private final SimpleFeedforward feedforward = new SimpleFeedforward(feedforwardGains);

    private DcMotorEx shooter;
    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean velocityGainsValid = true;

    public final Command runFlywheel = instant(() -> setVelocity(targetVelocity));
    public final Command stopFlywheel = instant(this::stopImmediately);

    public void initialize(HardwareMap hardwareMap) {
        initializeMotor(hardwareMap.get(DcMotorEx.class, "shooter"));
    }

    void initializeMotor(DcMotorEx shooter) {
        this.shooter = shooter;
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
        stopImmediately();
    }

    @Override
    public void periodic() {
        if (shooter == null) {
            return;
        }
        if (!enabled || !velocityGainsValid || targetVelocity <= 0) {
            controller.reset();
            shooter.setPower(0);
            return;
        }

        double measuredVelocity = getVelocity();
        if (!Double.isFinite(measuredVelocity)) {
            controller.reset();
            shooter.setPower(0);
            return;
        }
        double power = controller.calculateFromReference(targetVelocity, measuredVelocity)
                + feedforward.calculate(targetVelocity);
        shooter.setPower(Double.isFinite(power) ? Math.max(0, Math.min(1, power)) : 0);
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
            controller.reset();
        }
        enabled = shouldRun;
    }

    private void updateTargetVelocity(double velocity) {
        double target = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
        if (targetVelocity != target) {
            targetVelocity = target;
            controller.reset();
        }
        if (target == 0) {
            shooter.setPower(0);
        }
    }

    /** Software PID plus velocity feedforward: f is motor power per tick/second. */
    public void setPIDFCoefficients(double p, double i, double d, double f) {
        boolean valid = Double.isFinite(p) && Double.isFinite(i)
                && Double.isFinite(d) && Double.isFinite(f);
        if (!valid) {
            velocityGainsValid = false;
            controller.reset();
            shooter.setPower(0);
            return;
        }
        if (!velocityGainsValid || gains.kP != p || gains.kI != i || gains.kD != d
                || feedforwardGains.kV != f) {
            gains.kP = p;
            gains.kI = i;
            gains.kD = d;
            feedforwardGains.kV = f;
            controller.reset();
        }
        velocityGainsValid = true;
    }

    public Command runFlywheel(double velocity) {
        return instant(() -> setVelocity(velocity));
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    public double getPower() {
        return shooter.getPower();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isReady() {
        if (!enabled || !velocityGainsValid || targetVelocity <= 0) {
            return false;
        }
        double velocity = getVelocity();
        return Double.isFinite(velocity) && velocity >= getMinimumVelocity();
    }

    public void stopImmediately() {
        enabled = false;
        controller.reset();
        shooter.setPower(0);
    }
}
