package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.feedforward.SimpleFFCoefficients;
import dev.nextftc.control.feedforward.SimpleFeedforward;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
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
    private Gamepad driver;
    private boolean enabled;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean velocityGainsValid = true;

    public void initialize(HardwareMap hardwareMap) {
        initialize(hardwareMap, null);
    }

    public void initialize(HardwareMap hardwareMap, Gamepad driver) {
        initializeMotor(hardwareMap.get(DcMotorEx.class, "launcher"), driver);
    }

    void initializeMotor(DcMotorEx launcher, Gamepad driver) {
        this.shooter = launcher;
        this.driver = driver;
        targetVelocity = DEFAULT_TARGET_VELOCITY;
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPIDFCoefficients(DEFAULT_P, DEFAULT_I, DEFAULT_D, DEFAULT_F);
        stop();
    }

    @Override
    public Command getDefaultCommand() {
        return infinite(() -> {
            if (driver != null) {
                if (driver.dpad_up) {
                    setEnabled(true);
                }
                if (driver.dpad_down) {
                    setEnabled(false);
                }
            }
        });
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
        setEnabled(targetVelocity > 0);
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

    private void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            controller.reset();
        }
        this.enabled = enabled;
        if (!enabled) {
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

    public Command runAtVelocity(double velocity) {
        return instant(() -> setVelocity(velocity));
    }

    public Command stopCommand() {
        return instant(this::stop);
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

    public void stop() {
        setEnabled(false);
        controller.reset();
    }

    public void stopImmediately() {
        stop();
    }
}
