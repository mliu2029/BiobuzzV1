package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    private DcMotorEx launcher;

    private Gamepad driver;
    private boolean enabled;
    private double targetVelocity = 1375;
    private double minimumVelocity = 1275;
    private PIDFCoefficients appliedGains;
    private boolean velocityGainsValid = true;

    public void initialize(HardwareMap hardwareMap) {
        initialize(hardwareMap, null);
    }

    public void initialize(HardwareMap hardwareMap, Gamepad driver) {
        this.driver = driver;
        targetVelocity = 1400;
        minimumVelocity = 1300;
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        appliedGains = new PIDFCoefficients(40, 0, 0, 12.5);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, appliedGains);
        velocityGainsValid = true;

        stop();
    }

    @Override
    public Command getDefaultCommand() {
        return infinite(() -> {
            if (driver != null) {
                if (driver.dpad_up) enabled = true;
                if (driver.dpad_down) enabled = false;
            }
            applyVelocity();
        });
    }

    public void adjustVelocity(int change) {
        updateTargetVelocity(targetVelocity + change);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getMinimumVelocity() {
        return minimumVelocity;
    }

    /** Direct velocity control for the Panels tuner; zero stops the shooter. */
    public void setVelocity(double velocity) {
        updateTargetVelocity(velocity);
        enabled = targetVelocity > 0;
        applyVelocity();
    }

    private void updateTargetVelocity(double velocity) {
        targetVelocity = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
        minimumVelocity = Math.max(0, targetVelocity - 100);
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        velocityGainsValid = Double.isFinite(p) && Double.isFinite(i)
                && Double.isFinite(d) && Double.isFinite(f);
        if (!velocityGainsValid) {
            launcher.setVelocity(0);
            return;
        }
        if (appliedGains == null || appliedGains.p != p || appliedGains.i != i
                || appliedGains.d != d || appliedGains.f != f) {
            PIDFCoefficients gains = new PIDFCoefficients(p, i, d, f);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
            appliedGains = gains;
        }
    }

    private void applyVelocity() {
        launcher.setVelocity(enabled && velocityGainsValid ? targetVelocity : 0);
    }

    public Command runAtVelocity(double velocity) {
        return instant(() -> setVelocity(velocity));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public double getVelocity() {
        return launcher.getVelocity();
    }

    public double getPower() {
        return launcher.getPower();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isReady() {
        return enabled && velocityGainsValid && targetVelocity > 0
                && getVelocity() >= minimumVelocity;
    }

    public void stop() {
        enabled = false;
        launcher.setVelocity(0);
    }

    public void stopImmediately() {
        stop();
    }
}
