package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    private final NextMotor intake = new NextMotor("intake");

    public void initialize() {
        // The SDK resets direction between OpModes, while NextFTC reuses this mechanism.
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
        stopImmediately();
    }

    public final Command runIntake = setPower(0.8);
    public final Command runIntakeReverse = setPower(-1.0);
    public final Command stopIntake = setPower(0.0);

    public Command setPower(double power) {
        return instant(() -> {
            intake.setThrottle(power);
        });
    }

    public void stopImmediately() {
        intake.setThrottle(0.0);
        intake.update();
    }
}
