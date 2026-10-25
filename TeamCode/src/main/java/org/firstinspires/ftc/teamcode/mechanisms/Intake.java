package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    private final NextMotor intake = new NextMotor("intake");

    public Intake() {
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
        // TODO: i have no fucking clue which direction this is supposed to be
    }

    public final Command runIntake = setPower(1.0);
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
