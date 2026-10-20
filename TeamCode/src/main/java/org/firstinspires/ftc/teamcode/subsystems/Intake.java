package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    public static final Intake INSTANCE = new Intake();
    private final NextMotor intake = new NextMotor("intake");

    private Intake() {
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    public final Command runIntake = setPower(1.0);
    public final Command runIntakeReverse = setPower(-1.0);
    public final Command stopIntake = setPower(0.0);

    public Command setPower(double power) {
        return instant(() -> intake.setThrottle(power));
    }

    public void stopImmediately() {
        intake.setThrottle(0.0);
        intake.update();
    }
}
