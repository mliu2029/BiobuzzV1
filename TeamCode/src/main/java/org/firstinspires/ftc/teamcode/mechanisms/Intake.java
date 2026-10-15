package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    private final NextMotor motor = new NextMotor("intake");

    public Intake() {
        motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Starts the intake; it keeps running until another power command is scheduled. */
    public Command run() {
        return setPower(1.0);
    }

    public Command reverse() {
        return setPower(-1.0);
    }

    public Command setPower(double power) {
        return instant(() -> motor.setThrottle(power));
    }

    public Command stop() {
        return setPower(0.0);
    }

    public void stopImmediately() {
        motor.setThrottle(0.0);
        motor.update();
    }
}
