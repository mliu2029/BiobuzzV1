package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import java.util.function.DoubleSupplier;

public class Transfer implements Mechanism {
    private final NextMotor[] motors = {
            new NextMotor("transfer"),
            new NextMotor("transfer2")
    };

    public Transfer() {
        for (NextMotor motor : motors) {
            motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public Command on() {
        return setPower(0.6);
    }

    public Command reverse() {
        return setPower(-0.6);
    }

    public Command setPower(double power) {
        return instant(() -> applyPower(power));
    }

    public Command run() {
        return run(() -> 0.8);
    }

    public Command run(DoubleSupplier power) {
        return infinite(() -> applyPower(power.getAsDouble()))
                .setEnd(condition -> applyPower(0.0));
    }

    public Command stop() {
        return setPower(0.0);
    }

    private void applyPower(double power) {
        for (NextMotor motor : motors) {
            motor.setThrottle(power);
        }
    }

    public void stopImmediately() {
        applyPower(0.0);
        for (NextMotor motor : motors) {
            motor.update();
        }
    }
}
