package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import java.util.function.DoubleSupplier;

public class Transfer implements Mechanism {
    private final NextMotor transfer = new NextMotor("transfer");
    private final NextMotor leftTransfer = new NextMotor("transfer2");

    public Transfer() {
        transfer.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        leftTransfer.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command on() {
        return setPower(1.0);
    }

    public Command reverse() {
        return setPower(-1.0);
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
        transfer.setThrottle(power);
        leftTransfer.setThrottle(power);
    }

    public void stopImmediately() {
        applyPower(0.0);
        transfer.update();
        leftTransfer.update();
    }
}
