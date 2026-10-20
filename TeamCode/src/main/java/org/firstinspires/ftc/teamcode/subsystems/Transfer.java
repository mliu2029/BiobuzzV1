package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {
    public static final Transfer INSTANCE = new Transfer();
    private final NextMotor transfer = new NextMotor("transfer");

    private Transfer() {
        transfer.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    public final Command stopTransfer = runTransfer(0.0);

    public Command runTransfer(double power) {
        return instant(() -> transfer.setThrottle(power));
    }

    public void stopImmediately() {
        transfer.setThrottle(0.0);
        transfer.update();
    }
}
