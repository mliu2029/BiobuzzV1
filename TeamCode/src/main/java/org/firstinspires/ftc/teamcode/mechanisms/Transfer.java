package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

@Configurable
public class Transfer implements Mechanism {
    public static volatile double openPosition = 0.2;
    public static volatile double blockedPosition = 0.75;

    private final NextServo transfer;

    public final Command openBlocker = instant(this::openImmediately);
    public final Command closeBlocker = instant(this::closeImmediately);

    public Transfer() {
        this(new NextServo("transfer"));
    }

    Transfer(NextServo transfer) {
        this.transfer = transfer;
    }

    public void initialize() {
        transfer.setDirection(NextMotor.Direction.FORWARD);
        closeImmediately();
    }

    public double getPosition() { return transfer.getPosition(); }

    public void openImmediately() {
        transfer.setPosition(openPosition);
    }

    public void closeImmediately() {
        transfer.setPosition(blockedPosition);
    }
}
