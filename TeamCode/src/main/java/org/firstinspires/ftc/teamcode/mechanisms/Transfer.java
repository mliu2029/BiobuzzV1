package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.robot.Mechanism;

// Temporarily disable Panels registration while transfer hardware is disabled.
// @Configurable
public class Transfer implements Mechanism {
    // TODO: tune positions
    public static volatile double openPosition = 0.5;

    public static volatile double BLOCKED_POSITION = 0.5;

    // Hardware disabled for drivetrain-only testing.
    // private final NextServo transfer = new NextServo("transfer");

    public Transfer() {
    }

    public final Command openBlocker = instant(() -> {
        // transfer.setPosition(OPEN_POSITION);
    });
    public final Command closeBlocker = instant(this::stopImmediately);

    public void stopImmediately() {
        // transfer.setPosition(BLOCKED_POSITION);
    }
}
