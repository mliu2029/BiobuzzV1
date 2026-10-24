package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

// Temporarily disable Panels registration so it does not load the servo singleton.
// @Configurable
public class Transfer implements Mechanism {
    // TODO: tune positions
    public static volatile double OPEN_POSITION = 0.5;
    public static volatile double BLOCKED_POSITION = 0.5;

    public static final Transfer INSTANCE = new Transfer();
    // Hardware disabled for drivetrain-only testing.
    // private final NextServo transfer = new NextServo("transfer");

    private Transfer() {
    }

    public final Command openBlocker = instant(() -> {
        // transfer.setPosition(OPEN_POSITION);
    });
    public final Command closeBlocker = instant(this::stopImmediately);

    public void stopImmediately() {
        // transfer.setPosition(BLOCKED_POSITION);
    }
}
