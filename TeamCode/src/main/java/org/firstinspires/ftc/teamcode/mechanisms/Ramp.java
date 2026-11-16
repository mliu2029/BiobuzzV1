package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

@Configurable
public class Ramp implements Mechanism {
    public static volatile double ramp1UpPosition = 0.2;
    public static volatile double ramp1DownPosition = 0.95;
    public static volatile double ramp2UpPosition = 0.8;
    public static volatile double ramp2DownPosition = 0.05;
    private final NextServo ramp1 = new NextServo("ramp1");
    private final NextServo ramp2 = new NextServo("ramp2");
    private Boolean raised;

    public final Command up = instant(() -> setRaised(true));
    public final Command down = instant(() -> setRaised(false));
    // With no prior ramp command, the first press raises it.
    public final Command toggle = instant(() -> setRaised(!Boolean.TRUE.equals(raised)));

    private void setRaised(boolean raised) {
        ramp1.setPosition(raised ? ramp1UpPosition : ramp1DownPosition);
        ramp2.setPosition(raised ? ramp2UpPosition : ramp2DownPosition);
        this.raised = raised;
    }

    public String getStatus() {
        return raised == null ? "Not commanded" : raised ? "Up" : "Down";
    }
}
