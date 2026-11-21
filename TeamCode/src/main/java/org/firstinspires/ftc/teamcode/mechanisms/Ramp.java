package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

@Configurable
public class Ramp implements Mechanism {
    public static volatile double ramp1UpPosition = 0.7;
    public static volatile double ramp1DownPosition = 0.03;
    public static volatile double ramp2UpPosition = 0.3;
    public static volatile double ramp2DownPosition = 0.97;
    private final NextServo ramp1 = new NextServo("ramp1");
    private final NextServo ramp2 = new NextServo("ramp2");
    private boolean raised = false;

    public final Command up = instant(() -> setRaised(true));
    public final Command down = instant(() -> setRaised(false));
    public final Command toggle = instant(() -> setRaised(!raised));

    public void setRaised(boolean raised) {
        ramp1.setPosition(raised ? ramp1UpPosition : ramp1DownPosition);
        ramp2.setPosition(raised ? ramp2UpPosition : ramp2DownPosition);
        this.raised = raised;
    }

    public boolean isRaised() {
        return raised;
    }

    public String getStatus() {
        return raised ? "Up" : "Down";
    }

    public void initialize() {
        raised = false;
    }

    public void stopImmediately() {
    }
}
