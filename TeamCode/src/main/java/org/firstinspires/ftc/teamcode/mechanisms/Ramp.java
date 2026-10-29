package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

@Configurable
public class Ramp implements Mechanism {
    public static volatile double ramp1UpPosition = 0.5;
    public static volatile double ramp1DownPosition = 0.5;
    public static volatile double ramp2UpPosition = 0.5;
    public static volatile double ramp2DownPosition = 0.5;
    private final NextServo ramp1 = new NextServo("ramp1");
    private final NextServo ramp2 = new NextServo("ramp2");
    public Ramp() {
    }

    public final Command up = instant(() -> {
        ramp1.setPosition(ramp1UpPosition);
        ramp2.setPosition(ramp2UpPosition);
    });
    public final Command down = instant(() -> {
        ramp1.setPosition(ramp1DownPosition);
        ramp2.setPosition(ramp2DownPosition);
    });


}
