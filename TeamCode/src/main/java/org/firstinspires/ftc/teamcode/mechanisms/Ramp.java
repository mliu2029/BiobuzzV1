package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Ramp implements Mechanism {
    public static volatile double upPosition = 0.5;

    public static volatile double downPosition = 0.5;
//    private final NextServo ramp = new NextServo("ramp");
    public Ramp() {
    }

    public final Command up = instant(() -> {
//        ramp.setPosition(upPosition);
    });
    public final Command down = instant(()->{
//        ramp.setPosition(downPosition);
    });


}
