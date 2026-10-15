package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain implements Mechanism {
    private Follower follower;

    public void initialize(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        stopImmediately();
    }

    public Follower getFollower() {
        return follower;
    }

    public Command drive(DoubleSupplier forward, DoubleSupplier strafe,
                         DoubleSupplier rotate, double driveScale) {
        return infinite(() -> applyDrive(forward.getAsDouble(), strafe.getAsDouble(),
                rotate.getAsDouble(), driveScale))
                .setEnd(condition -> stopImmediately());
    }

    private void applyDrive(double forward, double strafe, double rotate, double driveScale) {
        double maximum = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate));
        double scale = driveScale / maximum;
        follower.manual(forward * scale, -strafe * scale, -rotate * scale);
        follower.update();
    }

    public Command stop() {
        return instant(this::stopImmediately);
    }

    public void stopImmediately() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop(true);
        }
    }
}
