package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain implements Mechanism {
    public Drivetrain() { }
    private Follower follower;

    public void initialize(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        stopImmediately();
    }

    public Follower getFollower() {
        return follower;
    }

    public Command drive(Gamepad gamepad) {
        return infinite(() -> {
            follower.manual(-gamepad.right_stick_y, -gamepad.right_stick_x,
                    -gamepad.left_stick_x);
            follower.update();
        })
                .setEnd(condition -> stopImmediately());
    }
    public Command relocalize() {
        return Commands.instant(() -> follower.setPose(new Pose(8, 8, 0)));
    }

    public Command followPath(Path path) {
        return PedroCommands.follow(follower, path)
                .setDone(() -> follower.atParametricEnd() && !follower.isBusy())
                .requiring(this);
    }

    public void stopImmediately() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop(true);
        }
    }
}
