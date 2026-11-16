package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain implements Mechanism {
    private static final double PATH_TIMEOUT_MS = 5000;

    private Follower follower;

    public void initialize(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        stopImmediately();
    }

    public final Command stopDrivetrain = instant(this::stopImmediately);

    public Command stop() {
        return stopDrivetrain;
    }

    public Command off() {
        return stopDrivetrain;
    }

    public Follower getFollower() {
        return follower;
    }

    public Command drive(Gamepad gamepad) {
        return infinite(() -> {
            follower.manual(-gamepad.right_stick_y, -gamepad.right_stick_x,
                    -gamepad.left_stick_x * 0.75);
            follower.update();
        })
                .setEnd(condition -> stopImmediately());
    }

    public Command followPath(Path path) {
        return PedroCommands.follow(follower, path)
                .setDone(() -> follower.atParametricEnd() && !follower.isBusy())
                .setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) {
                        stopImmediately();
                    }
                })
                .requiring(this)
                .raceWith(Commands.waitMs(PATH_TIMEOUT_MS));
    }

    public void stopImmediately() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop(true);
        }
    }
}
