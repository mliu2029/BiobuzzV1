package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain implements Mechanism {
    private static final long PATH_TIMEOUT_NANOS = 5_000_000_000L;

    public enum PathResult { NOT_STARTED, RUNNING, COMPLETED, TIMED_OUT, CANCELLED }

    private Follower follower;
    private long pathStartedNanos;
    private PathResult pathResult = PathResult.NOT_STARTED;

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

    public PathResult getLastPathResult() {
        return pathResult;
    }

    public Command followPath(Path path) {
        return PedroCommands.follow(follower, path)
                .setStart(() -> {
                    pathResult = PathResult.RUNNING;
                    pathStartedNanos = System.nanoTime();
                    follower.follow(path);
                })
                .setDone(() -> pathFinished()
                        || System.nanoTime() - pathStartedNanos >= PATH_TIMEOUT_NANOS)
                .setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) {
                        pathResult = PathResult.CANCELLED;
                    } else {
                        pathResult = pathFinished() ? PathResult.COMPLETED : PathResult.TIMED_OUT;
                    }
                    if (pathResult != PathResult.COMPLETED) stopImmediately();
                })
                .requiring(this);
    }

    private boolean pathFinished() {
        return follower.atParametricEnd() && !follower.isBusy();
    }

    public void stopImmediately() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop(true);
        }
    }
}
