package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain implements Mechanism {
    public static final Drivetrain INSTANCE = new Drivetrain();
    private Drivetrain() { }
    private Follower follower;

    public void initialize(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        stopImmediately();
    }

    public Follower getFollower() {
        return follower;
    }

    public Command drive(DoubleSupplier forward, DoubleSupplier strafe,
                         DoubleSupplier rotate) {
        return infinite(() -> applyDrive(forward.getAsDouble(), strafe.getAsDouble(),
                rotate.getAsDouble()))
                .setEnd(condition -> stopImmediately());
    }

    private void applyDrive(double forward, double strafe, double rotate) {
        double maximum = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate));
        double scale = 1.0 / maximum;
        follower.manual(forward * scale, -strafe * scale, -rotate * scale);
        follower.update();
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
