package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.Path;
import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.robot.Robot;

/** Reusable autonomous actions using the OpMode's existing robot. */
public class AutoCommands {
    private final Robot robot;
    private final Consumer<String> status;

    public AutoCommands(Robot robot, Consumer<String> status) {
        this.robot = robot;
        this.status = status;
    }

    public Command shootPreload() {
        return sequential(
                Commands.instant(() -> status.accept("Shooting")),
                robot.getShooter().runFlywheel,
                Commands.waitMs(500),
                robot.getIntake().runIntake,
                // Transfer hardware remains disabled.
                Commands.waitMs(5000)
        );
    }

    public Command followPath(Path path, String name) {
        return sequential(
                Commands.instant(() -> status.accept(name)),
                robot.getDrivetrain().followPath(path)
        );
    }

    public Command finish() {
        return sequential(
                robot.stop(),
                Commands.instant(() -> status.accept("Finished"))
        );
    }
}
