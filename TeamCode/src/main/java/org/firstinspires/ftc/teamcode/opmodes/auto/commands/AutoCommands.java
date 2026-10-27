package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.Path;
import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.Robot;

/** Reusable autonomous actions using the OpMode's existing robot. */
public class AutoCommands {
    private final Robot robot;
    private final Consumer<String> status;

    public AutoCommands(Robot robot, Consumer<String> status) {
        this.robot = robot;
        this.status = status;
    }

    /** Startup actions, executed by the routine after the OpMode's START callback. */
    public Command init() {
        return sequential(
                Commands.instant(() -> status.accept("Starting")),
                robot.Shooter().runFlywheel,
                robot.Transfer().openBlocker,
                robot.Ramp().up,
                robot.Turret().aimTurret,
                robot.Intake().runIntake
        );
    }

    public Command followPath(Path path, String name) {
        return sequential(
                Commands.instant(() -> status.accept(name)),
                robot.Drivetrain().followPath(path)
        );
    }

    public Command finish() {
        return sequential(
                robot.stop(),
                Commands.instant(() -> status.accept("Finished"))
        );
    }
}
