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

    public Command init() {
        return sequential(
                Commands.instant(() -> status.accept("Starting")),
                robot.Shooter().runFlywheel,
                robot.Transfer().feed,
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

    /** Moves to a robot-relative angle and waits until aligned, then holds it. */
    public Command moveTurret(double degrees) {
        return sequential(
                Commands.instant(() -> status.accept("Turret to " + degrees + " degrees")),
                robot.Turret().moveToDegrees(degrees)
        );
    }

    public Command finish() {
        return sequential(
                robot.stop(),
                Commands.instant(() -> status.accept("Finished"))
        );
    }
}
