package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.Path;
import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.Robot;
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
                robot.Ramp().up,
                robot.Turret().aimTurret,
                robot.Intake().runIntake
        );
    }
    public Command shoot(){
        return sequential(
                Commands.instant(() -> status.accept("Preparing to shoot")),
                Commands.waitMs(1000),
                Commands.instant(()->status.accept("Shooting")),
                robot.Transfer().feed,
                Commands.waitMs(1000),
                robot.Transfer().stopFeeding

        );

    }
    public Command followPath(Path path, String name) {
        return sequential(
                Commands.instant(() -> status.accept(name)),
                robot.Drivetrain().followPath(path)
        );
    }

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
