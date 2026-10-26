package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.IntakeAndParkPaths;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.ReturnToShootPaths;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.ShootAndParkPaths;
import org.firstinspires.ftc.teamcode.Robot;

/** Autonomous sequences, separate from coordinates and OpMode lifecycle. */
public class Routines {
    private final AutoCommands commands;

    public Routines(Robot robot, Consumer<String> status) {
        commands = new AutoCommands(robot, status);
    }

    public Command shootAndPark(ShootAndParkPaths paths) {
        return sequential(
                commands.shootPreload(),
                commands.followPath(paths.runToPark, "Run to Park"),
                commands.finish()
        );
    }

    public Command intakeAndPark(IntakeAndParkPaths paths) {
        return sequential(
                commands.shootPreload(),
                commands.followPath(paths.goToIntakeCorner, "Go to Intake Corner"),
                commands.followPath(paths.intakeCorner, "Intake Corner"),
                commands.followPath(paths.runToPark, "Run to Park"),
                commands.finish()
        );
    }

    public Command returnToShoot(ReturnToShootPaths paths) {
        return sequential(
                commands.shootPreload(),
                commands.followPath(paths.goToIntakeCorner, "Go to Intake "),
                commands.followPath(paths.intakeCorner, "Intaking "),
                commands.followPath(paths.returnToShootingPose, "Return to Shooting "),
                commands.followPath(paths.runToPark, "Run to Park"),
                commands.finish()
        );
    }
}
