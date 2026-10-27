package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.blueSolo2Tip;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.redSolo2Tip;
import org.firstinspires.ftc.teamcode.Robot;

public class Routines {
    private final AutoCommands commands;

    public Routines(Robot robot, Consumer<String> status) {
        commands = new AutoCommands(robot, status);
    }

    public Command redSolo2Tip(redSolo2Tip paths) {
        return sequential(
                commands.shootPreload(),
                commands.followPath(paths.intakeGarden, "intakeGarden"),
                commands.followPath(paths.shoot1, "shoot1"),
                commands.followPath(paths.flowerPrep1, "flowerPrep1"),
                commands.followPath(paths.flowerIntake1, "flowerIntake1"),
                commands.followPath(paths.flowerPrep2, "flowerPrep2"),
                commands.followPath(paths.flowerIntake2, "flowerIntake2"),
                commands.followPath(paths.shoot2, "shoot2"),
                commands.followPath(paths.park, "park"),
                commands.finish()
        );
    }

    public Command blueSolo2Tip(blueSolo2Tip paths) {
        return sequential(
                commands.shootPreload(),
                commands.followPath(paths.intakeGarden, "intakeGarden"),
                commands.followPath(paths.shoot1, "shoot1"),
                commands.followPath(paths.flowerPrep1, "flowerPrep1"),
                commands.followPath(paths.flowerIntake1, "flowerIntake1"),
                commands.followPath(paths.flowerPrep2, "flowerPrep2"),
                commands.followPath(paths.flowerIntake2, "flowerIntake2"),
                commands.followPath(paths.shoot2, "shoot2"),
                commands.followPath(paths.park, "park"),
                commands.finish()
        );
    }
}
