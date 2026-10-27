package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.bluePartnerAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.blueSolo2Tip;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.redPartnerAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.redSolo2Tip;
import org.firstinspires.ftc.teamcode.Robot;

public class Routines {
    private final Robot robot;
    private final AutoCommands commands;

    public Routines(Robot robot, Consumer<String> status) {
        this.robot = robot;
        commands = new AutoCommands(robot, status);
    }

    public Command bluePartnerAuto(bluePartnerAuto paths) {
        return sequential(
                commands.init(),
                commands.followPath(paths.path1(), "path1"),
                commands.followPath(paths.path2(), "path2"),
                commands.followPath(paths.path3(), "path3"),
                commands.followPath(paths.path4(), "path4"),
                commands.followPath(paths.path5(), "path5"),
                commands.followPath(paths.path6(), "path6"),
                commands.followPath(paths.path7(), "path7"),
                commands.followPath(paths.path8(), "path8"),
                commands.finish()
        );
    }

    public Command redPartnerAuto(redPartnerAuto paths) {
        return sequential(
                commands.init(),
                commands.followPath(paths.path1(), "path1"),
                commands.followPath(paths.path2(), "path2"),
                commands.followPath(paths.path3(), "path3"),
                commands.followPath(paths.path4(), "path4"),
                commands.followPath(paths.path5(), "path5"),
                commands.followPath(paths.path6(), "path6"),
                commands.followPath(paths.path7(), "path7"),
                commands.followPath(paths.path8(), "path8"),
                commands.finish()
        );
    }

    public Command redSolo2Tip(redSolo2Tip paths) {
        return sequential(
                commands.init(),
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
                commands.init(),
                commands.followPath(paths.intakeGarden, "intakeGarden"),
                commands.followPath(paths.shoot1, "shoot1"),
                robot.Ramp().down,
                commands.followPath(paths.flowerPrep1, "flowerPrep1"),
                commands.followPath(paths.flowerIntake1, "flowerIntake1"),
                Commands.waitMs(500),
                commands.followPath(paths.flowerPrep2, "flowerPrep2"),
                commands.followPath(paths.flowerIntake2, "flowerIntake2"),
                commands.followPath(paths.shoot2, "shoot2"),
                commands.followPath(paths.park, "park"),
                commands.finish()
        );
    }
}
