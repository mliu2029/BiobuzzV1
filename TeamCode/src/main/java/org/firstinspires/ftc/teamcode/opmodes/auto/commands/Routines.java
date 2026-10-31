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
                commands.shoot(),
                robot.Ramp().down,
                commands.followPath(paths.PrepFlowerIntake1(), "PrepFlowerIntake1"),
                commands.followPath(paths.FlowerIntake1(), "FlowerIntake1"),
                Commands.waitMs(1000),
                commands.followPath(paths.returnToShootFar(), "returnToShootFar"),
                commands.shoot(),
                commands.followPath(paths.PrepFlowerIntake2(), "PrepFlowerIntake2"),
                commands.followPath(paths.FlowerIntake2(), "FlowerIntake2"),
                Commands.waitMs(1000),
                commands.followPath(paths.shootClose1(), "shootClose1"),
                commands.shoot(),
                commands.followPath(paths.prepGardenIntake(), "prepGardenIntake"),
                commands.followPath(paths.gardenIntake(), "gardenIntake"),
                Commands.waitMs(250),
                commands.followPath(paths.shootClose2(), "shootClose2"),
                commands.shoot(),
                commands.followPath(paths.park(), "park"),
                commands.finish()
        );
    }

    public Command redPartnerAuto(redPartnerAuto paths) {
        return sequential(
                commands.init(),
                commands.shoot(),
                robot.Ramp().down,
                commands.followPath(paths.PrepFlowerIntake1(), "PrepFlowerIntake1"),
                commands.followPath(paths.FlowerIntake1(), "FlowerIntake1"),
                Commands.waitMs(1000),
                commands.followPath(paths.returnToShootFar(), "returnToShootFar"),
                commands.shoot(),
                commands.followPath(paths.PrepFlowerIntake2(), "PrepFlowerIntake2"),
                commands.followPath(paths.FlowerIntake2(), "FlowerIntake2"),
                Commands.waitMs(1000),
                commands.followPath(paths.shootClose1(), "shootClose1"),
                commands.shoot(),
                commands.followPath(paths.prepGardenIntake(), "prepGardenIntake"),
                commands.followPath(paths.gardenIntake(), "gardenIntake"),
                Commands.waitMs(250),
                commands.followPath(paths.shootClose2(), "shootClose2"),
                commands.shoot(),
                commands.followPath(paths.park(), "park"),
                commands.finish()
        );  
        
    }

    public Command redSolo2Tip(redSolo2Tip paths) {
        return sequential(
                commands.init(),
                Commands.waitMs(1500),

                commands.shoot(),
                commands.followPath(paths.gardenPrep, "gardenPrep"),
                commands.followPath(paths.intakeGarden, "intakeGarden"),
                Commands.waitMs(2000),

                //commands.followPath(paths.shoot1, "shoot1"),
                //commands.shoot(),
                //robot.Ramp().down,
                //commands.followPath(paths.flowerPrep1, "flowerPrep1"),
                //commands.followPath(paths.flowerIntake1, "flowerIntake1"),
                //Commands.waitMs(1500),
                //commands.followPath(paths.returnToShootFar, "returnToShootFar"),
                //commands.shoot(),
                //commands.followPath(paths.flowerPrep2, "flowerPrep2"),
                //commands.followPath(paths.flowerIntake2, "flowerIntake2"),
                //Commands.waitMs(1500),
                //commands.followPath(paths.shoot2, "shoot2"),
               // commands.shoot(),
                commands.followPath(paths.park, "park"),
                commands.finish()
        );
    }

    public Command blueSolo2Tip(blueSolo2Tip paths) {
        return sequential(
                commands.init(),
                Commands.waitMs(1500),

                commands.shoot(),
                commands.followPath(paths.gardenPrep, "gardenPrep"),
                commands.followPath(paths.intakeGarden, "intakeGarden"),
                //commands.followPath(paths.shoot1, "shoot1"),
                //commands.shoot(),
                //robot.Ramp().down,
                //commands.followPath(paths.flowerPrep1, "flowerPrep1"),
                //commands.followPath(paths.flowerIntake1, "flowerIntake1"),
                //Commands.waitMs(1500),
                //commands.followPath(paths.returnToShootFar, "returnToShootFar"),
                //commands.shoot(),
                //commands.followPath(paths.flowerPrep2, "flowerPrep2"),
                //commands.followPath(paths.flowerIntake2, "flowerIntake2"),
                //Commands.waitMs(1500),
                //commands.followPath(paths.shoot2, "shoot2"),
                //commands.shoot(),
                commands.followPath(paths.park, "park"),
                commands.finish()
        );
    }
}
