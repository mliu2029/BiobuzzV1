package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.BioBuzzTurretTeleop;
import org.firstinspires.ftc.teamcode.util.Calculations.Alliance;

@NextTeleop(name = "blue turret teleop", group = "BioBuzz")
public class BioBuzzBlueTurretTeleop extends BioBuzzTurretTeleop {
    public BioBuzzBlueTurretTeleop(Robot robot) {
        super(robot, Alliance.BLUE);
    }
}
