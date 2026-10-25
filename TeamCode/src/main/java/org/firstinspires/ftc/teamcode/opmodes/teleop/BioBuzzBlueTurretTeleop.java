package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

@NextTeleop(name = "blue turret teleop", group = "BioBuzz")
public class BioBuzzBlueTurretTeleop extends BioBuzzTurretTeleop {
    public BioBuzzBlueTurretTeleop(Robot robot) {
        super(robot, Alliance.BLUE);
    }
}
