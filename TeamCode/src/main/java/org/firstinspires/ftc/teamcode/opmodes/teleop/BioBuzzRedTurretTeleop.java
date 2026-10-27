package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

@NextTeleop(name = "red turret teleop", group = "BioBuzz")
public class BioBuzzRedTurretTeleop extends BioBuzzTurretTeleop {
    public BioBuzzRedTurretTeleop(Robot robot) {
        super(robot, Alliance.RED);
    }
}
