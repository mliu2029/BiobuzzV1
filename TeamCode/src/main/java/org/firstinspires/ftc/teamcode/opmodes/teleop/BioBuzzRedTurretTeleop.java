package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.BioBuzzTurretTeleop;
import org.firstinspires.ftc.teamcode.util.Calculations.Alliance;

// Temporarily hidden while non-drive hardware is disabled.
// @NextTeleop(name = "red turret teleop", group = "BioBuzz")
public class BioBuzzRedTurretTeleop extends BioBuzzTurretTeleop {
    public BioBuzzRedTurretTeleop(Robot robot) {
        super(robot, Alliance.RED);
    }
}
