package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

@NextTeleop(name = "SOTM Red", group = "BioBuzz")
public class SotmRed extends SotmTeleOp {
    public SotmRed(Robot robot) { super(robot, Alliance.RED); }
}
