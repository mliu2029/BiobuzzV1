package org.firstinspires.ftc.teamcode.tests.sotm;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@NextTeleop(name = "SOTM Red", group = "BioBuzz")
public class SotmRed extends SotmTeleOp {
    public SotmRed(Robot robot) { super(robot, Alliance.RED); }
}
