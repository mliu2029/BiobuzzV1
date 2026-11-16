package org.firstinspires.ftc.teamcode.tests.sotm;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@NextTeleop(name = "SOTM Blue", group = "BioBuzz")
public class SotmBlue extends SotmTeleOp {
    public SotmBlue(Robot robot) { super(robot, Alliance.BLUE); }
}
