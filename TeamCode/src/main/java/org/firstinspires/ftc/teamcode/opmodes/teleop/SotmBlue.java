package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

@NextTeleop(name = "SOTM Blue", group = "BioBuzz")
public class SotmBlue extends SotmTeleOp {
    public SotmBlue(Robot robot) { super(robot, Alliance.BLUE); }
}
