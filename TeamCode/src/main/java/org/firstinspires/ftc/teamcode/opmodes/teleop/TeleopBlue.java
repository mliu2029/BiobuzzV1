package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@NextTeleop(name = "TeleOp Blue", group = "BioBuzz")
public class TeleopBlue extends mainTeleOp {
    public TeleopBlue(Robot robot) {
        super(robot, Alliance.BLUE);
    }
}
