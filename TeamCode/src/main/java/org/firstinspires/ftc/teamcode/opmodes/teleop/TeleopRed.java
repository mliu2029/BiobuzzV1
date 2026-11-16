package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@NextTeleop(name = "TeleOp Red", group = "BioBuzz")
public class TeleopRed extends MainTeleOp {
    public TeleopRed(Robot robot) {
        super(robot, Alliance.RED);
    }
}
