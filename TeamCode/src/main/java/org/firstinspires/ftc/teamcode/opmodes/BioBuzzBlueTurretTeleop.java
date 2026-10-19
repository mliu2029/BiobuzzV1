package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.aiming.TurretCalculations.Alliance;

@NextTeleop(name = "blue turret teleop", group = "BioBuzz")
public class BioBuzzBlueTurretTeleop extends BioBuzzTurretTeleop {
    public BioBuzzBlueTurretTeleop(Robot robot) {
        super(robot, Alliance.BLUE);
    }
}
