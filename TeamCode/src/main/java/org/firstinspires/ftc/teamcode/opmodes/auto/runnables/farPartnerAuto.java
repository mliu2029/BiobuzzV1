package org.firstinspires.ftc.teamcode.opmodes.auto.runnables;

import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

@NextAutonomous(name = "BlueFarPartnerAuto", group = "Pedro")
public class farPartnerAuto extends BaseAuto {
    private final org.firstinspires.ftc.teamcode.opmodes.auto.paths.farPartnerAuto paths =
            new org.firstinspires.ftc.teamcode.opmodes.auto.paths.farPartnerAuto();

    public farPartnerAuto(Robot robot) {
        super(robot);
    }

    @Override
    public void start() {
        startAuto(Alliance.BLUE, paths.startPose);
        routines.farPartnerAuto(paths).schedule();
    }

}
