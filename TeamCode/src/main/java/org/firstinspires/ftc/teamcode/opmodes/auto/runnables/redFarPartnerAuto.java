package org.firstinspires.ftc.teamcode.opmodes.auto.runnables;

import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

@NextAutonomous(name = "red far partner auto", group = "Pedro")
public class redFarPartnerAuto extends BaseAuto {
    private final org.firstinspires.ftc.teamcode.opmodes.auto.paths.farPartnerAuto paths =
            new org.firstinspires.ftc.teamcode.opmodes.auto.paths.farPartnerAuto();

    public redFarPartnerAuto(Robot robot) {
        super(robot);
    }

    @Override
    public void start() {
        startAuto(Alliance.RED, paths.startPose);
        routines.farPartnerAuto(paths).schedule();
    }

}
