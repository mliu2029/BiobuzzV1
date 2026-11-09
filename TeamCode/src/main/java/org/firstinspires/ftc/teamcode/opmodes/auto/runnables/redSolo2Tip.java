package org.firstinspires.ftc.teamcode.opmodes.auto.runnables;

import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

@NextAutonomous(name = "Red Solo (2 Tip)", group = "Pedro")
public class redSolo2Tip extends BaseAuto {
    private final org.firstinspires.ftc.teamcode.opmodes.auto.paths.redSolo2Tip paths = new org.firstinspires.ftc.teamcode.opmodes.auto.paths.redSolo2Tip();

    public redSolo2Tip(Robot robot) {
        super(robot);
    }

    @Override
    public void start() {
        startAuto(Alliance.RED, paths.startPose);
        routines.redSolo2Tip(paths).schedule();
    }

}
