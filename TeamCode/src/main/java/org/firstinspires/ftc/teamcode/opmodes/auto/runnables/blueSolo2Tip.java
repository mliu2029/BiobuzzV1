package org.firstinspires.ftc.teamcode.opmodes.auto.runnables;

import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

/** Runs the red solo sequence mirrored across both field centerlines. */
@NextAutonomous(name = "REAL AUTO", group = "Pedro")
public class blueSolo2Tip extends BaseAuto {
    private final org.firstinspires.ftc.teamcode.opmodes.auto.paths.blueSolo2Tip paths = new org.firstinspires.ftc.teamcode.opmodes.auto.paths.blueSolo2Tip();

    public blueSolo2Tip(Robot robot) {
        super(robot);
    }

    @Override
    public void start() {
        startAuto(Alliance.BLUE, paths.startPose);
        routines.blueSolo2Tip(paths).schedule();
    }

}
