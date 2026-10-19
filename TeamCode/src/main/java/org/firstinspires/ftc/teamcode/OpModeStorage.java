package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/** Retains the autonomous pose until the Robot Controller process restarts. */
public final class OpModeStorage {
    private OpModeStorage() { }

    public static Pose autonomousEndPose = new Pose(0, 0, 0);
}
