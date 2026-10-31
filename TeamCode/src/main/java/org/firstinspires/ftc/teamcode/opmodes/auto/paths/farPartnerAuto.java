package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Supplied single-curve far partner route. */
public class farPartnerAuto {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(64.3683, 134.3288, 270);
    private final Pose path1End = poseFactory.of(9.894, 116.3838, 0);
    private final Pose path1Control = poseFactory.of(46.7638, 114.1626, 0);

    public Path path1() {
        return Paths.curve(startPose, path1Control, path1End)
                .linear(startPose, path1End);
    }
}
