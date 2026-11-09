package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Far partner route: a single line to park. */
public class farPartnerAuto {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(64.2742, 134.4415, 270);
    private final Pose path1End = poseFactory.of(11.8757, 91.5075, 0);

    public Path path1() {
        return Paths.line(startPose, path1End)
                .linear(startPose, path1End);
    }
}
