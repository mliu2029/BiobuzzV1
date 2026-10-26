package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Poses and path geometry for RedShootAndParkAuto. */
public class ShootAndParkPaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public final Pose startPose = poseFactory.of(55, 9, 90);
    private final Pose park = poseFactory.of(14.2259, 95.9422, 0);
    private final Pose parkControlPose = poseFactory.of(22.7118, 50.0193, 0);

    public final Path runToPark;

    public ShootAndParkPaths() {
        runToPark = curve(startPose, parkControlPose, park).linear(startPose, park);
    }
}
