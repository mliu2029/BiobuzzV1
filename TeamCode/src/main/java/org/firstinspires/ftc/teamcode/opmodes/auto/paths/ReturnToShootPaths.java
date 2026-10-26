package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Poses and path geometry for RedTestReturnToShootAuto. */
public class ReturnToShootPaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public final Pose startPose = poseFactory.of(55, 9, 90);
    private final Pose intakeCornerStart = poseFactory.of(9.4454, 23.2844, 90);
    private final Pose intakeCornerStartControl = poseFactory.of(47.052, 43.7612, 0);
    private final Pose endIntakeCorner = poseFactory.of(9.2452, 10.5186, 89.1666);
    private final Pose shootPoseControl = poseFactory.of(30.516, 37.0893, 0);
    private final Pose park = poseFactory.of(14.2259, 95.9422, 0);
    private final Pose parkControlPose = poseFactory.of(22.7118, 50.0193, 0);

    public final Path goToIntakeCorner;
    public final Path intakeCorner;
    public final Path returnToShootingPose;
    public final Path runToPark;

    public ReturnToShootPaths() {
        goToIntakeCorner = curve(startPose, intakeCornerStartControl, intakeCornerStart)
                .linear(startPose, intakeCornerStart);
        intakeCorner = line(intakeCornerStart, endIntakeCorner).reverseTangent();
        returnToShootingPose = curve(endIntakeCorner, shootPoseControl, startPose)
                .linear(endIntakeCorner, startPose);
        runToPark = curve(startPose, parkControlPose, park)
                .linear(startPose, park);
    }
}
