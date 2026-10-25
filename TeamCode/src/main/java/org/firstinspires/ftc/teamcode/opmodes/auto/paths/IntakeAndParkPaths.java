package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Poses and path geometry for RedTestAuto. */
public class IntakeAndParkPaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public final Pose start = poseFactory.of(59.0879, 8.1816, 90);
    private final Pose path1 = poseFactory.of(15.6213, 11.8409, 75);
    private final Pose path1Control1 = poseFactory.of(47.052, 43.7612, 0);
    private final Pose point2 = poseFactory.of(9.6085, 10.0635, 75);
    private final Pose point3Start = poseFactory.of(9.6085, 10.0635, 90);
    private final Pose point3 = poseFactory.of(10.3158, 99.4397, 0);
    private final Pose point3Control1 = poseFactory.of(30.6675, 75.482, 0);

    public final Path goToIntakeCorner;
    public final Path intakeCorner;
    public final Path runToPark;

    public IntakeAndParkPaths() {
        goToIntakeCorner = curve(start, path1Control1, path1)
                .linear(start, path1);
        intakeCorner = line(path1, point2).reverseTangent();
        runToPark = curve(point3Start, point3Control1, point3)
                .linear(point3Start, point3);
    }
}
