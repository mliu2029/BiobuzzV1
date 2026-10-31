package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Supplied three-segment red 1-tip route. */
public class Red1Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(53.0036, 6.0788, 90);
    private final Pose gardenPrepEnd = poseFactory.of(6.5879, 16.2961, 270);
    private final Pose gardenPrepControl = poseFactory.of(30.7223, 33.5367, 0);
    private final Pose intakeGardenEnd = poseFactory.of(6.469, 7.35, 270);
    private final Pose parkStart = poseFactory.of(6.469, 7.35, 270);
    private final Pose parkEnd = poseFactory.of(11.9839, 116.3157, 0);
    private final Pose parkControl = poseFactory.of(40.278, 111.0821, 0);

    public final Path gardenPrep;
    public final Path intakeGarden;
    public final Path park;

    public Red1Tip() {
        gardenPrep = Paths.curve(startPose, gardenPrepControl, gardenPrepEnd)
                .linear(startPose, gardenPrepEnd);
        intakeGarden = Paths.line(gardenPrepEnd, intakeGardenEnd)
                .constant(intakeGardenEnd);
        park = Paths.curve(parkStart, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }
}
