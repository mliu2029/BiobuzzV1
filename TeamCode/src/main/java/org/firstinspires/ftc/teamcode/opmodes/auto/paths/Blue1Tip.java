package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Red 1-tip route mirrored across both field axes; headings rotated 180 degrees. */
public class Blue1Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(88.4964, 135.4212, 270);
    private final Pose gardenPrepEnd = poseFactory.of(134.9121, 125.2039, 90);
    private final Pose gardenPrepControl = poseFactory.of(110.7777, 107.9633, 180);
    private final Pose intakeGardenEnd = poseFactory.of(135.031, 134.15, 90);
    private final Pose parkStart = poseFactory.of(135.031, 134.15, 90);
    private final Pose parkEnd = poseFactory.of(129.5161, 25.1843, 180);
    private final Pose parkControl = poseFactory.of(101.222, 30.4179, 180);

    public final Path gardenPrep;
    public final Path intakeGarden;
    public final Path park;

    public Blue1Tip() {
        gardenPrep = Paths.curve(startPose, gardenPrepControl, gardenPrepEnd)
                .linear(startPose, gardenPrepEnd);
        intakeGarden = Paths.line(gardenPrepEnd, intakeGardenEnd)
                .constant(intakeGardenEnd);
        park = Paths.curve(parkStart, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }
}
