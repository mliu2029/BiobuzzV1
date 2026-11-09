package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Red solo route: garden prep, garden intake, then park. */
public class redSolo2Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(53.0036, 6.0788, 90);
    private final Pose gardenPrepEnd = poseFactory.of(6.75, 16.75, 270);
    private final Pose gardenPrepControl = poseFactory.of(30.7223, 33.5367, 0);
    private final Pose intakeGardenEnd = poseFactory.of(6.469, 7.35, 270);
    private final Pose parkStart = poseFactory.of(6.4821, 7.5665, 270);
    private final Pose parkEnd = poseFactory.of(12.0657, 93.2373, 0);
    private final Pose parkControl = poseFactory.of(38.6222, 56.7238, 0);

    public final Path gardenPrep;
    public final Path intakeGarden;
    public final Path park;

    public redSolo2Tip() {
        gardenPrep = Paths.curve(startPose, gardenPrepControl, gardenPrepEnd)
                .linear(startPose, gardenPrepEnd);
        intakeGarden = Paths.line(gardenPrepEnd, intakeGardenEnd)
                .constant(intakeGardenEnd);
        park = Paths.curve(parkStart, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }
}
