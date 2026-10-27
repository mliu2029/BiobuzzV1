package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Exported garden, two-flower, and park route for GardenAndFlowersAuto. */
public class redSolo2Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(54.0964, 6.2233, 90);
    // The export starts localization at 90 degrees and the first path at 180 degrees.
    private final Pose intakeGardenStart = poseFactory.of(54.0964, 6.2233, 180);
    private final Pose intakeGardenEnd = poseFactory.of(8.6641, 5.7964, 180);
    private final Pose shoot1End = poseFactory.of(58.878, 128.2045, 270);
    private final Pose shoot1Control = poseFactory.of(23.0036, 98.1166, 0);
    private final Pose flowerPrep1End = poseFactory.of(47.7762, 123.7677, 90);
    private final Pose flowerPrep1Control = poseFactory.of(51.4368, 113.8484, 0);
    private final Pose flowerIntake1End = poseFactory.of(47.2381, 132.3215, 90);
    private final Pose flowerPrep2End = poseFactory.of(23.4507, 47.1668, 180);
    private final Pose flowerIntake2End = poseFactory.of(11.583, 47.1973, 180);
    private final Pose shoot2End = poseFactory.of(56.8578, 128.996, 270);
    private final Pose shoot2Control = poseFactory.of(28.7152, 98.3556, 0);
    private final Pose parkEnd = poseFactory.of(11.9839, 116.3157, 0);
    private final Pose parkControl = poseFactory.of(40.278, 111.0821, 0);

    public final Path intakeGarden;
    public final Path shoot1;
    public final Path flowerPrep1;
    public final Path flowerIntake1;
    public final Path flowerPrep2;
    public final Path flowerIntake2;
    public final Path shoot2;
    public final Path park;

    public redSolo2Tip() {
        intakeGarden = line(intakeGardenStart, intakeGardenEnd)
                .linear(intakeGardenStart, intakeGardenEnd);
        shoot1 = curve(intakeGardenEnd, shoot1Control, shoot1End)
                .linear(intakeGardenEnd, shoot1End);
        flowerPrep1 = curve(shoot1End, flowerPrep1Control, flowerPrep1End)
                .linear(shoot1End, flowerPrep1End);
        flowerIntake1 = line(flowerPrep1End, flowerIntake1End)
                .constant(flowerIntake1End);
        flowerPrep2 = line(flowerIntake1End, flowerPrep2End)
                .linear(flowerIntake1End, flowerPrep2End);
        flowerIntake2 = line(flowerPrep2End, flowerIntake2End)
                .constant(flowerIntake2End);
        shoot2 = curve(flowerIntake2End, shoot2Control, shoot2End)
                .linear(flowerIntake2End, shoot2End);
        park = curve(shoot2End, parkControl, parkEnd)
                .linear(shoot2End, parkEnd);
    }
}
