package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Red route mirrored across both field centerlines: x/y = 141.5 - x/y. */
public class blueSolo2Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(87.4036, 135.2767, 270);
    // Mirroring both axes adds 180 degrees to every heading, including the start pose.
    private final Pose intakeGardenStart = poseFactory.of(87.4036, 135.2767, 0);
    private final Pose intakeGardenEnd = poseFactory.of(132.8359, 135.7036, 0);
    private final Pose shoot1End = poseFactory.of(82.622, 13.2955, 90);
    private final Pose shoot1Control = poseFactory.of(118.4964, 43.3834, 180);
    private final Pose flowerPrep1End = poseFactory.of(93.7238, 17.7323, 270);
    private final Pose flowerPrep1Control = poseFactory.of(90.0632, 27.6516, 180);
    private final Pose flowerIntake1End = poseFactory.of(94.2619, 9.1785, 270);
    private final Pose flowerPrep2End = poseFactory.of(118.0493, 94.3332, 0);
    private final Pose flowerIntake2End = poseFactory.of(129.917, 94.3027, 0);
    private final Pose shoot2End = poseFactory.of(84.6422, 12.504, 90);
    private final Pose shoot2Control = poseFactory.of(112.7848, 43.1444, 180);
    private final Pose parkEnd = poseFactory.of(129.5161, 25.1843, 180);
    private final Pose parkControl = poseFactory.of(101.222, 30.4179, 180);

    public final Path intakeGarden;
    public final Path shoot1;
    public final Path flowerPrep1;
    public final Path flowerIntake1;
    public final Path flowerPrep2;
    public final Path flowerIntake2;
    public final Path shoot2;
    public final Path park;

    public blueSolo2Tip() {
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
