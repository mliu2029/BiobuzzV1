package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Supplied ten-segment red solo route. */
public class redSolo2Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(53.0036, 6.0788, 90);
    private final Pose gardenPrepEnd = poseFactory.of(6.75, 16.75, 270);
    private final Pose gardenPrepControl = poseFactory.of(30.7223, 33.5367, 0);
    private final Pose intakeGardenEnd = poseFactory.of(6.469, 7.35, 270);
    private final Pose shoot1Start = poseFactory.of(6.469, 7.35, 270);
    private final Pose shoot1End = poseFactory.of(58.878, 128.2045, 270);
    private final Pose shoot1Control = poseFactory.of(23.0036, 98.1166, 0);
    private final Pose flowerPrep1Start = poseFactory.of(58.878, 128.2045, 270);
    private final Pose flowerPrep1End = poseFactory.of(46.75, 118.75, 90);
    private final Pose flowerPrep1Control = poseFactory.of(51.4368, 113.8484, 0);
    private final Pose flowerIntake1End = poseFactory.of(46.75, 132.75, 90);
    private final Pose returnToShootFarStart = poseFactory.of(46.75, 132.75, 90);
    private final Pose returnToShootFarEnd = poseFactory.of(57.0428, 128.7781, 270);
    // Preserve the supplied heading change after returnToShootFar.
    private final Pose flowerPrep2Start = poseFactory.of(57.0428, 128.7781, 90);
    private final Pose flowerPrep2End = poseFactory.of(23.75, 46.75, 180);
    private final Pose flowerPrep2Control = poseFactory.of(31.1188, 94.1752, 0);
    private final Pose flowerIntake2End = poseFactory.of(11.495, 46.8662, 180);
    private final Pose shoot2Start = poseFactory.of(11.495, 46.8662, 180);
    private final Pose shoot2End = poseFactory.of(56.8578, 128.996, 270);
    private final Pose shoot2Control = poseFactory.of(28.7152, 98.3556, 0);
    private final Pose parkStart = poseFactory.of(56.8578, 128.996, 270);
    private final Pose parkEnd = poseFactory.of(11.9839, 116.3157, 0);
    private final Pose parkControl = poseFactory.of(40.278, 111.0821, 0);

    public final Path gardenPrep;
    public final Path intakeGarden;
    public final Path shoot1;
    public final Path flowerPrep1;
    public final Path flowerIntake1;
    public final Path returnToShootFar;
    public final Path flowerPrep2;
    public final Path flowerIntake2;
    public final Path shoot2;
    public final Path park;

    public redSolo2Tip() {
        gardenPrep = Paths.curve(startPose, gardenPrepControl, gardenPrepEnd)
                .linear(startPose, gardenPrepEnd);
        intakeGarden = Paths.line(gardenPrepEnd, intakeGardenEnd)
                .constant(intakeGardenEnd);
        shoot1 = Paths.curve(shoot1Start, shoot1Control, shoot1End)
                .linear(shoot1Start, shoot1End);
        flowerPrep1 = Paths.curve(flowerPrep1Start, flowerPrep1Control, flowerPrep1End)
                .linear(flowerPrep1Start, flowerPrep1End);
        flowerIntake1 = Paths.line(flowerPrep1End, flowerIntake1End)
                .constant(flowerIntake1End);
        returnToShootFar = Paths.line(returnToShootFarStart, returnToShootFarEnd)
                .linear(returnToShootFarStart, returnToShootFarEnd);
        flowerPrep2 = Paths.curve(flowerPrep2Start, flowerPrep2Control, flowerPrep2End)
                .linear(flowerPrep2Start, flowerPrep2End);
        flowerIntake2 = Paths.line(flowerPrep2End, flowerIntake2End)
                .constant(flowerIntake2End);
        shoot2 = Paths.curve(shoot2Start, shoot2Control, shoot2End)
                .linear(shoot2Start, shoot2End);
        park = Paths.curve(parkStart, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }
}
