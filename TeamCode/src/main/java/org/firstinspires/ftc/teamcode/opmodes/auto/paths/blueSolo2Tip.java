package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Red solo route mirrored across both axes: x/y = 141.5 - x/y; headings +180 degrees. */
public class blueSolo2Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(88.4964, 135.4212, 270);
    private final Pose gardenPrepEnd = poseFactory.of(134.75, 124.75, 90);
    private final Pose gardenPrepControl = poseFactory.of(110.7777, 107.9633, 180);
    private final Pose intakeGardenEnd = poseFactory.of(135.031, 134.15, 90);
    private final Pose shoot1Start = poseFactory.of(135.031, 134.15, 90);
    private final Pose shoot1End = poseFactory.of(82.622, 13.2955, 90);
    private final Pose shoot1Control = poseFactory.of(118.4964, 43.3834, 180);
    private final Pose flowerPrep1Start = poseFactory.of(82.622, 13.2955, 90);
    private final Pose flowerPrep1End = poseFactory.of(94.75, 22.75, 270);
    private final Pose flowerPrep1Control = poseFactory.of(90.0632, 27.6516, 180);
    private final Pose flowerIntake1End = poseFactory.of(94.75, 8.75, 270);
    private final Pose returnToShootFarStart = poseFactory.of(94.75, 8.75, 270);
    private final Pose returnToShootFarEnd = poseFactory.of(84.4572, 12.7219, 90);
    // Preserve the supplied heading change after returnToShootFar.
    private final Pose flowerPrep2Start = poseFactory.of(84.4572, 12.7219, 270);
    private final Pose flowerPrep2End = poseFactory.of(117.75, 94.75, 0);
    private final Pose flowerPrep2Control = poseFactory.of(110.3812, 47.3248, 180);
    private final Pose flowerIntake2End = poseFactory.of(130.005, 94.6338, 0);
    private final Pose shoot2Start = poseFactory.of(130.005, 94.6338, 0);
    private final Pose shoot2End = poseFactory.of(84.6422, 12.504, 90);
    private final Pose shoot2Control = poseFactory.of(112.7848, 43.1444, 180);
    private final Pose parkStart = poseFactory.of(84.6422, 12.504, 90);
    private final Pose parkEnd = poseFactory.of(129.5161, 25.1843, 180);
    private final Pose parkControl = poseFactory.of(101.222, 30.4179, 180);

    public final Path gardenPrep;
    public final Path intakeGarden;
    //public final Path shoot1;
    //public final Path flowerPrep1;
    //public final Path flowerIntake1;
    //public final Path returnToShootFar;
    //public final Path flowerPrep2;
    //public final Path flowerIntake2;
    //public final Path shoot2;
    public final Path park;

    public blueSolo2Tip() {
        gardenPrep = Paths.curve(startPose, gardenPrepControl, gardenPrepEnd)
                .linear(startPose, gardenPrepEnd);
        intakeGarden = Paths.line(gardenPrepEnd, intakeGardenEnd)
                .constant(intakeGardenEnd);
        //shoot1 = Paths.curve(shoot1Start, shoot1Control, shoot1End)
                //.linear(shoot1Start, shoot1End);
        //flowerPrep1 = Paths.curve(flowerPrep1Start, flowerPrep1Control, flowerPrep1End)
                //.linear(flowerPrep1Start, flowerPrep1End);
        //flowerIntake1 = Paths.line(flowerPrep1End, flowerIntake1End)
                //.constant(flowerIntake1End);
        //returnToShootFar = Paths.line(returnToShootFarStart, returnToShootFarEnd)
                //.linear(returnToShootFarStart, returnToShootFarEnd);
        //flowerPrep2 = Paths.curve(flowerPrep2Start, flowerPrep2Control, flowerPrep2End)
                //.linear(flowerPrep2Start, flowerPrep2End);
        //flowerIntake2 = Paths.line(flowerPrep2End, flowerIntake2End)
                //.constant(flowerIntake2End);
        //shoot2 = Paths.curve(shoot2Start, shoot2Control, shoot2End)
                //.linear(shoot2Start, shoot2End);
        park = Paths.curve(parkStart, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }
}
