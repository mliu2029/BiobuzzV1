package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Imported from farRedPartnerAuto.pp; segment order follows the visualizer sequence. */
public class redPartnerAuto {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(60.541078305519896, 133.51540436456997, 90);
    // Preserve the exported heading discontinuity at this segment boundary.
    private final Pose PrepFlowerIntake1Start = poseFactory.of(60.541078305519896, 133.51540436456997, 270);
    private final Pose PrepFlowerIntake1Control = poseFactory.of(51.75, 119.75, 0);
    private final Pose PrepFlowerIntake1End = poseFactory.of(46.75, 119.75, 90);
    private final Pose FlowerIntake1End = poseFactory.of(46.75, 129.75, 90);
    private final Pose returnToShootFarControl = poseFactory.of(51.75, 119.75, 0);
    private final Pose returnToShootFarEnd = poseFactory.of(58.75, 132.75, 270);
    private final Pose PrepFlowerIntake2Control = poseFactory.of(22.038107557894058, 99.91097372445887, 0);
    private final Pose PrepFlowerIntake2End = poseFactory.of(22.75, 46.75, 180);
    private final Pose FlowerIntake2End = poseFactory.of(10.75, 46.75, 180);
    private final Pose shootClose1End = poseFactory.of(56.82486361383621, 8.566077641755463, 90);
    private final Pose prepGardenIntakeControl = poseFactory.of(33.860468752442344, 28.357867461852212, 0);
    private final Pose prepGardenIntakeEnd = poseFactory.of(7.465132879435952, 14.969995215421497, 270);
    private final Pose gardenIntakeEnd = poseFactory.of(6.766643359137815, 7.866283693666176, 270);
    private final Pose shootClose2End = poseFactory.of(56.703702716801764, 8.297587500873107, 90);
    private final Pose parkControl = poseFactory.of(9.298524874092516, 62.57689080069474, 0);
    private final Pose parkEnd = poseFactory.of(11.796271421341455, 94.76518581999326, 0);

    public Path PrepFlowerIntake1() {
        return curve(PrepFlowerIntake1Start, PrepFlowerIntake1Control, PrepFlowerIntake1End)
                .linear(PrepFlowerIntake1Start, PrepFlowerIntake1End);
    }

    public Path FlowerIntake1() {
        return line(PrepFlowerIntake1End, FlowerIntake1End)
                .constant(FlowerIntake1End);
    }

    public Path returnToShootFar() {
        // Match the visualizer's counterclockwise half-turn.
        return curve(FlowerIntake1End, returnToShootFarControl, returnToShootFarEnd)
                .heading((ignored, progress) -> FlowerIntake1End.heading() + Math.PI * progress);
    }

    public Path PrepFlowerIntake2() {
        return curve(returnToShootFarEnd, PrepFlowerIntake2Control, PrepFlowerIntake2End)
                .linear(returnToShootFarEnd, PrepFlowerIntake2End);
    }

    public Path FlowerIntake2() {
        return line(PrepFlowerIntake2End, FlowerIntake2End)
                .constant(FlowerIntake2End);
    }

    public Path shootClose1() {
        return line(FlowerIntake2End, shootClose1End)
                .linear(FlowerIntake2End, shootClose1End);
    }

    public Path prepGardenIntake() {
        // Match the visualizer's counterclockwise half-turn.
        return curve(shootClose1End, prepGardenIntakeControl, prepGardenIntakeEnd)
                .heading((ignored, progress) -> shootClose1End.heading() + Math.PI * progress);
    }

    public Path gardenIntake() {
        return line(prepGardenIntakeEnd, gardenIntakeEnd)
                .constant(gardenIntakeEnd);
    }

    public Path shootClose2() {
        return line(gardenIntakeEnd, shootClose2End)
                .linear(gardenIntakeEnd, shootClose2End);
    }

    public Path park() {
        return curve(shootClose2End, parkControl, parkEnd)
                .linear(shootClose2End, parkEnd);
    }

}
