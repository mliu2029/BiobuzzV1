package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Imported from farBluePartnerAuto.pp; segment order follows the visualizer sequence. */
public class bluePartnerAuto {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(80.9589216944801, 7.984595635430026, 270);
    // Preserve the exported heading discontinuity at this segment boundary.
    private final Pose PrepFlowerIntake1Start = poseFactory.of(80.9589216944801, 7.984595635430026, 90);
    private final Pose PrepFlowerIntake1Control = poseFactory.of(89.75, 21.75, 0);
    private final Pose PrepFlowerIntake1End = poseFactory.of(94.75, 21.75, 270);
    private final Pose FlowerIntake1End = poseFactory.of(94.75, 11.75, 270);
    private final Pose returnToShootFarControl = poseFactory.of(89.75, 21.75, 0);
    private final Pose returnToShootFarEnd = poseFactory.of(82.75, 8.75, 90);
    private final Pose PrepFlowerIntake2Control = poseFactory.of(119.46189244210595, 41.58902627554113, 0);
    private final Pose PrepFlowerIntake2End = poseFactory.of(118.75, 94.75, 0);
    private final Pose FlowerIntake2End = poseFactory.of(130.75, 94.75, 0);
    private final Pose shootClose1End = poseFactory.of(84.67513638616379, 132.93392235824453, 270);
    private final Pose prepGardenIntakeControl = poseFactory.of(107.63953124755766, 113.14213253814779, 0);
    private final Pose prepGardenIntakeEnd = poseFactory.of(134.03486712056406, 126.53000478457851, 90);
    private final Pose gardenIntakeEnd = poseFactory.of(134.73335664086218, 133.63371630633384, 90);
    private final Pose shootClose2End = poseFactory.of(84.79629728319824, 133.2024124991269, 270);
    private final Pose parkControl = poseFactory.of(132.20147512590748, 78.92310919930526, 0);
    private final Pose parkEnd = poseFactory.of(129.70372857865854, 46.73481418000674, 180);

    public Path PrepFlowerIntake1() {
        // Match the visualizer's counterclockwise half-turn.
        return curve(PrepFlowerIntake1Start, PrepFlowerIntake1Control, PrepFlowerIntake1End)
                .heading((ignored, progress) -> PrepFlowerIntake1Start.heading() + Math.PI * progress);
    }

    public Path FlowerIntake1() {
        return line(PrepFlowerIntake1End, FlowerIntake1End)
                .constant(FlowerIntake1End);
    }

    public Path returnToShootFar() {
        return curve(FlowerIntake1End, returnToShootFarControl, returnToShootFarEnd)
                .linear(FlowerIntake1End, returnToShootFarEnd);
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
        return curve(shootClose1End, prepGardenIntakeControl, prepGardenIntakeEnd)
                .linear(shootClose1End, prepGardenIntakeEnd);
    }

    public Path gardenIntake() {
        return line(prepGardenIntakeEnd, gardenIntakeEnd)
                .constant(gardenIntakeEnd);
    }

    public Path shootClose2() {
        // Match the visualizer's counterclockwise half-turn.
        return line(gardenIntakeEnd, shootClose2End)
                .heading((ignored, progress) -> gardenIntakeEnd.heading() + Math.PI * progress);
    }

    public Path park() {
        return curve(shootClose2End, parkControl, parkEnd)
                .linear(shootClose2End, parkEnd);
    }

}
