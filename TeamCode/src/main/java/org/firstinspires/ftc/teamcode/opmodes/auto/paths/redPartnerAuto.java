package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class redPartnerAuto {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(63.9923, 133.8787, 90);
    // Mirroring both axes adds 180 degrees to both exported starting headings.
    private final Pose path1Start = poseFactory.of(63.9923, 133.8787, 270);
    private final Pose path1 = poseFactory.of(47.2811, 119.9191, 90);
    private final Pose path1Control1 = poseFactory.of(56.2734, 110.1566, 180);
    private final Pose point2 = poseFactory.of(47.2394, 129.319, 90);
    private final Pose point3 = poseFactory.of(50.3235, 5.9891, 180);
    private final Pose point3Control1 = poseFactory.of(70.2677, 73.9634, 180);
    private final Pose point4 = poseFactory.of(8.1226, 6.1284, 180);
    private final Pose point5 = poseFactory.of(15.1672, 47.0796, 180);
    private final Pose point5Control1 = poseFactory.of(28.2259, 30.7041, 180);
    private final Pose point6 = poseFactory.of(11.6791, 47.1226, 180);
    private final Pose point7 = poseFactory.of(53.8569, 7.8723, 90);
    private final Pose point8 = poseFactory.of(11.6008, 95.2657, 90);
    private final Pose point8Control1 = poseFactory.of(8.9641, 69.4782, 180);

    public Path path1() {
        return curve(path1Start, path1Control1, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return line(path1, point2).constant(point2);
    }

    public Path path3() {
        return curve(point2, point3Control1, point3).linear(point2, point3);
    }

    public Path path4() {
        return line(point3, point4).constant(point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }

    public Path path6() {
        return line(point5, point6).constant(point6);
    }

    public Path path7() {
        return line(point6, point7).linear(point6, point7);
    }

    public Path path8() {
        return curve(point7, point8Control1, point8).linear(point7, point8);
    }
}
