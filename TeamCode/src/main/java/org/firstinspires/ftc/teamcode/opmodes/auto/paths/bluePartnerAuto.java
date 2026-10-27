package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class bluePartnerAuto {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPose = poseFactory.of(77.5077, 7.6213, 270);
    private final Pose path1Start = poseFactory.of(77.5077, 7.6213, 90);
    private final Pose path1 = poseFactory.of(94.2189, 21.5809, 270);
    private final Pose path1Control1 = poseFactory.of(85.2266, 31.3434, 0);
    private final Pose point2 = poseFactory.of(94.2606, 12.181, 270);
    private final Pose point3 = poseFactory.of(91.1765, 135.5109, 0);
    private final Pose point3Control1 = poseFactory.of(71.2323, 67.5366, 0);
    private final Pose point4 = poseFactory.of(133.3774, 135.3716, 0);
    private final Pose point5 = poseFactory.of(126.3328, 94.4204, 0);
    private final Pose point5Control1 = poseFactory.of(113.2741, 110.7959, 0);
    private final Pose point6 = poseFactory.of(129.8209, 94.3774, 0);
    private final Pose point7 = poseFactory.of(87.6431, 133.6277, 270);
    private final Pose point8 = poseFactory.of(129.8992, 46.2343, 270);
    private final Pose point8Control1 = poseFactory.of(132.5359, 72.0218, 0);

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
