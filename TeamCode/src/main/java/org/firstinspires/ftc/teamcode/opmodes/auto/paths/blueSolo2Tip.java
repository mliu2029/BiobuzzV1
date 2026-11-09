package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Red solo route mirrored across both field centerlines via PoseFactory.mirrorX/mirrorY (x/y = 141.5 - x/y, headings +180). Edit redSolo2Tip, then copy the pose numbers here unchanged. */
public class blueSolo2Tip {
    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

    public final Pose startPose = poseFactory.of(53.0036, 6.0788, 90);
    private final Pose parkStart = poseFactory.of(53.4821, 6.5665, 90);
    private final Pose parkEnd = poseFactory.of(14.5, 93.2373, 0);
    private final Pose parkControl = poseFactory.of(15, 55, 0);

    public final Path park;

    public blueSolo2Tip() {
        park = Paths.curve(startPose, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }
}
