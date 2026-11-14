package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.core.BaseAuto;
import org.firstinspires.ftc.teamcode.util.Alliance;

@NextAutonomous(name = "REAL AUTO", group = "Pedro")
public class RealAuto extends BaseAuto {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(53.0036, 6.0788, 90);
    private final Pose parkStart = poseFactory.of(53.4821, 6.5665, 90);
    private final Pose parkEnd = poseFactory.of(14.5, 93.2373, 0);
    private final Pose parkControl = poseFactory.of(15, 55, 0);

    private final Path park;

    public RealAuto(Robot robot) {
        super(robot);
        park = Paths.curve(startPose, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }

    @Override
    public void start() {
        startAuto(Alliance.RED, startPose);
        sequential(
                commands.init(),
                Commands.waitMs(1500),
                commands.shoot(),
                commands.followPath(park, "park"),
                commands.finish()
        ).schedule();
    }
}
