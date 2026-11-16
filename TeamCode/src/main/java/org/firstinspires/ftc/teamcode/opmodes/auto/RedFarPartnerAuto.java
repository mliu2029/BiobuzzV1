package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

@NextAutonomous(name = "red far partner auto", group = "Pedro")
public class RedFarPartnerAuto extends BaseAuto {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(64.2742, 134.4415, 270);
    private final Pose path1End = poseFactory.of(11.8757, 91.5075, 0);

    private Path path1() {
        return Paths.line(startPose, path1End)
                .linear(startPose, path1End);
    }

    public RedFarPartnerAuto(Robot robot) {
        super(robot);
    }

    @Override
    public void start() {
        startAuto(Alliance.RED, startPose);
        sequential(
                commands.init(),
                Commands.waitMs(20000),
                commands.shoot(),
                commands.followPath(path1(), "path1"),
                commands.finish()
        ).schedule();
    }
}
