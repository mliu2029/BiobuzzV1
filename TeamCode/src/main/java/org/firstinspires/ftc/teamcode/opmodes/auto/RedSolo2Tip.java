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

@NextAutonomous(name = "Red Solo (2 Tip)", group = "Pedro")
public class RedSolo2Tip extends BaseAuto {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(53.0036, 6.0788, 90);
    private final Pose gardenPrepEnd = poseFactory.of(6.75, 16.75, 270);
    private final Pose gardenPrepControl = poseFactory.of(30.7223, 33.5367, 0);
    private final Pose intakeGardenEnd = poseFactory.of(6.469, 7.35, 270);
    private final Pose parkStart = poseFactory.of(6.4821, 7.5665, 270);
    private final Pose parkEnd = poseFactory.of(12.0657, 93.2373, 0);
    private final Pose parkControl = poseFactory.of(38.6222, 56.7238, 0);

    private final Path gardenPrep;
    private final Path intakeGarden;
    private final Path park;

    public RedSolo2Tip(Robot robot) {
        super(robot);
        gardenPrep = Paths.curve(startPose, gardenPrepControl, gardenPrepEnd)
                .linear(startPose, gardenPrepEnd);
        intakeGarden = Paths.line(gardenPrepEnd, intakeGardenEnd)
                .constant(intakeGardenEnd);
        park = Paths.curve(parkStart, parkControl, parkEnd)
                .linear(parkStart, parkEnd);
    }

    @Override
    public void start() {
        startAuto(Alliance.RED, startPose);
        sequential(
                commands.init(),
                Commands.waitMs(1500),
                commands.shoot(),
                commands.followPath(gardenPrep, "gardenPrep"),
                commands.followPath(intakeGarden, "intakeGarden"),
                Commands.waitMs(2000),
                commands.followPath(park, "park"),
                commands.finish()
        ).schedule();
    }
}
