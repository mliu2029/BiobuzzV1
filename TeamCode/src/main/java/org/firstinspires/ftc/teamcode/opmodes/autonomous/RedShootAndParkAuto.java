package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@NextAutonomous(name = "red hive+park", group = "Pedro")
public class RedShootAndParkAuto extends NextOpMode {
    private final Robot robot;
    private final Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private String currentPath = "Ready";

    private final Pose startPose = poseFactory.of(55, 9, 90);
    private final Pose park = poseFactory.of(14.2259, 95.9422, 0);
    private final Pose parkControlPose = poseFactory.of(22.7118, 50.0193, 0);

    private Path runToPark;

    public RedShootAndParkAuto(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);
        follower = Drivetrain.INSTANCE.getFollower();
        follower.setPose(startPose);
        follower.update();
        buildPaths();
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    public void buildPaths() {
        runToPark = curve(startPose, parkControlPose, park).linear(startPose, park);
    }

    private Command auto() {
        return sequential(
                Commands.instant(() -> currentPath = "Shooting"),
                Shooter.INSTANCE.runFlywheel,
                Commands.waitMs(500),
                Intake.INSTANCE.runIntake,
                Transfer.INSTANCE.runTransfer(0.8),
                Commands.waitMs(5000),
                Transfer.INSTANCE.stopTransfer,
                followPath(runToPark, "Run to Park"),
                robot.stop(),
                Commands.instant(() -> currentPath = "Finished")
        );
    }

    private Command followPath(Path path, String pathName) {
        return sequential(
                Commands.instant(() -> currentPath = pathName),
                Drivetrain.INSTANCE.followPath(path)
        );
    }

    @Override
    public void start() {
        auto().schedule();
    }

    @Override
    public void periodic() {
        follower.update();
        telemetry.addData("Current Path", currentPath);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (degrees)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void end() {
        try {
            OpModeStorage.autonomousEndPose = follower.pose();
        } finally {
            robot.stopImmediately();
        }
    }
}
