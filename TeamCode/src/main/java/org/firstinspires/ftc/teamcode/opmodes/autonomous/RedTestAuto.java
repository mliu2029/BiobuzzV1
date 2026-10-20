package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
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

@NextAutonomous(name = "red hive+intake", group = "Pedro")
public class RedTestAuto extends NextOpMode {
    private final Robot robot;
    private final Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private String currentPath = "Ready";

    private final Pose start = poseFactory.of(59.0879, 8.1816, 90);
    private final Pose path1 = poseFactory.of(15.6213, 11.8409, 75);
    private final Pose path1Control1 = poseFactory.of(47.052, 43.7612, 0);
    private final Pose point2 = poseFactory.of(9.6085, 10.0635, 75);
    private final Pose point3Start = poseFactory.of(9.6085, 10.0635, 90);
    private final Pose point3 = poseFactory.of(10.3158, 99.4397, 0);
    private final Pose point3Control1 = poseFactory.of(30.6675, 75.482, 0);

    private Path goToIntakeCorner, intakeCorner, runToPark;

    public RedTestAuto(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);
        follower = Drivetrain.INSTANCE.getFollower();
        follower.setPose(start);
        follower.update();
        buildPaths();
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    public void buildPaths() {
        goToIntakeCorner = curve(start, path1Control1, path1)
                .linear(start, path1);
        intakeCorner = line(path1, point2).reverseTangent();
        runToPark = curve(point3Start, point3Control1, point3)
                .linear(point3Start, point3);
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
                followPath(goToIntakeCorner, "Go to Intake Corner"),
                followPath(intakeCorner, "Intake Corner"),
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
