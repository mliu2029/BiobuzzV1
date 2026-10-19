package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import dev.nextftc.hardware.actuators.NextMotor;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "red hive+intake", group = "Pedro")
public class RedTestAuto extends OpMode {
    private final Robot robot = new Robot();
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private String currentPath = "Ready";

    // Poses
    private final Pose start = poseFactory.of(59.0879, 8.1816, 90);
    private final Pose path1 = poseFactory.of(15.6213, 11.8409, 75);
    private final Pose path1Control1 = poseFactory.of(47.052, 43.7612, 0);
    private final Pose point2 = poseFactory.of(9.6085, 10.0635, 75);
    private final Pose point3Start = poseFactory.of(9.6085, 10.0635, 90);
    private final Pose point3 = poseFactory.of(10.3158, 99.4397, 0);
    private final Pose point3Control1 = poseFactory.of(30.6675, 75.482, 0);

    // Path methods
    private Path goToIntakeCornerPath() {
        return curve(start, path1Control1, path1)
                .linear(start, path1);
    }

    private Path intakeCornerPath() {
        return line(path1, point2).reverseTangent();
    }

    private Path runToParkPath() {
        return curve(point3Start, point3Control1, point3)
                .linear(point3Start, point3);
    }

    private Command autoRoutine() {
        return sequential(
                Commands.instant(() -> currentPath = "Shooting"),
                robot.shooter.runAtVelocity(robot.shooter.getTargetVelocity()),
                Commands.waitMs(500),
                robot.transfer.run().with(robot.intake.run())
                        .raceWith(Commands.waitMs(5000)),
                followPath(goToIntakeCornerPath(), "Go to Intake Corner"),
                followPath(intakeCornerPath(), "Intake Corner"),
                followPath(runToParkPath(), "Run to Park"),
                robot.stop(),
                Commands.instant(() -> currentPath = "Finished")
        );
    }

    private Command followPath(Path path, String pathName) {
        return sequential(
                Commands.instant(() -> currentPath = pathName),
                follow(follower, path)
                        .setDone(() -> follower.atParametricEnd() && !follower.isBusy())
                        .requiring(robot.drivetrain)
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        currentPath = "Ready";
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.initialize(hardwareMap);
        follower = robot.drivetrain.getFollower();
        follower.setPose(start);
        follower.update();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Current Path", currentPath);
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        robot.shooter.periodic();
        NextMotor.Companion.getMotorEventLoop().poll();

        telemetry.addData("Current Path", currentPath);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (degrees)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        try {
            if (follower != null) OpModeStorage.autonomousEndPose = follower.pose();
            Scheduler.reset();
        } finally {
            robot.stopImmediately();
        }
    }
}
