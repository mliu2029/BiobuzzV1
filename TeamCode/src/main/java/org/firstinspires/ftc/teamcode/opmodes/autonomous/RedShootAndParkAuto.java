package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.curve;
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

@Autonomous(name = "red hive+park", group = "Pedro")
public class RedShootAndParkAuto extends OpMode {
    private final Robot robot = new Robot();
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private String currentPath = "Ready";

    private final Pose startPose = poseFactory.of(55, 9, 90);
    private final Pose park = poseFactory.of(14.2259, 95.9422, 0);
    private final Pose parkControlPose = poseFactory.of(22.7118, 50.0193, 0);

    private Path runToParkPath() {
        return curve(startPose, parkControlPose, park).linear(startPose, park);
    }

    private Command autoRoutine() {
        return sequential(
                Commands.instant(() -> currentPath = "Shooting"),
                robot.shooter.runAtVelocity(robot.shooter.getTargetVelocity()),
                Commands.waitMs(500),
                robot.transfer.run().with(robot.intake.run())
                        .raceWith(Commands.waitMs(5000)),
                Commands.instant(() -> currentPath = "Run to Park"),
                follow(follower, runToParkPath())
                        .setDone(() -> follower.atParametricEnd() && !follower.isBusy())
                        .requiring(robot.drivetrain),
                robot.stop(),
                Commands.instant(() -> currentPath = "Finished")
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
        follower.setPose(startPose);
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
        // Plain SDK OpModes must update NextMotor outputs explicitly.
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
