package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

public abstract class BaseAuto extends NextOpMode {
    private final Robot robot;
    private Follower follower;
    protected final AutoCommands commands;
    private String currentPath = "Ready";

    protected BaseAuto(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        commands = new AutoCommands(robot, status -> currentPath = status);
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    protected final void startAuto(Alliance alliance, Pose startPose) {
        OpModeStorage.autonomousEndTurretDegrees = 0.0;
        robot.initialize(hardwareMap);
        robot.Turret().initialize(hardwareMap, alliance);
        follower = robot.Drivetrain().getFollower();
        follower.setPose(startPose);
        OpModeStorage.saveAutonomousEndPose(startPose);
        follower.update();
    }

    @Override
    public void periodic() {
        if (follower == null) return;
        follower.update();
        if (!Thread.currentThread().isInterrupted()) {
            OpModeStorage.saveAutonomousEndPose(follower.pose());
        }
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
            if (follower != null) {
                OpModeStorage.saveAutonomousEndTurretDegrees(robot.Turret().getDegrees());
            }
        } finally {
            robot.stopImmediately();
        }
    }
}
