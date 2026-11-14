package org.firstinspires.ftc.teamcode.opmodes.auto.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.LoopTiming;

public abstract class BaseAuto extends NextOpMode {
    private final Robot robot;
    private Follower follower;
    protected final AutoCommands commands;
    private String currentPath = "Ready";
    private final LoopTiming loopTiming;
    private long lastTelemetryNanos;

    protected BaseAuto(Robot robot) {
        this(robot, new LoopTiming());
    }

    private BaseAuto(Robot robot, LoopTiming loopTiming) {
        super(robot, BulkReadHook.INSTANCE, loopTiming);
        this.robot = robot;
        this.loopTiming = loopTiming;
        telemetry.setMsTransmissionInterval(200);
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
        lastTelemetryNanos = 0;
    }

    @Override
    public void periodic() {
        if (follower == null) return;
        follower.update();
        if (!Thread.currentThread().isInterrupted()) {
            OpModeStorage.saveAutonomousEndPose(follower.pose());
        }
        long now = System.nanoTime();
        if (lastTelemetryNanos != 0 && now - lastTelemetryNanos < 200_000_000L) return;
        lastTelemetryNanos = now;
        Pose pose = follower.pose();
        telemetry.addData("Path", currentPath);
        telemetry.addData("Pose (in / deg)", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Loop ms (avg / p95 / max; last 256)", loopTiming.summary());
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
