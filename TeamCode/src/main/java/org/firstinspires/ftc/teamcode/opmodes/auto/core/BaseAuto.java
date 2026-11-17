package org.firstinspires.ftc.teamcode.opmodes.auto.core;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.LoopTiming;

public abstract class BaseAuto extends NextOpMode {
    protected final Robot robot;
    private Follower follower;
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
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    public Command initAuto() {
        return sequential(
                Commands.instant(() -> currentPath = "Starting"),
                robot.shooter.runFlywheel,
                robot.ramp.up,
                Commands.instant(() -> {
                    Follower fol = robot.drivetrain.getFollower();
                    robot.turret.useOdometryAim(fol::pose, fol::velocity);
                    robot.turret.setOdometryVisionEnabled(true);
                }).requiring(robot.turret),
                robot.intake.runIntake
        );
    }

    public Command followPath(Path path, String name) {
        return sequential(
                Commands.instant(() -> currentPath = name),
                robot.drivetrain.followPath(path)
        );
    }

    protected final void startAuto(Alliance alliance, Pose startPose) {
        OpModeStorage.autonomousEndTurretDegrees = 0.0;
        robot.initialize(hardwareMap, alliance);
        follower = robot.drivetrain.getFollower();
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
        telemetry.addData("Turret encoder", robot.turret.getEncoderConfigurationStatus());
        telemetry.addData("Turret ticks / power", "%d / %.3f",
                robot.turret.getEncoderTicks(), robot.turret.getPower());
        telemetry.addData("Turret deg (actual / target / PID error)", "%.2f / %.2f / %.2f",
                robot.turret.getDegrees(), robot.turret.getTargetDegrees(), robot.turret.getPidErrorDegrees());
        telemetry.update();
    }

    @Override
    public void end() {
        try {
            if (follower != null) {
                OpModeStorage.saveAutonomousEndTurretDegrees(robot.turret.getDegrees());
            }
        } finally {
            robot.stopImmediately();
        }
    }
}
