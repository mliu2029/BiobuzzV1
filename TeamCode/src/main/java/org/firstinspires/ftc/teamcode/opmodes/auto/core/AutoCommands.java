package org.firstinspires.ftc.teamcode.opmodes.auto.core;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.Path;
import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

public class AutoCommands {
    private static final double SHOT_READY_TIMEOUT_MS = 2000;
    private static final double FEED_DURATION_MS = 1000;
    private static final double FLYWHEEL_TOLERANCE_TICKS_PER_SECOND = 75;
    private static final double MAX_TURRET_RATE_DEGREES_PER_SECOND = 30;

    private final Robot robot;
    private final Consumer<String> status;

    public AutoCommands(Robot robot, Consumer<String> status) {
        this.robot = robot;
        this.status = status;
    }

    public Command init() {
        return sequential(
                Commands.instant(() -> status.accept("Starting")),
                robot.Shooter().runFlywheel,
                robot.Ramp().up,
                Commands.instant(() -> {
                    Follower follower = robot.Drivetrain().getFollower();
                    robot.Turret().useOdometryAim(follower::pose, follower::velocity);
                    robot.Turret().setOdometryVisionEnabled(true);
                }).requiring(robot.Turret()),
                robot.Intake().runIntake
        );
    }
    public Command shoot() {
        Command feedBurst = Commands.waitMs(FEED_DURATION_MS)
                .setEnd(condition -> robot.Transfer().setFeeding(false))
                .with(Commands.instant(() -> {
                    status.accept("Shooting");
                    robot.Transfer().setFeeding(true);
                    robot.Intake().restart();
                }).requiring(robot.Transfer(), robot.Intake()));

        return sequential(
                Commands.instant(() -> status.accept("Waiting for turret and flywheel")),
                Commands.waitUntil(this::shotReady).raceWith(Commands.waitMs(SHOT_READY_TIMEOUT_MS)),
                Commands.conditional(this::shotReady, feedBurst,
                        Commands.instant(() -> status.accept("Shot skipped: aim or speed not ready")))
        );
    }

    private boolean shotReady() {
        Shooter shooter = robot.Shooter();
        Turret turret = robot.Turret();
        double target = shooter.getTargetVelocity();
        return shooter.isEnabled() && target > 0
                && Math.abs(shooter.getFlywheelVelocity() - target) <= FLYWHEEL_TOLERANCE_TICKS_PER_SECOND
                && turret.isAligned() && turret.isVisionAligned()
                && Math.abs(turret.getVelocityDegreesPerSecond()) <= MAX_TURRET_RATE_DEGREES_PER_SECOND;
    }
    public Command followPath(Path path, String name) {
        return sequential(
                Commands.instant(() -> status.accept(name)),
                robot.Drivetrain().followPath(path)
        );
    }

    public Command finish() {
        return sequential(
                robot.stop(),
                Commands.instant(() -> status.accept("Finished"))
        );
    }
}
