package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.util.TurretConfig;
import org.firstinspires.ftc.teamcode.util.TurretAngleEstimator;
import org.firstinspires.ftc.teamcode.opmodes.teleop.mainTeleOp;

@Configurable
@NextTeleop(name = "odo turret test", group = "Tuning")
public class OdoTurretTest extends mainTeleOp {
    public static volatile double shooterVelocity = 0;
    public static volatile boolean autoShooterVelocity = true;
    public static volatile boolean blueAlliance = false;
    public static volatile boolean useVision = true;

    public OdoTurretTest(Robot robot) {
        super(robot, blueAlliance ? Alliance.BLUE : Alliance.RED);
        shooterVelocity = 0;
    }

    @Override
    protected Alliance getAlliance() {
        return blueAlliance ? Alliance.BLUE : Alliance.RED;
    }

    @Override
    protected double getShooterTargetVelocity() {
        double requestedVelocity = shooterVelocity;
        if (!Double.isFinite(requestedVelocity) || requestedVelocity <= 0) return 0;
        if (!autoShooterVelocity) return requestedVelocity;
        double distanceInches = getGoalDistanceInches();
        if (!Double.isFinite(distanceInches) || distanceInches < 0) return 0;
        double velocity = Calculations.velocity(distanceInches);
        return Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
    }

    @Override
    protected void setShooterTargetVelocity(double velocity) {
        shooterVelocity = Double.isFinite(velocity) ? Math.max(0, velocity) : 0;
    }

    @Override
    protected void adjustVelocity(double change) {
        if (autoShooterVelocity) {
            if (shooterVelocity > 0) shooterVelocity = getShooterTargetVelocity();
            autoShooterVelocity = false;
        }
        super.adjustVelocity(change);
    }

    protected double getGoalDistanceInches() {
        Pose pose = robot.Drivetrain().getFollower().pose();
        HiveTarget goal = robot.Turret().getTarget();
        return pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                ? Double.NaN : Math.hypot(goal.x - pose.x(), goal.y - pose.y());
    }

    @Override
    protected void configureAdditionalControls(CommandGamepad driver, CommandGamepad driver2) {
        driver2.cross().onTrue(Commands.instant(() -> useVision = !useVision));
        driver2.dpadUp().onTrue(robot.Turret().stopTurret);
        driver2.dpadRight().onTrue(robot.Turret().aimTurret);
    }
    @Override
    public void disabledPeriodic() {
        if (!telemetryDue(System.nanoTime())) return;
        Pose startingPose = getStartingPose();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Aim source", visionModeLabel());
        telemetry.addData("Shooter speed mode", autoShooterVelocity ? "AUTO DISTANCE" : "MANUAL");
        telemetry.addData("3D AprilTag pipeline", TurretConfig.pipeline);
        telemetry.addData("Auto end pose (inches / degrees)", "x=%.2f y=%.2f heading=%.1f",
                startingPose.x(), startingPose.y(), Math.toDegrees(startingPose.heading()));

        telemetry.update();
    }
    @Override
    protected double getStartingTurretDegrees() {
        return 0;
    }

    @Override
    protected void startTurretAim() {
        Follower follower = robot.Drivetrain().getFollower();
        robot.Turret().useOdometryAim(follower::pose, follower::velocity);
        updateTurretAim();
    }

    @Override
    protected void updateTurretAim() {
        robot.Turret().setAlliance(getAlliance());
        if (useVision) robot.Limelight().setPipeline(TurretConfig.pipeline);
        robot.Turret().setOdometryVisionEnabled(useVision);
    }

    @Override
    protected void addAdditionalDriverTelemetry() {
        Pose pose = robot.Drivetrain().getFollower().pose();
        HiveTarget goal = robot.Turret().getTarget();
        double distanceInches = getGoalDistanceInches();
        addDriverData("Goal (selected by odometry)", goal);
        addDriverData("Turret alliance / accepted hive tags", robot.Turret().getAlliance()
                + " / " + goal.firstTagId + "-" + (goal.firstTagId + 3));
        addDriverData("Distance to goal (in, odometry)", Double.isFinite(distanceInches)
                ? String.format(Locale.US, "%.1f", distanceInches) : "Unavailable");
        addDriverData("Pose (inches / degrees)", pose == null ? "Unavailable"
                : String.format(Locale.US, "x=%.2f y=%.2f heading=%.1f",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        addDriverData("Turret chassis angle / saved offset (deg)", String.format(Locale.US,
                "%.1f / %+.1f", robot.Turret().getChassisDegrees(),
                robot.Turret().getOdometryAngleOffsetDegrees()));
        addDriverData("Vision correction", robot.Turret().getOdometryCorrectionStatus());
        addDriverData("Shooter speed mode", autoShooterVelocity ? "AUTO DISTANCE" : "MANUAL");
    }

    private static String visionModeLabel() {
        return !useVision ? "ODOMETRY ONLY"
                : TurretAngleEstimator.applyVisionCorrections
                        ? "LIMELIGHT 3D; odometry fallback; stationary offset learning ON"
                        : "LIMELIGHT 3D; odometry fallback; stationary offset learning OFF";
    }
}
