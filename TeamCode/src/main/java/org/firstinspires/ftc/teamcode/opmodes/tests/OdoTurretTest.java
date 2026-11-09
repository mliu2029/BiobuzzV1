package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.Calculations;
import org.firstinspires.ftc.teamcode.data.HiveTarget;
import org.firstinspires.ftc.teamcode.data.TurretConfig;
import org.firstinspires.ftc.teamcode.opmodes.teleop.mainTeleOp;

/** Main TeleOp controls with odometry fallback and four-tag 3D cell aiming. */
@Configurable
@NextTeleop(name = "odo turret test", group = "Tuning")
public class OdoTurretTest extends mainTeleOp {
    // Zero stops the shooter. Positive values enable it; manual mode uses this target directly.
    public static volatile double shooterVelocity = 0;
    public static volatile boolean autoShooterVelocity = true;
    public static boolean blueAlliance = false;
    public static volatile boolean useVision = true;
    public static volatile boolean usePointOfInterest = true;
    public static volatile boolean useTagGeometry = true;
    public static volatile int pointOfInterestTagId = 35;
    public static double startX = 70.75;
    public static double startY = 70.75;
    public static double startHeadingDegrees = 90;

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
            // A manual adjustment starts from the current regression target.
            if (shooterVelocity > 0) shooterVelocity = getShooterTargetVelocity();
            autoShooterVelocity = false;
        }
        super.adjustVelocity(change);
    }

    protected double getGoalDistanceInches() {
        Pose pose = robot.Drivetrain().getFollower().pose();
        HiveTarget goal = robot.Turret().getTarget();
        // Horizontal distance from the robot's odometry reference to the selected hive.
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
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Vision correction", visionModeLabel());
        telemetry.addData("Shooter speed mode", autoShooterVelocity ? "AUTO DISTANCE" : "MANUAL");
        telemetry.addData("Fallback tag hint / pipeline", "%d / %d", pointOfInterestTagId, TurretConfig.pipeline);
        telemetry.addData("Place robot (inches / degrees)", "x=%.2f y=%.2f heading=%.1f",
                startX, startY, startHeadingDegrees);

        telemetry.update();
    }

    @Override
    protected Pose getStartingPose() {
        if (!Double.isFinite(startX) || !Double.isFinite(startY)
                || !Double.isFinite(startHeadingDegrees)) {
            throw new IllegalArgumentException("Starting pose must be finite");
        }
        return new Pose(startX, startY, Math.toRadians(startHeadingDegrees));
    }

    @Override
    protected double getStartingTurretDegrees() {
        return 0;
    }

    @Override
    protected void startTurretAim() {
        robot.Turret().useOdometryForTest(robot.Drivetrain().getFollower()::pose);
        updateTurretAim();
    }

    @Override
    protected void updateTurretAim() {
        if (useVision) robot.Limelight().setPipeline(TurretConfig.pipeline);
        robot.Turret().setTestPointOfInterestTag(usePointOfInterest ? pointOfInterestTagId : -1);
        robot.Turret().setTestTagGeometryEnabled(usePointOfInterest && useTagGeometry);
        robot.Turret().setTestVisionEnabled(useVision);
    }

    @Override
    protected void addAdditionalDriverTelemetry() {
        Pose pose = robot.Drivetrain().getFollower().pose();
        HiveTarget goal = robot.Turret().getTarget();
        double distanceInches = getGoalDistanceInches();
        addDriverData("Goal", goal);
        addDriverData("Distance to goal (in, odometry)", Double.isFinite(distanceInches)
                ? String.format(Locale.US, "%.1f", distanceInches) : "Unavailable");
        addDriverData("Pose (inches / degrees)", pose == null ? "Unavailable"
                : String.format(Locale.US, "x=%.2f y=%.2f heading=%.1f",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        addDriverData("Vision correction", visionModeLabel());
        addDriverData("Shooter speed mode", autoShooterVelocity ? "AUTO DISTANCE" : "MANUAL");
        addDriverData("Test shooter controls", "D1 Up: start; Left/Right: manual -/+25; D2 Down: stop; Panels: autoShooterVelocity");
        addDriverData("Fallback tag hint / pipeline", pointOfInterestTagId + " / " + TurretConfig.pipeline);
        addDriverData("Test turret controls", "D2 Cross: vision; D-pad Up: stop; Right: resume");
    }

    private static String visionModeLabel() {
        return !useVision ? "OFF (odometry)" : usePointOfInterest && useTagGeometry
                ? "ON (four-tag 3D POI; odometry fallback)" : usePointOfInterest
                ? "ON (cell POI; odometry fallback)" : "ON (raw tag; odometry fallback)";
    }
}
