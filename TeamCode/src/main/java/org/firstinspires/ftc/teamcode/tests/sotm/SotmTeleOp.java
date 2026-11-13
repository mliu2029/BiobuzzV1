package org.firstinspires.ftc.teamcode.tests.sotm;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import dev.nextftc.robot.triggers.CommandGamepad;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.teleop.mainTeleOp;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.util.TurretConfig;

/** Standard driver controls with one coherent odometry-based moving-shot solution per loop. */
public abstract class SotmTeleOp extends mainTeleOp {
    private static final double RATE_LOOKAHEAD_SECONDS = 0.01;
    private ShootingOnMove.Solution solution = ShootingOnMove.Solution.invalid("Waiting for START");
    private HiveTarget goal;
    private double turretRate = Double.NaN;
    private double velocityTrim;
    private long lastLoopNanos;
    private long readySinceNanos;
    private String shotStatus = "Waiting for START";

    protected SotmTeleOp(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }

    @Override
    protected Pose getStartingPose() {
        if (SotmConfig.useAutonomousEndPose) return super.getStartingPose();
        if (!Double.isFinite(SotmConfig.startX) || !Double.isFinite(SotmConfig.startY)
                || !Double.isFinite(SotmConfig.startHeadingDegrees)) {
            throw new IllegalArgumentException("Set a finite SOTM starting pose");
        }
        return new Pose(SotmConfig.startX, SotmConfig.startY,
                Math.toRadians(SotmConfig.startHeadingDegrees));
    }

    @Override
    protected double getStartingTurretDegrees() {
        double handoffDegrees = super.getStartingTurretDegrees();
        return SotmConfig.useAutonomousEndPose ? handoffDegrees : 0;
    }

    @Override
    protected void startDrive() {
        // updateDrive owns the update so the shot uses this loop's localization sample.
    }

    @Override
    protected void updateDrive() {
        Follower follower = robot.Drivetrain().getFollower();
        follower.manual(-gamepad1.right_stick_y, -gamepad1.right_stick_x,
                -gamepad1.left_stick_x * 0.75);
        follower.update();
    }

    @Override
    protected void startTurretAim() {
        robot.Turret().useMovingAim(() -> solution.turretDegrees, () -> turretRate);
    }

    @Override
    protected void configureAdditionalControls(CommandGamepad driver, CommandGamepad driver2) {
        driver2.dpadUp().onTrue(robot.Turret().stopTurret);
        driver2.dpadRight().onTrue(robot.Turret().aimTurret);
    }

    @Override
    protected void adjustVelocity(double change) {
        velocityTrim += change;
        readySinceNanos = 0;
    }

    @Override
    protected double getShooterTargetVelocity() {
        return Double.isFinite(targetVelocity) && targetVelocity > 0 && solution.valid
                ? solution.flywheelVelocity + velocityTrim : 0;
    }

    @Override
    protected void updateTurretAim() {
        long now = System.nanoTime();
        double elapsedMs = lastLoopNanos == 0 ? 0 : (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;
        turretRate = Double.NaN;
        solution = calculateSolution(elapsedMs);
        updateReadiness(now);
    }

    private ShootingOnMove.Solution calculateSolution(double elapsedMs) {
        if (!validLimits()) return ShootingOnMove.Solution.invalid("Invalid SOTM limits / tolerances");
        if (elapsedMs > SotmConfig.maxLoopMs) {
            return ShootingOnMove.Solution.invalid("Loop delayed; reacquiring");
        }
        Follower follower = robot.Drivetrain().getFollower();
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        if (pose == null || velocity == null || !Double.isFinite(pose.y())) {
            return ShootingOnMove.Solution.invalid("No valid drivetrain state");
        }
        if (Math.hypot(velocity.vx, velocity.vy) > SotmConfig.maxRobotSpeedInchesPerSecond
                || Math.abs(Math.toDegrees(velocity.omega)) > SotmConfig.maxRobotTurnDegreesPerSecond) {
            return ShootingOnMove.Solution.invalid("Robot motion above SOTM limits");
        }
        HiveTarget previousGoal = goal;
        if (goal == null) goal = getAlliance().targetForY(pose.y(), TurretConfig.halfwayY);
        else if (goal == getAlliance().close
                && pose.y() > TurretConfig.halfwayY + SotmConfig.hiveSwitchHysteresisInches) {
            goal = getAlliance().far;
        } else if (goal == getAlliance().far
                && pose.y() < TurretConfig.halfwayY - SotmConfig.hiveSwitchHysteresisInches) {
            goal = getAlliance().close;
        }
        if (goal == null) return ShootingOnMove.Solution.invalid("Invalid hive selection");
        if (goal != previousGoal) readySinceNanos = 0;
        ShootingOnMove.Model model = new ShootingOnMove.Model(SotmConfig.launchAngleDegrees,
                SotmConfig.goalMinusReleaseHeightInches, SotmConfig.shooterForwardInches,
                SotmConfig.shooterLeftInches, SotmConfig.releaseDelaySeconds);
        ShootingOnMove.Solution shot = solve(pose, velocity, model, 0);
        if (!shot.valid) return shot;
        if (shot.distanceInches < SotmConfig.minDistanceInches
                || shot.distanceInches > SotmConfig.maxDistanceInches) {
            return ShootingOnMove.Solution.invalid("Distance outside SOTM limits");
        }
        double flywheel = shot.flywheelVelocity + velocityTrim;
        if (!Double.isFinite(flywheel) || flywheel <= 0 || flywheel > SotmConfig.maxFlywheelVelocity) {
            return ShootingOnMove.Solution.invalid("Flywheel target outside SOTM limits");
        }
        double reachable = Calculations.reachableAngle(shot.turretDegrees, robot.Turret().getDegrees(),
                TurretConfig.minDegrees, TurretConfig.maxDegrees);
        if (!Double.isFinite(reachable)) {
            return ShootingOnMove.Solution.invalid("Shot outside turret travel limits");
        }
        ShootingOnMove.Solution next = solve(pose, velocity, model, RATE_LOOKAHEAD_SECONDS);
        if (!next.valid) return ShootingOnMove.Solution.invalid("Shot leaving ballistic envelope");
        turretRate = Calculations.normalize(next.turretDegrees - shot.turretDegrees)
                / RATE_LOOKAHEAD_SECONDS;
        if (Math.abs(turretRate) > SotmConfig.maxTurretSpeedDegreesPerSecond) {
            return ShootingOnMove.Solution.invalid("Required turret rate above SOTM limit");
        }
        return shot;
    }

    private ShootingOnMove.Solution solve(Pose pose, Velocity velocity,
                                          ShootingOnMove.Model model, double dt) {
        return ShootingOnMove.solve(pose.x() + velocity.vx * dt, pose.y() + velocity.vy * dt,
                pose.heading() + velocity.omega * dt, velocity.vx, velocity.vy, velocity.omega,
                goal.x, goal.y, model);
    }

    private static boolean validLimits() {
        double[] values = {SotmConfig.minDistanceInches, SotmConfig.maxDistanceInches,
                SotmConfig.maxFlywheelVelocity, SotmConfig.maxRobotSpeedInchesPerSecond,
                SotmConfig.maxRobotTurnDegreesPerSecond, SotmConfig.maxTurretSpeedDegreesPerSecond,
                SotmConfig.flywheelTolerance, SotmConfig.turretRateToleranceDegreesPerSecond,
                SotmConfig.readyDwellMs, SotmConfig.maxLoopMs, SotmConfig.hiveSwitchHysteresisInches,
                TurretConfig.halfwayY, TurretConfig.toleranceDegrees};
        for (double value : values) if (!Double.isFinite(value) || value < 0) return false;
        return SotmConfig.maxDistanceInches > SotmConfig.minDistanceInches
                && SotmConfig.maxFlywheelVelocity > 0 && SotmConfig.maxLoopMs > 0;
    }

    private void updateReadiness(long now) {
        double desiredSpeed = getShooterTargetVelocity();
        double desiredAngle = solution.valid ? Calculations.reachableAngle(solution.turretDegrees,
                robot.Turret().getDegrees(), TurretConfig.minDegrees, TurretConfig.maxDegrees) : Double.NaN;
        double speedError = Math.abs(robot.Shooter().getFlywheelVelocity() - desiredSpeed);
        double angleError = Math.abs(desiredAngle - robot.Turret().getDegrees());
        double rateError = Math.abs(turretRate - robot.Turret().getVelocityDegreesPerSecond());
        if (!solution.valid) shotStatus = solution.status;
        else if (desiredSpeed <= 0) shotStatus = "Shooter off";
        else if (!robot.Turret().isTrackingMovingAim()) shotStatus = "Turret stopped / manual";
        else if (!(angleError <= TurretConfig.toleranceDegrees)) shotStatus = "Waiting for turret aim";
        else if (!(rateError <= SotmConfig.turretRateToleranceDegreesPerSecond)) shotStatus = "Waiting for turret tracking rate";
        else if (!(speedError <= SotmConfig.flywheelTolerance)) shotStatus = "Waiting for flywheel speed";
        else {
            if (readySinceNanos == 0) readySinceNanos = now;
            shotStatus = (now - readySinceNanos) / 1e6 >= SotmConfig.readyDwellMs ? "READY" : "Settling";
            return;
        }
        readySinceNanos = 0;
    }

    @Override
    public void disabledPeriodic() {
        telemetry.addData("Status", SotmConfig.useAutonomousEndPose
                ? "SOTM INIT - motors stopped; preserve autonomous turret angle"
                : "SOTM INIT - motors stopped; center turret before START");
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Launch angle / goal minus release height", "%.1f deg / %.1f in",
                SotmConfig.launchAngleDegrees, SotmConfig.goalMinusReleaseHeightInches);
        telemetry.addData("Starting pose source", SotmConfig.useAutonomousEndPose ? "AUTO HANDOFF" : "SotmConfig start pose");
        telemetry.addData("Standalone pose", "x=%.2f y=%.2f heading=%.1f deg",
                SotmConfig.startX, SotmConfig.startY, SotmConfig.startHeadingDegrees);
        telemetry.addLine("Set actual launch angle, height difference and pose in SotmConfig");
        telemetry.addLine("D1 Up: shooter on; Left/Right: trim -/+25; hold RB: manual feed");
        telemetry.addLine("D2 Down: shooter off; Up: turret stop; Right: aim; Square: recenter");
        telemetry.update();
    }

    @Override
    protected void addAdditionalDriverTelemetry() {
        Pose pose = robot.Drivetrain().getFollower().pose();
        Velocity velocity = robot.Drivetrain().getFollower().velocity();
        addDriverData("SOTM solution", solution.status);
        addDriverData("SOTM shot readiness", shotStatus);
        addDriverData("SOTM hive", goal);
        addDriverData("SOTM range / flight", String.format(Locale.US, "%.1f in / %.3f s",
                solution.distanceInches, solution.flightSeconds));
        addDriverData("SOTM turret rate / speed trim", String.format(Locale.US, "%.1f deg/s / %+.0f ticks/s",
                turretRate, velocityTrim));
        addDriverData("SOTM pose", pose);
        addDriverData("SOTM field velocity", velocity);
    }
}
