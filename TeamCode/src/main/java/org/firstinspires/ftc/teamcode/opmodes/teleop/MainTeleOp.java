package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.LoopTiming;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import java.util.Locale;

@Configurable
public abstract class MainTeleOp extends NextOpMode {
    private static final int TELEMETRY_INTERVAL_MS = 200;
    public static volatile double redPoseResetX = 8;
    public static volatile double redPoseResetY = 8;
    public static volatile double redPoseResetHeadingDegrees = 90;

    protected final Robot robot;
    private final Alliance alliance;
    private final LoopTiming loopTiming;
    private final TelemetryManager panelsTelemetry =
        PanelsTelemetry.INSTANCE.getTelemetry();
    private double shooterPresetVelocity;
    private long lastTelemetryNanos;
    private String poseResetStatus = "Not reset";

    private boolean wasIntakeStalled;
    private boolean wasTurretAligned;

    protected double targetVelocity = 0.0;

    protected MainTeleOp(Robot robot, Alliance alliance) {
        this(robot, alliance, new LoopTiming());
    }

    private MainTeleOp(Robot robot, Alliance alliance, LoopTiming loopTiming) {
        super(robot, BulkReadHook.INSTANCE, loopTiming);
        this.loopTiming = loopTiming;
        this.robot = robot;
        this.alliance = alliance;
        targetVelocity = 0;
        robot.prepareForInit(hardwareMap);
        panelsTelemetry.setUpdateInterval(TELEMETRY_INTERVAL_MS);
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void configureControls() {
        CommandGamepad driver = new CommandGamepad(gamepad1);
        CommandGamepad driver2 = new CommandGamepad(gamepad2);

        driver.triangle().onTrue(robot.ramp.toggle);

        driver.circle().onTrue(robot.intake.runIntake);
        driver.circle().onFalse(robot.intake.stopIntake);
        driver.square().onTrue(robot.intake.reverseIntake);
        driver.square().onFalse(robot.intake.stopIntake);
        driver.cross().onTrue(robot.intake.stopIntake);

        driver.dpadUp().onTrue(robot.shooter.runFlywheel);
        driver.dpadDown().onTrue(robot.shooter.stopFlywheel);
        driver.dpadRight().onTrue(robot.shooter.adjustRPM(50));
        driver.dpadLeft().onTrue(robot.shooter.adjustRPM(-50));

        driver.rightBumper().onTrue(parallel(robot.intake.runIntake, robot.transfer.feed));
        driver.rightBumper().onFalse(parallel(robot.intake.stopIntake, robot.transfer.stopFeeding));

        driver2.triangle().onTrue(robot.intake.stopIntake);
        driver2.square().onTrue(robot.turret.toggleRecenter);
        driver2.circle().onTrue(robot.transfer.toggleServoFree);
        driver2.dpadDown().onTrue(robot.shooter.stopFlywheel);
        driver2.dpadLeft().onTrue(Commands.instant(this::resetPoseAtMark));

        driver2.leftBumper().onTrue(robot.turret.manualLeft);
        driver2.leftBumper().onFalse(robot.turret.manualStop);
        driver2.rightBumper().onTrue(robot.turret.manualRight);
        driver2.rightBumper().onFalse(robot.turret.manualStop);

        configureAdditionalControls(driver, driver2);
    }

    private void resetPoseAtMark() {
        double x = redPoseResetX;
        double y = redPoseResetY;
        double headingDegrees = redPoseResetHeadingDegrees;
        if (getAlliance() == Alliance.BLUE) {
            x = 141.5 - x;
            y = 141.5 - y;
            headingDegrees += 180;
        }
        headingDegrees = (headingDegrees % 360 + 360) % 360;
        robot.drivetrain.getFollower().setPose(new Pose(x, y, Math.toRadians(headingDegrees)));
        robot.turret.resetOdometryAimCorrection();
        poseResetStatus = String.format(Locale.US, "Reset to (%.1f, %.1f, %.1f deg)",
                x, y, headingDegrees);
    }

    protected void configureAdditionalControls(CommandGamepad driver, CommandGamepad driver2) { }

    protected Alliance getAlliance() { return alliance; }

    protected Pose getStartingPose() {
        Pose startingPose = OpModeStorage.autonomousEndPose;
        return startingPose != null ? startingPose : Pose.zero();
    }

    protected double getStartingTurretDegrees() {
        return OpModeStorage.consumeAutonomousEndTurretDegrees();
    }

    protected void startTurretAim() {
        Follower follower = robot.drivetrain.getFollower();
        robot.turret.useOdometryAim(follower::pose, follower::velocity);
        robot.turret.setOdometryVisionEnabled(true);
    }

    protected void startDrive() { robot.startDrive(gamepad1); }

    protected void updateDrive() { }

    protected void addAdditionalDriverTelemetry() { }

    protected double getShooterTargetVelocity() {
        return robot.shooter.isEnabled() ? robot.shooter.getTargetVelocity() : targetVelocity;
    }

    protected void setShooterTargetVelocity(double velocity) {
        targetVelocity = velocity;
        robot.shooter.setVelocity(velocity);
    }

    protected void adjustVelocity(double change) {
        robot.shooter.adjustVelocity(change);
        targetVelocity = robot.shooter.getTargetVelocity();
        shooterPresetVelocity = targetVelocity;
    }

    @Override
    public void disabledPeriodic() {
        if (!telemetryDue(System.nanoTime())) return;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", getAlliance());
        telemetry.update();
    }

    @Override
    public void start() {
        Pose startingPose = getStartingPose();
        telemetry.clearAll();
        robot.initialize(hardwareMap, getAlliance(), getStartingTurretDegrees());
        robot.drivetrain.getFollower().setPose(startingPose);
        robot.drivetrain.getFollower().update();
        shooterPresetVelocity = robot.shooter.getTargetVelocity();
        robot.transfer.setRollerVelocity(1925);
        poseResetStatus = "Not reset";
        robot.transfer.setFeeding(false);
        wasIntakeStalled = false;
        wasTurretAligned = false;
        configureControls();
        startDrive();
        lastTelemetryNanos = 0;
        startTurretAim();
    }

    @Override
    public void periodic() {
        updateDrive();

        targetVelocity = robot.shooter.isEnabled() ? robot.shooter.getTargetVelocity() : 0.0;
        double shooterVelocity = getShooterTargetVelocity();
        if (robot.shooter.isEnabled()) {
            robot.shooter.setVelocity(shooterVelocity);
            if (Double.isFinite(shooterVelocity) && shooterVelocity > 0) {
                shooterPresetVelocity = shooterVelocity;
            }
        }

        updateHaptics();
        if (telemetryDue(System.nanoTime())) updateDriverTelemetry();
    }

    private void updateHaptics() {
        boolean stalled = robot.intake.isStalled();
        if (stalled && !wasIntakeStalled) {
            gamepad1.rumbleBlips(3);
        }
        wasIntakeStalled = stalled;

        boolean aligned = robot.turret.isAligned();
        if (aligned && !wasTurretAligned && !stalled && !gamepad1.isRumbling()) {
            gamepad1.rumble(300);
        }
        wasTurretAligned = aligned;
    }

    protected final boolean telemetryDue(long now) {
        if (lastTelemetryNanos != 0
                && now - lastTelemetryNanos < TELEMETRY_INTERVAL_MS * 1_000_000L) return false;
        lastTelemetryNanos = now;
        return true;
    }

    private void updateDriverTelemetry() {
        double turretDegrees = robot.turret.getDegrees();
        double turretTarget = robot.turret.getTargetDegrees();
        addDriverData("Loop ms (avg / p95 / max; last 256)", loopTiming.summary());
        addDriverData("Alliance / aim", getAlliance() + " / " + robot.turret.getMode());
        addDriverData("Pose reset", poseResetStatus);
        addDriverData("Turret motion", robot.turret.getMotionStatus());
        addDriverData("Turret encoder", robot.turret.getEncoderConfigurationStatus());
        addDriverData("Vision", robot.turret.getVisionStatus());
        int tag = robot.turret.getVisionTagId();
        addDriverData("Vision tag / error / age", tag < 0 ? "No usable tag"
                : tag + " / " + oneDecimal(robot.turret.getVisionErrorDegrees()) + " deg / "
                        + oneDecimal(robot.turret.getVisionFrameAgeMs()) + " ms");
        addDriverData("Turret deg (actual / target / PID error)", oneDecimal(turretDegrees)
                + " / " + oneDecimal(turretTarget) + " / " + oneDecimal(robot.turret.getPidErrorDegrees()));
        addDriverData("Turret cmd / encoder / recenter count", String.format(Locale.US,
                "%.0f%% / %d ticks / %d", robot.turret.getPower() * 100,
                robot.turret.getEncoderTicks(), robot.turret.getRecenterCount()));
        addDriverData("Shooter RPM (actual / target)", String.format(Locale.US, "%.0f / %.0f",
                robot.shooter.getFlywheelVelocity(), robot.shooter.getTargetVelocity()));
        addDriverData("Intake / transfer", robot.intake.getStatus() + " / " + robot.transfer.getStatus());
        addDriverData("Ramp", robot.ramp.getStatus());
        addDriverData("Haptics", wasIntakeStalled ? "Intake Stalled (Rumbling)" : wasTurretAligned ? "Aligned" : "Idle");
        addAdditionalDriverTelemetry();
        panelsTelemetry.update();
        telemetry.update();
    }

    private static String oneDecimal(double value) {
        return Double.isFinite(value) ? String.format(Locale.US, "%.1f", value) : "--";
    }

    protected final void addDriverData(String caption, Object value) {
        telemetry.addData(caption, value);
        panelsTelemetry.addData(caption, value);
    }

    @Override
    public void end() {
        gamepad1.stopRumble();
        gamepad2.stopRumble();
        wasIntakeStalled = false;
        wasTurretAligned = false;
        targetVelocity = 0;
        robot.stopImmediately();
    }
}
