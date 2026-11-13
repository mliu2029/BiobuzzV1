package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.TurretConfig;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import java.util.Locale;

@Configurable
public abstract class mainTeleOp extends NextOpMode {
    private static final int TELEMETRY_INTERVAL_MS = 200;
    private static final double FIELD_SIZE_INCHES = 141.5;
    public static volatile double targetVelocity = 0.0;
    public static volatile boolean runIntake = false;
    public static volatile boolean reverseIntake = false;
    public static volatile boolean openTransfer = false;
    public static volatile double redPoseResetX = 8;
    public static volatile double redPoseResetY = 8;
    public static volatile double redPoseResetHeadingDegrees = 90;

    protected final Robot robot;
    private final Alliance alliance;
    private boolean started;
    private boolean wasTransferOpen;
    private final TelemetryManager panelsTelemetry =
        PanelsTelemetry.INSTANCE.getTelemetry();
    private double shooterPresetVelocity;
    private long lastTelemetryNanos;
    private String poseResetStatus = "D2 D-pad Left at the marked pose" ;

    protected mainTeleOp(Robot robot, Alliance alliance) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        this.alliance = alliance;
        resetMechanismControls();
        robot.prepareForInit(hardwareMap);
        panelsTelemetry.setUpdateInterval(TELEMETRY_INTERVAL_MS);
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void configureControls() {
        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.triangle().onTrue(robot.Ramp().toggle);
        driver.circle().onTrue(Commands.instant(() -> {
            reverseIntake = false;
            runIntake = true;
            robot.Intake().restart();
        }));
        driver.square().onTrue(Commands.instant(() -> {
            reverseIntake = true;
            runIntake = true;
        }));
        driver.cross().onTrue(Commands.instant(() -> runIntake = false));
        driver.dpadUp().onTrue(Commands.instant(() -> setShooterTargetVelocity(shooterPresetVelocity)));

        driver.dpadRight().onTrue(Commands.instant(() -> adjustVelocity(25)));
        driver.dpadLeft().onTrue(Commands.instant(() -> adjustVelocity(-25)));
        driver.rightBumper().onTrue(Commands.instant(() -> openTransfer = true));
        driver.rightBumper().onFalse(Commands.instant(() -> openTransfer = false));

        CommandGamepad driver2 = new CommandGamepad(gamepad2);
        driver2.triangle().onTrue(Commands.instant(() -> runIntake = false));
        driver2.square().onTrue(Commands.instant(robot.Turret()::toggleManualRecenter));
        driver2.circle().onTrue(Commands.instant(() ->
                Transfer.servoFreeMode = !Transfer.servoFreeMode));
        driver2.dpadDown().onTrue(Commands.instant(() -> setShooterTargetVelocity(0)));
        driver2.dpadLeft().onTrue(Commands.instant(this::resetPoseAtMark));
        configureAdditionalControls(driver, driver2);
    }

    private void resetPoseAtMark() {
        double x = redPoseResetX;
        double y = redPoseResetY;
        double headingDegrees = redPoseResetHeadingDegrees;
        if (!(x >= 0 && x <= FIELD_SIZE_INCHES && y >= 0 && y <= FIELD_SIZE_INCHES)
                || !Double.isFinite(headingDegrees)) {
            poseResetStatus = "Invalid marked pose in Panels";
            return;
        }
        if (getAlliance() == Alliance.BLUE) {
            x = FIELD_SIZE_INCHES - x;
            y = FIELD_SIZE_INCHES - y;
            headingDegrees += 180;
        }
        headingDegrees = (headingDegrees % 360 + 360) % 360;
        robot.Drivetrain().getFollower().setPose(new Pose(x, y, Math.toRadians(headingDegrees)));
        robot.Turret().resetOdometryAimCorrection();
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
        robot.Turret().aimTurret.schedule();
    }

    protected void updateTurretAim() { }

    protected void startDrive() { robot.startDrive(gamepad1); }

    protected void updateDrive() { }

    protected void addAdditionalDriverTelemetry() { }

    protected double getShooterTargetVelocity() { return targetVelocity; }

    protected void setShooterTargetVelocity(double velocity) { targetVelocity = velocity; }

    protected void adjustVelocity(double change) {
        double velocity = getShooterTargetVelocity();
        shooterPresetVelocity = Math.max(0,
                (velocity > 0 ? velocity : shooterPresetVelocity) + change);
        if (velocity > 0) setShooterTargetVelocity(shooterPresetVelocity);
    }

    private static void resetMechanismControls() {
        targetVelocity = 0;
        runIntake = false;
        reverseIntake = false;
        openTransfer = false;
    }

    @Override
    public void disabledPeriodic() {
        if (!telemetryDue(System.nanoTime())) return;
        telemetry.addData("Status", "INIT");
        telemetry.addData("Alliance", getAlliance());
        telemetry.update();
    }

    @Override
    public void start() {
        Pose startingPose = getStartingPose();
        telemetry.clearAll();
        robot.initialize(hardwareMap);
        shooterPresetVelocity = robot.Shooter().getTargetVelocity();
        runIntake = false;
        reverseIntake = false;
        openTransfer = false;
        robot.Transfer().setRollerVelocity(900);
        wasTransferOpen = false;
        poseResetStatus = "D2 D-pad Left at the marked pose";
        robot.Transfer().setFeeding(false);
        robot.Drivetrain().getFollower().setPose(startingPose);
        robot.Drivetrain().getFollower().update();
        robot.Turret().initialize(hardwareMap, getAlliance(), getStartingTurretDegrees());
        configureControls();
        startDrive();
        lastTelemetryNanos = 0;
        started = true;
        startTurretAim();
    }

    @Override
    public void periodic() {
        if (!started) return;
        updateDrive();
        updateTurretAim();
        robot.Turret().setManualInput(gamepad2.left_bumper, gamepad2.right_bumper);
        double shooterVelocity = getShooterTargetVelocity();
        robot.Shooter().setVelocity(shooterVelocity);
        if (openTransfer && !wasTransferOpen) {
            reverseIntake = false;
            runIntake = true;
            robot.Intake().restart();
        }
        wasTransferOpen = openTransfer;
        robot.Transfer().setFeeding(openTransfer);
        if (Double.isFinite(shooterVelocity) && shooterVelocity > 0) {
            shooterPresetVelocity = shooterVelocity;
        }
        robot.Intake().setVelocity(runIntake
                ? (reverseIntake ? -1 : 1) * Intake.DEFAULT_TARGET_VELOCITY : 0);
        if (telemetryDue(System.nanoTime())) updateDriverTelemetry();
    }

    private boolean telemetryDue(long now) {
        if (lastTelemetryNanos != 0
                && now - lastTelemetryNanos < TELEMETRY_INTERVAL_MS * 1_000_000L) return false;
        lastTelemetryNanos = now;
        return true;
    }

    private void updateDriverTelemetry() {
        double turretDegrees = robot.Turret().getDegrees();
        double turretTarget = robot.Turret().getTargetDegrees();
        int selectedTag = robot.Turret().getVisionTagId();
        addDriverData("Alliance", getAlliance());
        addDriverData("Pose reset", poseResetStatus);
        addDriverData("Aim", robot.Turret().getMode()
                + (robot.Turret().isAligned() ? " | ENCODER ALIGNED" : " | ENCODER NOT ALIGNED"));
        addDriverData("Turret motion", robot.Turret().getMotionStatus());
        addDriverData("Vision", robot.Turret().getVisionStatus());
        int errorTag = robot.Turret().getVisionErrorTagId();
        addDriverData("Vision aim (tag; error deg)", errorTag < 0 ? "NO USABLE TAG"
                : errorTag + "; " + oneDecimal(robot.Turret().getVisionErrorDegrees())
                + (robot.Turret().isVisionAligned() ? " | ALIGNED" : " | NOT ALIGNED"));
        addDriverData("Tags (seen / tracking)", robot.Turret().getVisionTags()
                + " / " + (selectedTag < 0 ? "none" : selectedTag));
        addDriverData("Camera (pipeline / expected; age)", robot.Turret().getVisionPipeline()
                + " / " + TurretConfig.pipeline + "; "
                + oneDecimal(robot.Turret().getVisionFrameAgeMs()) + " ms");
        double cameraTimestamp = robot.Turret().getVisionTimestamp();
        addDriverData("Camera frame timestamp", Double.isFinite(cameraTimestamp)
                ? oneDecimal(cameraTimestamp) + (cameraTimestamp > 0 ? " ms" : " (using hub receive time)")
                : robot.Turret().getVisionTagId() >= 0 ? "Invalid; using hub receive time" : "No tracked frame");
        addDriverData("Turret (deg actual / target / error)", oneDecimal(turretDegrees)
                + " / " + oneDecimal(turretTarget) + " / " + oneDecimal(turretTarget - turretDegrees));
        addDriverData("Turret (cmd / encoder)", String.format(Locale.US, "%.0f%% / %d ticks",
                robot.Turret().getPower() * 100, robot.Turret().getEncoderTicks()));
        addDriverData("Recenter (old deg -> 0; count)", robot.Turret().getRecenterCount() == 0 ? "None yet"
                : String.format(Locale.US, "%+.1f -> 0; %d", robot.Turret().getLastRecenterDegrees(),
                        robot.Turret().getRecenterCount()));
        addDriverData("Shooter (ticks/s actual / target)", String.format(Locale.US, "%.0f / %.0f",
                robot.Shooter().getFlywheelVelocity(), robot.Shooter().getTargetVelocity()));
        addDriverData("Shooter (control / cmd)", robot.Shooter().getControlStatus()
                + String.format(Locale.US, " / %.0f%%", robot.Shooter().getFlywheelPower() * 100));
        addDriverData("Intake (state / cmd)", robot.Intake().getStatus()
                + String.format(Locale.US, " / %.0f%%", robot.Intake().getPower() * 100));
        addDriverData("Intake (ticks/s actual / target)", String.format(Locale.US, "%.0f / %.0f",
                robot.Intake().getVelocity(), robot.Intake().getTargetVelocity()));
        addDriverData("Transfer", robot.Transfer().getStatus());
        addDriverData("Ramp command", robot.Ramp().getStatus());
        addDriverData("Rollers (ticks/s actual / target; cmd)", String.format(Locale.US, "%.0f / %.0f; %.0f%%",
                robot.Transfer().getRollerVelocity(), robot.Transfer().getCommandedRollerVelocity(),
                robot.Transfer().getRollerPower() * 100));
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
        started = false;
        resetMechanismControls();
        setShooterTargetVelocity(0);
        robot.stopImmediately();
    }

}
