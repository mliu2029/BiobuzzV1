package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.TurretConfig;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import java.util.Locale;
import java.util.List;

/** Shared main TeleOp for red and blue, with driving and Limelight turret aiming. */
@Configurable
public abstract class BioBuzzTurretTeleop extends NextOpMode {
    private static final int TELEMETRY_INTERVAL_MS = 200;
    public static volatile double p = Shooter.DEFAULT_P;
    public static volatile double i = Shooter.DEFAULT_I;
    public static volatile double d = Shooter.DEFAULT_D;
    public static volatile double f = Shooter.DEFAULT_F;
    public static volatile double targetVelocity = 0.0;
    public static volatile boolean runIntake = false;
    public static volatile boolean reverseIntake = false;
    public static volatile boolean openTransfer = false;

    private final Robot robot;
    private final Alliance alliance;
    private boolean started;
    private final TelemetryManager panelsTelemetry =
        PanelsTelemetry.INSTANCE.getTelemetry();
    private double shooterPresetVelocity;
    private final List<VoltageSensor> voltageSensors;
    private long lastTelemetryNanos;
    private long lastLoopNanos;
    private double loopTotalMs;
    private int loopSamples;
    private double peakLoopMs;
    private double minimumVoltage = Double.NaN;

    protected BioBuzzTurretTeleop(Robot robot, Alliance alliance) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        this.alliance = alliance;
        resetMechanismControls();
        robot.prepareForInit(hardwareMap);
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);
        panelsTelemetry.setUpdateInterval(TELEMETRY_INTERVAL_MS);
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        telemetry.addData("Status", "INIT - motors stopped; press START");
        telemetry.update();
    }

    private void configureControls() {
        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.circle().onTrue(Commands.instant(() -> {
            reverseIntake = false;
            runIntake = true;
        }));
        driver.square().onTrue(Commands.instant(() -> {
            reverseIntake = true;
            runIntake = true;
        }));
        driver.cross().onTrue(Commands.instant(() -> runIntake = false));
        driver.dpadUp().onTrue(Commands.instant(() -> targetVelocity = shooterPresetVelocity));

        driver.dpadRight().onTrue(Commands.instant(() -> adjustVelocity(25)));
        driver.dpadLeft().onTrue(Commands.instant(() -> adjustVelocity(-25)));
        driver.rightBumper().onTrue(Commands.instant(() -> openTransfer = true));
        driver.rightBumper().onFalse(Commands.instant(() -> openTransfer = false));

        CommandGamepad driver2 = new CommandGamepad(gamepad2);
        driver2.triangle().onTrue(Commands.instant(() -> runIntake = false));
        driver2.square().onTrue(Commands.instant(robot.Turret()::toggleManualRecenter));
//        driver2.dpadUp().onTrue(robot.Turret().aimTurret);
        driver2.circle().onTrue(Commands.instant(() ->
                Transfer.servoFreeMode = !Transfer.servoFreeMode));
        driver2.dpadDown().onTrue(Commands.instant(() -> targetVelocity = 0));

    }

    private void setTransferOpen(boolean open) {
        robot.Transfer().setRollerVelocity(900);
        if (open) {
            robot.Transfer().openImmediately();
        } else {
            robot.Transfer().closeImmediately();
        }
    }
    private void adjustVelocity(double change) {
        shooterPresetVelocity = Math.max(0,
                (targetVelocity > 0 ? targetVelocity : shooterPresetVelocity) + change);
        if (targetVelocity > 0) targetVelocity = shooterPresetVelocity;
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
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.clearAll();
        robot.initialize(hardwareMap);
        shooterPresetVelocity = robot.Shooter().getTargetVelocity();
        runIntake = false;
        reverseIntake = false;
        openTransfer = false;
        setTransferOpen(false);
        Pose startingPose = OpModeStorage.autonomousEndPose;
        robot.Drivetrain().getFollower().setPose(
                startingPose != null ? startingPose : Pose.zero());
        robot.Drivetrain().getFollower().update();
        robot.Turret().initialize(hardwareMap, alliance,
                OpModeStorage.consumeAutonomousEndTurretDegrees());
        configureControls();
        robot.startDrive(gamepad1);
        lastTelemetryNanos = 0;
        lastLoopNanos = 0;
        loopTotalMs = 0;
        loopSamples = 0;
        peakLoopMs = 0;
        minimumVoltage = Double.NaN;
        started = true;
        robot.Turret().aimTurret.schedule();
    }

    @Override
    public void periodic() {
        if (!started) return;
        // Entry-to-entry timing includes the framework's motor, scheduler, and hardware work.
        long now = System.nanoTime();
        if (lastLoopNanos != 0) {
            double elapsedMs = (now - lastLoopNanos) / 1_000_000.0;
            loopTotalMs += elapsedMs;
            loopSamples++;
            peakLoopMs = Math.max(peakLoopMs, elapsedMs);
        }
        lastLoopNanos = now;
        robot.Turret().setManualInput(gamepad2.left_bumper, gamepad2.right_bumper);
        robot.Shooter().setPIDFCoefficients(p, i, d, f);
        robot.Shooter().setVelocity(targetVelocity);
        setTransferOpen(openTransfer);
        if (Double.isFinite(targetVelocity) && targetVelocity > 0) {
            shooterPresetVelocity = targetVelocity;
        }
        (runIntake
                ? (reverseIntake ? robot.Intake().runIntakeReverse : robot.Intake().runIntake)
                : robot.Intake().stopIntake).schedule();
        if (telemetryDue(now)) updateDriverTelemetry();
    }

    private boolean telemetryDue(long now) {
        if (lastTelemetryNanos != 0
                && now - lastTelemetryNanos < TELEMETRY_INTERVAL_MS * 1_000_000L) return false;
        lastTelemetryNanos = now;
        return true;
    }

    private void updateDriverTelemetry() {
        double voltage = readVoltage();
        if (Double.isFinite(voltage)) {
            minimumVoltage = Double.isFinite(minimumVoltage)
                    ? Math.min(minimumVoltage, voltage) : voltage;
        }
        double turretDegrees = robot.Turret().getDegrees();
        double turretTarget = robot.Turret().getTargetDegrees();
        int selectedTag = robot.Turret().getVisionTagId();
        addDriverData("Alliance", alliance);
        addDriverData("Battery (V now / min)", oneDecimal(voltage) + " / " + oneDecimal(minimumVoltage));
        addDriverData("Loop (ms avg / peak)", String.format(Locale.US, "%.1f / %.1f",
                loopSamples == 0 ? 0 : loopTotalMs / loopSamples, peakLoopMs));
        loopTotalMs = 0;
        loopSamples = 0;
        addDriverData("Aim", robot.Turret().getMode()
                + (robot.Turret().isAligned() ? " | ALIGNED" : " | NOT ALIGNED"));
        addDriverData("Vision", robot.Turret().getVisionStatus());
        addDriverData("Tags (seen / tracking)", robot.Turret().getVisionTags()
                + " / " + (selectedTag < 0 ? "none" : selectedTag));
        addDriverData("Camera (pipeline / expected; age)", robot.Turret().getVisionPipeline()
                + " / " + TurretConfig.pipeline + "; "
                + oneDecimal(robot.Turret().getVisionFrameAgeMs()) + " ms");
        addDriverData("Turret (deg actual / target / error)", oneDecimal(turretDegrees)
                + " / " + oneDecimal(turretTarget) + " / " + oneDecimal(turretTarget - turretDegrees));
        addDriverData("Turret (cmd / encoder)", String.format(Locale.US, "%.0f%% / %d ticks",
                robot.Turret().getPower() * 100, robot.Turret().getEncoderTicks()));
        addDriverData("Shooter (ticks/s actual / target)", String.format(Locale.US, "%.0f / %.0f",
                robot.Shooter().getFlywheelVelocity(), robot.Shooter().getTargetVelocity()));
        addDriverData("Shooter (control / cmd)", robot.Shooter().getControlStatus()
                + String.format(Locale.US, " / %.0f%%", robot.Shooter().getFlywheelPower() * 100));
        addDriverData("Intake (state / cmd)", (runIntake ? (reverseIntake ? "Reverse" : "On") : "Off")
                + String.format(Locale.US, " / %.0f%%", robot.Intake().getPower() * 100));
        addDriverData("Transfer", robot.Transfer().getStatus());
        addDriverData("Rollers (ticks/s actual / target; cmd)", String.format(Locale.US, "%.0f / %.0f; %.0f%%",
                robot.Transfer().getRollerVelocity(), robot.Transfer().getCommandedRollerVelocity(),
                robot.Transfer().getRollerPower() * 100));
        panelsTelemetry.update();
        telemetry.update();
    }

    private double readVoltage() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (Double.isFinite(voltage) && voltage > 0) lowest = Math.min(lowest, voltage);
        }
        return Double.isFinite(lowest) ? lowest : Double.NaN;
    }

    private static String oneDecimal(double value) {
        return Double.isFinite(value) ? String.format(Locale.US, "%.1f", value) : "--";
    }

    private void addDriverData(String caption, Object value) {
        telemetry.addData(caption, value);
        panelsTelemetry.addData(caption, value);
    }

    @Override
    public void end() {
        started = false;
        resetMechanismControls();
        robot.stopImmediately();
    }

}
