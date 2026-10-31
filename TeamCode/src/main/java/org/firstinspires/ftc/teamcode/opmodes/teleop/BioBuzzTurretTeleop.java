package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import java.util.Locale;

/** Shared main TeleOp for red and blue, with driving and Limelight turret aiming. */
@Configurable
public abstract class BioBuzzTurretTeleop extends NextOpMode {
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

    protected BioBuzzTurretTeleop(Robot robot, Alliance alliance) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        this.alliance = alliance;
        resetMechanismControls();
        panelsTelemetry.setUpdateInterval(50);
        telemetry.addData("Status", "Ready");
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
        driver2.dpadUp().onTrue(robot.Turret().aimTurret);
        driver2.circle().onTrue(robot.Turret().stopTurret);
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
        telemetry.addData("Status", "Ready");
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void start() {
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
        started = true;
        robot.Turret().aimTurret.schedule();
    }

    @Override
    public void periodic() {
        if (!started) return;
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
        updateDriverTelemetry();
    }

    private void updateDriverTelemetry() {
        addDriverData("Alliance", alliance);
        addDriverData("Aim", robot.Turret().getMode());
        addDriverData("Vision", robot.Turret().getVisionStatus());
        addDriverData("Turret (deg)", String.format(Locale.US, "%.1f", robot.Turret().getDegrees()));
        addDriverData("Aligned", robot.Turret().isAligned() ? "Yes" : "No");
        addDriverData("Shooter (actual / target)", String.format(Locale.US, "%.0f / %.0f rad/s",
                robot.Shooter().getFlywheelVelocity(), robot.Shooter().getTargetVelocity()));
        addDriverData("Intake", runIntake ? (reverseIntake ? "Reverse" : "On") : "Off");
        addDriverData("Transfer", robot.Transfer().isFeeding() ? "Feeding"
                : openTransfer ? "Open" : "Closed");
        panelsTelemetry.update();
        telemetry.update();
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
