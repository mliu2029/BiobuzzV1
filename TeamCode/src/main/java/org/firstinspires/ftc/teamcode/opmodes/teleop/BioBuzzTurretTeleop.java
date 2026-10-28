package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.TurretConfig;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

/** Alliance turret test with automatic aiming and the standard gamepad 1 drivetrain controls. */
@Configurable
public abstract class BioBuzzTurretTeleop extends TeleOp {
    public static volatile double p = Shooter.DEFAULT_P;
    public static volatile double i = Shooter.DEFAULT_I;
    public static volatile double d = Shooter.DEFAULT_D;
    public static volatile double f = Shooter.DEFAULT_F;
    public static volatile double targetVelocity = 0.0;
    public static volatile boolean runIntake = false;
    public static volatile boolean reverseIntake = false;
    public static volatile boolean openTransfer = false;

    private final Alliance alliance;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private double shooterPresetVelocity;

    protected BioBuzzTurretTeleop(Robot robot, Alliance alliance) {
        super(robot);
        this.alliance = alliance;
        resetMechanismControls();
        panelsTelemetry.setUpdateInterval(50);
    }

    @Override
    protected void configureMechanismControls(CommandGamepad driver) {
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
        driver.dpadDown().onTrue(Commands.instant(() -> targetVelocity = 0));
        driver.dpadRight().onTrue(Commands.instant(() -> adjustVelocity(25)));
        driver.dpadLeft().onTrue(Commands.instant(() -> adjustVelocity(-25)));
    }

    @Override
    protected void configureTransferControls(CommandGamepad driver) {
        driver.rightBumper().onTrue(Commands.instant(() -> openTransfer = true));
        driver.rightBumper().onFalse(Commands.instant(() -> openTransfer = false));
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
        telemetry.addLine("START restores the turret angle saved by auto; otherwise starts at 0 degrees.");
        addControlTelemetry();
        telemetry.addLine("PLAY starts driving and automatic turret aiming.");
        telemetry.addData("TeleOp starting pose", OpModeStorage.autonomousEndPose);
        telemetry.addData("TeleOp starting turret (degrees)", OpModeStorage.autonomousEndTurretDegrees);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Limelight", "Not initialized until START");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start(); // Restore the pose and start continuous gamepad 1 driving.
        shooterPresetVelocity = robot.Shooter().getTargetVelocity();
        runIntake = false;
        reverseIntake = false;
        openTransfer = false;
        robot.Turret().initialize(hardwareMap, alliance,
                OpModeStorage.consumeAutonomousEndTurretDegrees());
        CommandGamepad operator = new CommandGamepad(gamepad2);
        operator.dpadUp().onTrue(robot.Turret().aimTurret);
        operator.circle().onTrue(robot.Turret().stopTurret);
        robot.Turret().aimTurret.schedule();
    }

    @Override
    public void periodic() {
        if (!started) return;
        robot.Shooter().setPIDFCoefficients(p, i, d, f);
        robot.Shooter().setVelocity(targetVelocity);
        if (Double.isFinite(targetVelocity) && targetVelocity > 0) {
            shooterPresetVelocity = targetVelocity;
        }
        (runIntake
                ? (reverseIntake ? robot.Intake().runIntakeReverse : robot.Intake().runIntake)
                : robot.Intake().stopIntake).schedule();
        (openTransfer ? robot.Transfer().openBlocker : robot.Transfer().closeBlocker).schedule();
        addControlTelemetry();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Aiming at: ", robot.Turret().getTarget());
        telemetry.addData("Aim mode", robot.Turret().getMode());
        telemetry.addData("Limelight", robot.Limelight().isAvailable()
                ? "Configured" : "Unavailable (check USB/config)");
        telemetry.addData("Turret degrees", robot.Turret().getDegrees());
        telemetry.addData("Turret encoder ticks", robot.Turret().getEncoderTicks());
        telemetry.addData("Turret ticks/degree", TurretConfig.ticksPerDegree);
        telemetry.addData("Target degrees", robot.Turret().getTargetDegrees());
        telemetry.addData("Aligned: ", robot.Turret().isAligned());
        telemetry.addData("Pose", robot.Drivetrain().getFollower().pose());
        panelsTelemetry.addData("Alliance", alliance);
        panelsTelemetry.addData("Aim mode", robot.Turret().getMode());
        panelsTelemetry.addData("Turret degrees", robot.Turret().getDegrees());
        panelsTelemetry.addData("Turret encoder ticks", robot.Turret().getEncoderTicks());
        panelsTelemetry.addData("Turret ticks/degree", TurretConfig.ticksPerDegree);
        panelsTelemetry.addData("Target degrees", robot.Turret().getTargetDegrees());
        panelsTelemetry.addData("Intake", runIntake ? (reverseIntake ? "Reverse" : "On") : "Off");
        panelsTelemetry.addData("Target Velocity (rad/s)", robot.Shooter().isEnabled()
                ? robot.Shooter().getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Flywheel Velocity (rad/s)", robot.Shooter().getFlywheelVelocity());
        panelsTelemetry.addData("Roller Velocity (rad/s)", robot.Transfer().getRollerVelocity());
        panelsTelemetry.addData("Flywheel Power", robot.Shooter().getFlywheelPower());
        panelsTelemetry.addData("Roller Power", robot.Transfer().getRollerPower());
        panelsTelemetry.addData("Transfer Servo Position", robot.Transfer().getPosition());
        panelsTelemetry.addData("Shooter P", p);
        panelsTelemetry.addData("Shooter I", i);
        panelsTelemetry.addData("Shooter D", d);
        panelsTelemetry.addData("Shooter F", f);
        panelsTelemetry.update();
        super.periodic();
    }

    @Override
    public void end() {
        resetMechanismControls();
        super.end();
    }

    private void addControlTelemetry() {
        telemetry.addLine("Gamepad 1: right stick drives/strafes; left stick X turns.");
        telemetry.addLine("Gamepad 1: Circle intake; Square reverse; Cross stops intake.");
        telemetry.addLine("Gamepad 1: D-pad up/down starts/stops shooter; right/left changes speed by 25 rad/s.");
        telemetry.addLine("Gamepad 1: Hold right bumper to open the blocker and run the transfer roller; release to stop both.");
        telemetry.addLine("Panels > BioBuzzTurretTeleop: targetVelocity (0 stops), runIntake, reverseIntake, openTransfer, shooter p/i/d/f.");
        telemetry.addLine("Gamepad 2: Circle stops auto aim; D-pad up resumes it.");
    }
}
