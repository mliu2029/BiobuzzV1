package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

@Configurable
@NextTeleop(name = "Velocity Tuning", group = "Tuning")
public class VelocityPidTuning extends NextOpMode {
    public static volatile double p = Shooter.DEFAULT_P;
    public static volatile double i = Shooter.DEFAULT_I;
    public static volatile double d = Shooter.DEFAULT_D;
    public static volatile double f = Shooter.DEFAULT_F;
    public static volatile double targetVelocity = 0.0;
    public static volatile double rollerP = Transfer.DEFAULT_ROLLER_P;
    public static volatile double rollerI = Transfer.DEFAULT_ROLLER_I;
    public static volatile double rollerD = Transfer.DEFAULT_ROLLER_D;
    public static volatile double rollerF = Transfer.DEFAULT_ROLLER_F;
    public static volatile double rollerTargetVelocity = Transfer.DEFAULT_ROLLER_TARGET_VELOCITY;
    public static volatile double intakeP = Intake.DEFAULT_P;
    public static volatile double intakeI = Intake.DEFAULT_I;
    public static volatile double intakeD = Intake.DEFAULT_D;
    public static volatile double intakeF = Intake.DEFAULT_F;
    public static volatile double intakeTargetVelocity = Intake.DEFAULT_TARGET_VELOCITY;
    public static volatile boolean runIntake = false;
    public static volatile boolean openTransfer = false;

    private final Robot robot;
    private boolean started;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public VelocityPidTuning(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        runIntake = false;
        openTransfer = false;
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("Ready - press START");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.initialize(hardwareMap);
        runIntake = false;
        openTransfer = false;
        started = true;
    }

    @Override
    public void periodic() {
        if (!started) return;
        robot.Shooter().setPIDFCoefficients(p, i, d, f);
        robot.Transfer().setRollerPIDFCoefficients(rollerP, rollerI, rollerD, rollerF);
        robot.Transfer().setRollerVelocity(rollerTargetVelocity);

        robot.Intake().setPIDFCoefficients(intakeP, intakeI, intakeD, intakeF);
        robot.Intake().setVelocity(runIntake ? intakeTargetVelocity : 0);
        if (openTransfer) {
            robot.Transfer().openImmediately();
        } else {
            robot.Transfer().closeImmediately();
        }
        robot.Shooter().setVelocity(targetVelocity);
        panelsTelemetry.addData("Target Velocity (rad/s)", robot.Shooter().isEnabled()
                ? robot.Shooter().getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Flywheel Velocity (rad/s)", robot.Shooter().getFlywheelVelocity());
        panelsTelemetry.addData("Roller Target Velocity (rad/s)", robot.Transfer().isFeeding()
                ? robot.Transfer().getRollerTargetVelocity() : 0.0);
        panelsTelemetry.addData("Roller Velocity (rad/s)", robot.Transfer().getRollerVelocity());
        panelsTelemetry.addData("Flywheel Power", robot.Shooter().getFlywheelPower());
        panelsTelemetry.addData("Roller Power", robot.Transfer().getRollerPower());
        panelsTelemetry.addData("Intake Target Velocity (rad/s)", robot.Intake().getTargetVelocity());
        panelsTelemetry.addData("Intake Velocity (rad/s)", robot.Intake().getVelocity());
        panelsTelemetry.addData("Intake Power", robot.Intake().getPower());
        panelsTelemetry.addData("Transfer Servo Position", robot.Transfer().getPosition());
        panelsTelemetry.addLine(String.format(Locale.US,
                "Shooter PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f", p, i, d, f));
        panelsTelemetry.addLine(String.format(Locale.US,
                "Roller PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f",
                rollerP, rollerI, rollerD, rollerF));
        panelsTelemetry.addLine(String.format(Locale.US,
                "Intake PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f",
                intakeP, intakeI, intakeD, intakeF));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        started = false;
        runIntake = false;
        openTransfer = false;
        robot.stopImmediately();
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
