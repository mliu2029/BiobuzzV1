package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;
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
    public static volatile double transferP = Transfer.DEFAULT_P;
    public static volatile double transferI = Transfer.DEFAULT_I;
    public static volatile double transferD = Transfer.DEFAULT_D;
    public static volatile double transferF = Transfer.DEFAULT_F;
    public static volatile double transferTargetVelocity = Transfer.DEFAULT_TARGET_VELOCITY;
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
        robot.Transfer().setPIDFCoefficients(transferP, transferI, transferD, transferF);
        robot.Transfer().setVelocity(transferTargetVelocity);

        if(runIntake){
            robot.Intake().runIntake.schedule();
        }else{
            robot.Intake().stopIntake.schedule();
        }
        (openTransfer ? robot.Transfer().openBlocker : robot.Transfer().closeBlocker).schedule();

        robot.Shooter().setVelocity(targetVelocity);
        panelsTelemetry.addData("Target Velocity (rad/s)", robot.Shooter().isEnabled()
                ? robot.Shooter().getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Flywheel Velocity (rad/s)", robot.Shooter().getFlywheelVelocity());
        panelsTelemetry.addData("Transfer Target Velocity (rad/s)", robot.Transfer().isEnabled()
                ? robot.Transfer().getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Roller Velocity (rad/s)", robot.Transfer().getRollerVelocity());
        panelsTelemetry.addData("Flywheel Power", robot.Shooter().getFlywheelPower());
        panelsTelemetry.addData("Roller Power", robot.Transfer().getRollerPower());
        panelsTelemetry.addData("Transfer Servo Position", robot.Transfer().getPosition());
        panelsTelemetry.addLine(String.format(Locale.US,
                "Shooter PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f", p, i, d, f));
        panelsTelemetry.addLine(String.format(Locale.US,
                "Transfer PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f",
                transferP, transferI, transferD, transferF));
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
