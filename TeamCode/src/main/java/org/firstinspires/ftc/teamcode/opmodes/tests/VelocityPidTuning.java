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

@Configurable
@NextTeleop(name = "Velocity Tuning", group = "Tuning")
public class VelocityPidTuning extends NextOpMode {
    public static volatile double p = Shooter.DEFAULT_P;
    public static volatile double i = Shooter.DEFAULT_I;
    public static volatile double d = Shooter.DEFAULT_D;
    public static volatile double f = Shooter.DEFAULT_F;
    public static volatile double targetVelocity = 0.0;
    public static volatile boolean runIntake = false;
    public static volatile boolean runTransfer = false;

    private final Robot robot;
    private boolean started;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public VelocityPidTuning(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        runIntake = false;
        runTransfer = false;
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("Ready - press START");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.initialize(hardwareMap);
        runIntake = false;
        runTransfer = false;
        started = true;
    }

    @Override
    public void periodic() {
        if (!started) return;
        robot.Shooter().setPIDFCoefficients(p, i, d, f);

        if(runIntake){
            robot.Intake().runIntake.schedule();
        }else{
            robot.Intake().stopIntake.schedule();
        }
        // (runTransfer ? robot.getTransfer().openBlocker : robot.getTransfer().closeBlocker).schedule();

        robot.Shooter().setVelocity(targetVelocity);
        panelsTelemetry.addData("Target Velocity (rad/s)", robot.Shooter().isEnabled()
                ? robot.Shooter().getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Flywheel Velocity (rad/s)", robot.Shooter().getFlywheelVelocity());
        panelsTelemetry.addData("Flywheel2 Velocity (rad/s)", robot.Shooter().getRollerVelocity());
        panelsTelemetry.addData("Flywheel Power", robot.Shooter().getFlywheelPower());
        panelsTelemetry.addData("Flywheel2 Power", robot.Shooter().getRollerPower());
        panelsTelemetry.addLine(String.format(Locale.US,
                "PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f", p, i, d, f));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        started = false;
        runIntake = false;
        runTransfer = false;
        robot.stopImmediately();
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
