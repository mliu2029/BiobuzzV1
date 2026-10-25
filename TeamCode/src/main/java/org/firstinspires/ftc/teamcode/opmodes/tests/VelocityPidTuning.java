package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.Robot;
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
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public VelocityPidTuning(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        runIntake = false;
        runTransfer = false;
        robot.initialize(hardwareMap);
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        runIntake = false;
        runTransfer = false;
    }

    @Override
    public void periodic() {
        robot.getShooter().setPIDFCoefficients(p, i, d, f);

        if(runIntake){
            robot.getIntake().runIntake.schedule();
        }else{
            robot.getIntake().stopIntake.schedule();
        }
        // (runTransfer ? robot.getTransfer().openBlocker : robot.getTransfer().closeBlocker).schedule();

        robot.getShooter().setVelocity(targetVelocity);
        panelsTelemetry.addData("Target Velocity (rad/s)", robot.getShooter().isEnabled()
                ? robot.getShooter().getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Flywheel Velocity (rad/s)", robot.getShooter().getFlywheelVelocity());
        panelsTelemetry.addData("Flywheel2 Velocity (rad/s)", robot.getShooter().getRollerVelocity());
        panelsTelemetry.addData("Flywheel Power", robot.getShooter().getFlywheelPower());
        panelsTelemetry.addData("Flywheel2 Power", robot.getShooter().getRollerPower());
        panelsTelemetry.addLine(String.format(Locale.US,
                "PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f", p, i, d, f));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        runIntake = false;
        runTransfer = false;
        robot.stopImmediately();
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
