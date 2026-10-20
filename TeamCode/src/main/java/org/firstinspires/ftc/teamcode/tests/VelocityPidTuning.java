package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Configurable
@NextTeleop(name = "Velocity Tuning", group = "Tuning")
public class VelocityPidTuning extends NextOpMode {
    private static final double TICKS_PER_REVOLUTION = 28.0;

    public static volatile double p = Shooter.DEFAULT_P;
    public static volatile double i = Shooter.DEFAULT_I;
    public static volatile double d = Shooter.DEFAULT_D;
    public static volatile double f = Shooter.DEFAULT_F;
    public static volatile double targetVelocity = 0.0;
    public static volatile boolean runTransfer = false;

    private final Robot robot;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public VelocityPidTuning(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        runTransfer = false;
        robot.initialize(hardwareMap);
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        runTransfer = false;
    }

    @Override
    public void periodic() {
        // NextFTC software PID plus velocity feedforward; gains use ticks/second.
        Shooter.INSTANCE.setPIDFCoefficients(p, i, d, f);

        boolean feeding = runTransfer;
        Intake.INSTANCE.setPower(feeding ? -0.7 : 0.0).schedule();
        Transfer.INSTANCE.runTransfer(feeding ? 0.8 : 0.0).schedule();

        Shooter.INSTANCE.setVelocity(targetVelocity);
        double currentVelocity = Shooter.INSTANCE.getVelocity();
        panelsTelemetry.addData("Target Velocity (ticks/s)", Shooter.INSTANCE.isEnabled()
                ? Shooter.INSTANCE.getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Measured Velocity (ticks/s)", currentVelocity);
        panelsTelemetry.addData("RPM", currentVelocity * 60.0 / TICKS_PER_REVOLUTION);
        panelsTelemetry.addData("Motor Power", Shooter.INSTANCE.getPower());
        panelsTelemetry.addLine(String.format(Locale.US,
                "PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.5f", p, i, d, f));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        runTransfer = false;
        robot.stopImmediately();
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
