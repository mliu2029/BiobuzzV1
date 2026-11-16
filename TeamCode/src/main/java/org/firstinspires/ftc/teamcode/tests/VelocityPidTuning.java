package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.util.LoopTiming;

@Configurable
@NextTeleop(name = "Velocity PID Tuning", group = "Tuning")
public class VelocityPidTuning extends NextOpMode {
    public static volatile double kP = Shooter.kP;
    public static volatile double kI = Shooter.kI;
    public static volatile double kD = Shooter.kD;
    public static volatile double kV = Shooter.kV;
    public static volatile double targetVelocity = 0.0;

    public static volatile double rollerKP = Transfer.kP;
    public static volatile double rollerKI = Transfer.kI;
    public static volatile double rollerKD = Transfer.kD;
    public static volatile double rollerKV = Transfer.kV;
    public static volatile double rollerTargetVelocity = Transfer.rollerDefaultRPM;

    public static volatile boolean runIntake = false;
    public static volatile boolean openTransfer = false;

    private final Robot robot;
    private boolean started;
    private boolean wasTransferOpen;
    private long lastTelemetryNanos;
    private final LoopTiming loopTiming;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public VelocityPidTuning(Robot robot) {
        this(robot, new LoopTiming());
    }

    private VelocityPidTuning(Robot robot, LoopTiming loopTiming) {
        super(robot, BulkReadHook.INSTANCE, loopTiming);
        this.robot = robot;
        this.loopTiming = loopTiming;
        runIntake = false;
        openTransfer = false;
        panelsTelemetry.setUpdateInterval(100);
        telemetry.setMsTransmissionInterval(100);
        panelsTelemetry.addLine("Ready - press START");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot.initialize(hardwareMap);
        runIntake = false;
        openTransfer = false;
        wasTransferOpen = false;
        lastTelemetryNanos = 0;
        started = true;
    }

    @Override
    public void periodic() {
        if (!started) return;
        robot.shooter.setVelocityConstants(kP, kI, kD, kV);
        robot.transfer.setRollerVelocityConstants(rollerKP, rollerKI, rollerKD, rollerKV);
        robot.transfer.setRollerVelocity(rollerTargetVelocity);

        if (openTransfer && !wasTransferOpen) {
            runIntake = true;
            robot.intake.restart();
        }
        wasTransferOpen = openTransfer;
        robot.intake.setPower(runIntake ? 1 : 0);
        robot.transfer.setFeeding(openTransfer);
        robot.shooter.setVelocity(targetVelocity);
        long now = System.nanoTime();
        if (lastTelemetryNanos != 0 && now - lastTelemetryNanos < 100_000_000L) return;
        lastTelemetryNanos = now;
        panelsTelemetry.addData("Loop ms (avg / p95 / max; last 256)", loopTiming.summary());
        panelsTelemetry.addData("Target Velocity (RPM)", robot.shooter.isEnabled()
                ? robot.shooter.getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Flywheel Velocity (RPM)", robot.shooter.getFlywheelVelocity());
        panelsTelemetry.addData("Roller Target Velocity (RPM)", robot.transfer.getTargetVelocity());
        panelsTelemetry.addData("Roller Velocity (RPM)", robot.transfer.getRollerVelocity());
        panelsTelemetry.addData("Flywheel Power", robot.shooter.getFlywheelPower());
        panelsTelemetry.addData("Roller Power", robot.transfer.getRollerPower());
        panelsTelemetry.addData("Transfer", robot.transfer.getStatus());
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
