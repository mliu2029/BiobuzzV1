package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.util.LoopTiming;
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
    public static volatile double rollerP = Transfer.DEFAULT_ROLLER_P;
    public static volatile double rollerI = Transfer.DEFAULT_ROLLER_I;
    public static volatile double rollerD = Transfer.DEFAULT_ROLLER_D;
    public static volatile double rollerF = Transfer.DEFAULT_ROLLER_F;
    public static volatile double rollerTargetVelocity = Transfer.DEFAULT_ROLLER_TARGET_VELOCITY;
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
        robot.Shooter().setPIDFCoefficients(p, i, d, f);
        robot.Transfer().setRollerPIDFCoefficients(rollerP, rollerI, rollerD, rollerF);
        robot.Transfer().setRollerVelocity(rollerTargetVelocity);

        if (openTransfer && !wasTransferOpen) {
            runIntake = true;
            robot.Intake().restart();
        }
        wasTransferOpen = openTransfer;
        robot.Intake().setPower(runIntake ? 1 : 0);
        robot.Transfer().setFeeding(openTransfer);
        robot.Shooter().setVelocity(targetVelocity);
        long now = System.nanoTime();
        if (lastTelemetryNanos != 0 && now - lastTelemetryNanos < 100_000_000L) return;
        lastTelemetryNanos = now;
        panelsTelemetry.addData("Loop ms (avg / p95 / max; last 256)", loopTiming.summary());
        panelsTelemetry.addData("Target Velocity (rad/s)", robot.Shooter().isEnabled()
                ? robot.Shooter().getTargetVelocity() : 0.0);
        panelsTelemetry.addData("Flywheel Velocity (rad/s)", robot.Shooter().getFlywheelVelocity());
        panelsTelemetry.addData("Roller Target Velocity (rad/s)", robot.Transfer().getCommandedRollerVelocity());
        panelsTelemetry.addData("Roller Velocity (rad/s)", robot.Transfer().getRollerVelocity());
        panelsTelemetry.addData("Flywheel Power", robot.Shooter().getFlywheelPower());
        panelsTelemetry.addData("Roller Power", robot.Transfer().getRollerPower());
        panelsTelemetry.addData("Transfer", robot.Transfer().getStatus());
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
