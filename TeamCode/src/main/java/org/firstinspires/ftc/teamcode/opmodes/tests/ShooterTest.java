package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.feedforward.SimpleFFCoefficients;
import dev.nextftc.control.feedforward.SimpleFeedforward;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@NextTeleop(name = "Launcher Test", group = "Tuning")
public class ShooterTest extends NextOpMode {
    // usePid false: power is sent straight to the motor. True: PID holds targetVelocity.
    public static volatile boolean usePid = false;
    public static volatile double power = 0.0;
    public static volatile double targetVelocity = 0.0;

    // Gains use encoder ticks/second; f is motor power per tick/second.
    public static volatile double p = 0.01577;
    public static volatile double i = 0.0;
    public static volatile double d = 0.00070;
    public static volatile double f = 0.000496;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final PIDCoefficients gains = new PIDCoefficients(p, i, d);
    private final PIDController controller = new PIDController(gains);
    private final SimpleFFCoefficients feedforwardGains = new SimpleFFCoefficients(0.0, f, 0.0);
    private final SimpleFeedforward feedforward = new SimpleFeedforward(feedforwardGains);
    private DcMotorEx launcher;
    private boolean wasUsingPid;

    public ShooterTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("Ready - press START");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(0);
    }

    @Override
    public void periodic() {
        if (launcher == null) return;
        gains.kP = p;
        gains.kI = i;
        gains.kD = d;
        feedforwardGains.kV = f;

        boolean pid = usePid;
        if (pid != wasUsingPid) {
            controller.reset();
            wasUsingPid = pid;
        }

        double velocity = launcher.getVelocity();
        double output = pid
                ? controller.calculateFromReference(targetVelocity, velocity)
                        + feedforward.calculate(targetVelocity)
                : power;
        output = Double.isFinite(output) ? Math.max(-1, Math.min(1, output)) : 0;
        launcher.setPower(output);

        panelsTelemetry.addData("Mode", pid ? "PID" : "Power");
        panelsTelemetry.addData("Target Velocity (ticks/s)", pid ? targetVelocity : 0.0);
        panelsTelemetry.addData("Velocity (ticks/s)", velocity);
        panelsTelemetry.addData("Motor Power", output);
        panelsTelemetry.addLine(String.format(Locale.US,
                "PIDF | P: %.5f, I: %.5f, D: %.5f, F: %.6f", p, i, d, f));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        if (launcher != null) launcher.setPower(0);
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
