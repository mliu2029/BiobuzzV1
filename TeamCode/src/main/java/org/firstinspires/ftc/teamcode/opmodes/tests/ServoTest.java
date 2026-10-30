package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp;

@Configurable
@TeleOp(name = "ServoTest", group = "Tuning")
public class ServoTest extends OpMode {
    public static volatile double position = 0.5;
    public static volatile double ramp1Position = 0.5;
    public static volatile double ramp2Position = 0.5;
    // One-shot Panels controls; each applies both configured ramp positions.
    public static volatile boolean rampUp = false;
    public static volatile boolean rampDown = false;
    public static volatile boolean runIntake = false;
    public static volatile double intakePower = 1.0;

    private Servo servo;
    private Servo ramp1;
    private Servo ramp2;
    private DcMotorEx intake;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        rampUp = false;
        rampDown = false;
        runIntake = false;
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("After START: tune position, ramp1Position, and ramp2Position.");
        panelsTelemetry.addLine("Set rampUp or rampDown true to move both ramps using the Ramp presets.");
        panelsTelemetry.addLine("runIntake runs/stops the intake; intakePower sets its power (0..1).");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        servo = hardwareMap.get(Servo.class, "transfer");
        ramp1 = hardwareMap.get(Servo.class, "ramp1");
        ramp2 = hardwareMap.get(Servo.class, "ramp2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setPower(0);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runIntake = false;
        servo.setDirection(Servo.Direction.FORWARD);
        ramp1.setDirection(Servo.Direction.FORWARD);
        ramp2.setDirection(Servo.Direction.FORWARD);
        loop();
    }

    @Override
    public void loop() {
        if (servo == null || ramp1 == null || ramp2 == null) return;
        // Consume a preset once, then keep the individual sliders editable.
        // Down takes priority if both requests arrive in the same loop.
        if (rampDown || rampUp) {
            boolean up = !rampDown;
            rampUp = false;
            rampDown = false;
            ramp1Position = up ? Ramp.ramp1UpPosition : Ramp.ramp1DownPosition;
            ramp2Position = up ? Ramp.ramp2UpPosition : Ramp.ramp2DownPosition;
        }
        double requestedPosition = position;
        double ramp1Requested = ramp1Position;
        double ramp2Requested = ramp2Position;
        setPosition(servo, requestedPosition);
        setPosition(ramp1, ramp1Requested);
        setPosition(ramp2, ramp2Requested);
        double requestedIntakePower = intakePower;
        if (intake != null) {
            intake.setPower(runIntake && Double.isFinite(requestedIntakePower)
                    ? Math.max(0.0, Math.min(1.0, requestedIntakePower)) : 0.0);
        }

        panelsTelemetry.addData("Requested Position", requestedPosition);
        panelsTelemetry.addData("Servo Position", servo.getPosition());
        panelsTelemetry.addData("Ramp 1 Requested Position", ramp1Requested);
        panelsTelemetry.addData("Ramp 1 Position", ramp1.getPosition());
        panelsTelemetry.addData("Ramp 2 Requested Position", ramp2Requested);
        panelsTelemetry.addData("Ramp 2 Position", ramp2.getPosition());
        panelsTelemetry.addData("Intake Running", intake != null && intake.getPower() > 0);
        panelsTelemetry.addData("Intake Power", intake == null ? 0 : intake.getPower());
        panelsTelemetry.update(telemetry);
    }

    private static void setPosition(Servo servo, double requestedPosition) {
        if (Double.isFinite(requestedPosition)) {
            servo.setPosition(Math.max(0.0, Math.min(1.0, requestedPosition)));
        }
    }

    @Override
    public void stop() {
        runIntake = false;
        rampUp = false;
        rampDown = false;
        if (intake != null) intake.setPower(0);
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
