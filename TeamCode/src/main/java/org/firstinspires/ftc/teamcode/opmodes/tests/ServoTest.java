package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoTest", group = "Tuning")
public class ServoTest extends OpMode {
    public static volatile double position = 0.5;
    public static volatile double ramp1Position = 0.5;
    public static volatile double ramp2Position = 0.5;

    private Servo servo;
    private Servo ramp1;
    private Servo ramp2;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("After START: tune position, ramp1Position, and ramp2Position.");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        servo = hardwareMap.get(Servo.class, "transfer");
        ramp1 = hardwareMap.get(Servo.class, "ramp1");
        ramp2 = hardwareMap.get(Servo.class, "ramp2");
        servo.setDirection(Servo.Direction.FORWARD);
        ramp1.setDirection(Servo.Direction.FORWARD);
        ramp2.setDirection(Servo.Direction.FORWARD);
        loop();
    }

    @Override
    public void loop() {
        if (servo == null || ramp1 == null || ramp2 == null) return;
        double requestedPosition = position;
        double ramp1Requested = ramp1Position;
        double ramp2Requested = ramp2Position;
        setPosition(servo, requestedPosition);
        setPosition(ramp1, ramp1Requested);
        setPosition(ramp2, ramp2Requested);

        panelsTelemetry.addData("Requested Position", requestedPosition);
        panelsTelemetry.addData("Servo Position", servo.getPosition());
        panelsTelemetry.addData("Ramp 1 Requested Position", ramp1Requested);
        panelsTelemetry.addData("Ramp 1 Position", ramp1.getPosition());
        panelsTelemetry.addData("Ramp 2 Requested Position", ramp2Requested);
        panelsTelemetry.addData("Ramp 2 Position", ramp2.getPosition());
        panelsTelemetry.update(telemetry);
    }

    private static void setPosition(Servo servo, double requestedPosition) {
        if (Double.isFinite(requestedPosition)) {
            servo.setPosition(Math.max(0.0, Math.min(1.0, requestedPosition)));
        }
    }

    @Override
    public void stop() {
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
