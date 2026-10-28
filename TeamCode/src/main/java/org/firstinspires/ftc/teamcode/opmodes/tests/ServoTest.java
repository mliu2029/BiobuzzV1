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

    private Servo servo;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        servo = hardwareMap.get(Servo.class, "transfer");
        servo.setDirection(Servo.Direction.FORWARD);
        loop();
    }

    @Override
    public void loop() {
        if (servo == null) return;
        double requestedPosition = position;
        if (Double.isFinite(requestedPosition)) {
            servo.setPosition(Math.max(0.0, Math.min(1.0, requestedPosition)));
        }

        panelsTelemetry.addData("Requested Position", requestedPosition);
        panelsTelemetry.addData("Servo Position", servo.getPosition());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
