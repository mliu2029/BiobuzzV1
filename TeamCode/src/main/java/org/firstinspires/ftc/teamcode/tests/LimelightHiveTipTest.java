package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.HiveTipDetector;
import org.firstinspires.ftc.teamcode.util.LoopTiming;
import org.firstinspires.ftc.teamcode.util.TurretConfig;

@Configurable
@TeleOp(name = "tip test", group = "Tuning")
public class LimelightHiveTipTest extends OpMode {
    public static volatile int tagId = 35;

    private Limelight3A camera;
    private HiveTipDetector detector;
    private int watchedTag;
    private int pipeline;
    private final LoopTiming loopTiming = new LoopTiming();
    private long lastTelemetryNanos;
    private String cameraStatus = "Waiting for camera";

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Keep chassis and turret still, with the watched tag visible.");
        telemetry.addLine("Set tagId in Panels. After START, press D1 Cross/A to begin.");
        telemetry.addLine("After seeing the tag, 200 ms without a fresh sighting counts as a tip.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTiming.beforeStart();
        lastTelemetryNanos = 0;
        watchedTag = tagId;
        pipeline = TurretConfig.pipeline;
        camera = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (camera == null) {
            cameraStatus = "No hardware named limelight";
            return;
        }
        camera.setPollRateHz(50);
        camera.start();
        cameraStatus = camera.pipelineSwitch(pipeline) ? "Camera running" : "Pipeline switch failed";
    }

    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()) detector = new HiveTipDetector(watchedTag, pipeline);
        long now = System.nanoTime();
        loopTiming.record(now);
        if (detector != null) detector.update(camera == null ? null : camera.getLatestResult(), now);
        if (lastTelemetryNanos != 0 && now - lastTelemetryNanos < 100_000_000L) return;
        lastTelemetryNanos = now;
        telemetry.addData("State", detector == null ? "IDLE" : detector.state());
        telemetry.addData("Camera / tag", cameraStatus + " / " + watchedTag);
        telemetry.addData("Loop ms (avg / p95 / max; last 256)", loopTiming.summary());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (camera != null) camera.stop();
    }
}
