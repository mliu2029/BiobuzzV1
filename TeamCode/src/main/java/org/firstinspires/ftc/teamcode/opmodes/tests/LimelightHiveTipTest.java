package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.data.Calculations;
import org.firstinspires.ftc.teamcode.data.TurretConfig;

@Configurable
@TeleOp(name = "Limelight Hive Tip Test", group = "Tuning")
public class LimelightHiveTipTest extends OpMode {
    private static final String LOG_TAG = "HiveTipTest";
    private static final int REQUIRED_ROTATION_FRAMES = 3;
    private static final long TAG_LOSS_NANOS = 200_000_000;

    public static volatile int tagId = 35;
    public static volatile double tipThresholdDegrees = 10;

    private enum State { IDLE, BASELINING, ARMED, TIPPING, LIKELY_TIPPED, UNKNOWN }

    private Limelight3A camera;
    private int watchedTag;
    private int pipeline;
    private double thresholdDegrees;
    private double lastFrameTimestamp;
    private State state = State.IDLE;
    private Quaternion baseline;
    private long lastUsableNanos;
    private double baselineTimestamp;
    private double rotationDegrees;
    private double peakDegrees;
    private int consecutiveFrames;
    private int rotationFrames;
    private double candidateDirection;
    private String cameraStatus = "Waiting for camera";

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Park at the shooting pose and keep chassis and turret still.");
        telemetry.addLine("Set tagId in Panels and use the Full 3D AprilTag pipeline.");
        telemetry.addLine("After START, press D1 Cross/A while the hive is upright.");
        telemetry.update();
    }

    @Override
    public void start() {
        watchedTag = tagId;
        pipeline = TurretConfig.pipeline;
        camera = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (camera == null) {
            cameraStatus = "No hardware named limelight";
            return;
        }
        camera.setPollRateHz(50);
        camera.start();
        if (!camera.pipelineSwitch(pipeline)) cameraStatus = "Pipeline switch failed";
    }

    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()) beginTrial();
        long now = System.nanoTime();
        if (camera != null) sampleFrame(camera.getLatestResult(), now);
        if ((state == State.ARMED || state == State.TIPPING)
                && now - lastUsableNanos >= TAG_LOSS_NANOS) {
            state = state == State.TIPPING ? State.LIKELY_TIPPED : State.UNKNOWN;
            RobotLog.ii(LOG_TAG, "tag=%d result=%s rotationFrames=%d peakDeg=%.2f",
                    watchedTag, state, rotationFrames, peakDegrees);
        }
        telemetry.addData("State", state);
        telemetry.addData("Tag / pipeline", "%d / %d", watchedTag, pipeline);
        telemetry.addData("Camera", cameraStatus);
        telemetry.addData("Rotation / peak (deg)", "%+.1f / %+.1f", rotationDegrees, peakDegrees);
        telemetry.addData("Visible rotation frames", rotationFrames);
        telemetry.update();
    }

    private void beginTrial() {
        thresholdDegrees = tipThresholdDegrees;
        if (!(thresholdDegrees > 0 && thresholdDegrees <= 90)) {
            state = State.IDLE;
            telemetry.addLine("Set tipThresholdDegrees to a value above 0 and at most 90.");
            return;
        }
        state = State.BASELINING;
        baseline = null;
        rotationDegrees = 0;
        peakDegrees = 0;
        consecutiveFrames = 0;
        rotationFrames = 0;
    }

    private void sampleFrame(LLResult result, long now) {
        if (result == null || result.getPipelineIndex() != pipeline) {
            cameraStatus = "Waiting for the selected pipeline";
            consecutiveFrames = 0;
            return;
        }
        double ageMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        if (!(ageMs >= 0 && ageMs <= TurretConfig.maxFrameAgeMs)) {
            cameraStatus = "Camera frame stale";
            consecutiveFrames = 0;
            return;
        }
        double timestamp = result.getTimestamp();
        if (!(Double.isFinite(timestamp) && timestamp > 0)) {
            cameraStatus = "No Limelight frame timestamp";
            return;
        }
        // Polling can return the same image more than once, with a new hub receipt time.
        if (timestamp <= lastFrameTimestamp) {
            if (now - lastUsableNanos >= TAG_LOSS_NANOS && baseline != null) {
                cameraStatus = "No new usable frame";
            }
            return;
        }
        lastFrameTimestamp = timestamp;
        Quaternion rotation = watchedTagRotation(result);
        if (rotation == null) {
            cameraStatus = "Waiting for watched tag with Full 3D pose";
            consecutiveFrames = 0;
            return;
        }
        cameraStatus = "Tracking watched tag";
        if (state == State.BASELINING) {
            baseline = rotation;
            baselineTimestamp = timestamp;
            lastUsableNanos = now;
            state = State.ARMED;
        } else if (state == State.ARMED || state == State.TIPPING) {
            lastUsableNanos = now;
            Quaternion relative = baseline.conjugate().multiply(rotation, now);
            // The hinge is parallel to the tag row (local X). Measure signed rotation around it.
            rotationDegrees = Calculations.normalize(
                    Math.toDegrees(2 * Math.atan2(relative.x, relative.w)));
            if (Math.abs(rotationDegrees) > Math.abs(peakDegrees)) peakDegrees = rotationDegrees;
            countRotationFrame();
        } else {
            return;
        }
        RobotLog.ii(LOG_TAG, String.format(Locale.US,
                "tag=%d frame=%.6f baseline=%.6f ageMs=%.1f rotationDeg=%.2f frames=%d state=%s",
                watchedTag, timestamp, baselineTimestamp, ageMs, rotationDegrees, rotationFrames, state));
    }

    private void countRotationFrame() {
        if (state == State.TIPPING) {
            rotationFrames++;
            return;
        }
        if (Math.abs(rotationDegrees) < thresholdDegrees) {
            consecutiveFrames = 0;
            return;
        }
        double direction = Math.signum(rotationDegrees);
        if (direction != candidateDirection) consecutiveFrames = 0;
        candidateDirection = direction;
        consecutiveFrames++;
        rotationFrames = Math.max(rotationFrames, consecutiveFrames);
        if (consecutiveFrames >= REQUIRED_ROTATION_FRAMES) state = State.TIPPING;
    }

    private Quaternion watchedTagRotation(LLResult result) {
        if (!result.isValid() || result.getFiducialResults() == null) return null;
        for (FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() != watchedTag) continue;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null || !(pose.getPosition().z > 0)
                    || !Double.isFinite(pose.getPosition().z)) return null;
            YawPitchRollAngles angles = pose.getOrientation();
            float roll = (float) angles.getRoll(AngleUnit.DEGREES);
            float pitch = (float) angles.getPitch(AngleUnit.DEGREES);
            float yaw = (float) angles.getYaw(AngleUnit.DEGREES);
            if (!Float.isFinite(roll) || !Float.isFinite(pitch) || !Float.isFinite(yaw)) return null;
            return Quaternion.fromMatrix(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XYZ, AngleUnit.DEGREES, roll, pitch, yaw), 0).normalized();
        }
        return null;
    }

    @Override
    public void stop() {
        if (camera != null) camera.stop();
    }
}
