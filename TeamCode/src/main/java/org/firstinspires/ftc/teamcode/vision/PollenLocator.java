package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.PollenVisionConfig.*;

import android.util.Size;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.vision.BallBlobPipeline.BallBlob;
import org.firstinspires.ftc.teamcode.vision.BallBlobPipeline.BallColor;

/**
 * Finds pollen (yellow) and nectar (red/blue) with Hive Vision's webcam HSV pipeline
 * and projects each detection onto the floor in Pedro field coordinates.
 * {@link #planPath(Pose)} hands the pollen to Marrow's Weaver, which orders them and
 * routes around the HIVE legs and nectar.
 */
public class PollenLocator {
    private final Detector detector = new Detector();
    private final VisionPortal portal;

    public PollenLocator(HardwareMap hardwareMap) {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                // This webcam supports higher frame rates at 1280x720 with MJPEG.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(detector)
                .enableLiveView(true)
                .build();
    }

    /** Field positions of every pollen in view. Headings point from the robot to the pollen. */
    public List<Pose> getPollen(Pose robotPose) {
        return locate(robotPose, true);
    }

    /** Field positions of every red or blue nectar in view. */
    public List<Pose> getNectar(Pose robotPose) {
        return locate(robotPose, false);
    }

    /**
     * Weaver route from the robot through the visible pollen, avoiding the HIVE legs
     * and nectar. Null when no pollen is visible or none can be reached.
     */
    public PathResult planPath(Pose robotPose) {
        List<Pose> pollen = getPollen(robotPose);
        if (pollen.isEmpty()) return null;

        List<Zone> obstacles = hiveLegs();
        for (Pose nectar : getNectar(robotPose)) {
            obstacles.add(new CircleZone(new Point(nectar.x(), nectar.y()), nectarDiameter / 2));
        }
        Weaver.setConfig(new PathConfig()
                .intakeWidth(intakeHalfWidth)
                .robotWidth(robotWidth)
                .robotHeight(robotLength)
                .clearance(clearance));
        PathResult result = Weaver.builder()
                .start(PedroPathingConverter.toPoint(robotPose))
                .targets(pollen, p -> new Point(p.x(), p.y()))
                .maxTargets(maxPollenPerPath)
                .obstacles(obstacles)
                .build();
        return result.getVisitedTargets().isEmpty() ? null : result;
    }

    /** {@link #planPath(Pose)} converted to a Pedro path, or null. */
    public Path planPedroPath(Pose robotPose) {
        PathResult result = planPath(robotPose);
        return result == null ? null : PedroPathingConverter.toPath(result);
    }

    /** The four HIVE legs as Weaver obstacles. */
    public static List<Zone> hiveLegs() {
        List<Zone> legs = new ArrayList<>();
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                legs.add(new CircleZone(new Point(
                        hiveCenterX + sx * hiveLegSpacingX / 2,
                        hiveCenterY + sy * hiveLegSpacingY / 2), hiveLegThickness / 2));
            }
        }
        return legs;
    }

    /** Stops detecting without closing the camera (saves Control Hub CPU). */
    public void pause() {
        detector.suspendDetection();
    }

    public void resume() {
        detector.resumeDetection();
    }

    public void close() {
        portal.close();
    }

    private List<Pose> locate(Pose robotPose, boolean pollen) {
        List<Pose> out = new ArrayList<>();
        double ballHeight = (pollen ? pollenDiameter : nectarDiameter) / 2;
        for (BallBlob blob : detector.getDetections()) {
            if ((blob.color == BallColor.YELLOW) != pollen) continue;
            Pose field = project(blob.cx, blob.cy, ballHeight, robotPose);
            if (field != null) out.add(field);
        }
        return out;
    }

    /**
     * Where the ray through pixel (u, v) meets the plane {@code heightIn} above the floor,
     * in Pedro field coordinates. Null if it misses the floor, is out of range, or is
     * off the field.
     */
    public Pose project(double u, double v, double heightIn, Pose robotPose) {
        double[] k = detector.intrinsics();
        double xn = (u - k[2]) / k[0];              // right of the optical axis
        double yn = (v - k[3]) / k[1];              // below the optical axis
        double pitch = Math.toRadians(cameraPitch);
        double forward = Math.cos(pitch) - yn * Math.sin(pitch);
        double down = Math.sin(pitch) + yn * Math.cos(pitch);
        if (down < 1e-6) return null;               // at or above the horizon
        double t = (cameraHeightIn - heightIn) / down;
        if (t <= 0 || t * Math.sqrt(1 + xn * xn + yn * yn) > maxRange) return null;

        // Lens frame -> robot frame
        double f = t * forward, left = -t * xn;
        double yaw = Math.toRadians(cameraYaw);
        double rx = cameraForward + f * Math.cos(yaw) - left * Math.sin(yaw);
        double ry = cameraLeft + f * Math.sin(yaw) + left * Math.cos(yaw);

        // Robot frame -> field frame
        double h = robotPose.heading();
        double x = robotPose.x() + rx * Math.cos(h) - ry * Math.sin(h);
        double y = robotPose.y() + rx * Math.sin(h) + ry * Math.cos(h);
        if (x < 0 || y < 0 || x > fieldSize || y > fieldSize) return null;
        return new Pose(x, y, Math.atan2(y - robotPose.y(), x - robotPose.x()));
    }

    /** Hive Vision's detector, also keeping the frame size and the SDK's lens calibration. */
    private static class Detector extends BallDetectorPipeline {
        private volatile double[] intrinsics = {1, 1, 0, 0};

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            super.init(width, height, calibration);
            if (calibration != null && !calibration.isFake() && !calibration.isDegenerate()) {
                intrinsics = new double[] {calibration.focalLengthX, calibration.focalLengthY,
                        calibration.principalPointX, calibration.principalPointY};
            } else {
                double focal = (width / 2.0) / Math.tan(Math.toRadians(horizontalFovDegrees / 2));
                intrinsics = new double[] {focal, focal, width / 2.0, height / 2.0};
            }
        }

        /** fx, fy, cx, cy in pixels. */
        double[] intrinsics() {
            return intrinsics;
        }
    }
}
