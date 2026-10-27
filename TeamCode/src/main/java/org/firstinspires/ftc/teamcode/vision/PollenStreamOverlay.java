package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.weaver.PathCurve;
import com.skeletonarmy.marrow.weaver.PathResult;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicInteger;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.teamcode.vision.BallBlobPipeline.BallBlob;
import org.firstinspires.ftc.teamcode.vision.BallBlobPipeline.BallColor;

/**
 * Annotated webcam feed for Panels. Runs after the detector in the same VisionPortal,
 * circles every pollen and nectar it found, and draws the planned Weaver path with an
 * arrow at each pollen it will visit.
 *
 * Each frame is shrunk to {@link PollenVisionConfig#streamWidth} before the annotations
 * are drawn, so the JPEG Panels ships over Wi-Fi stays small enough for a high frame
 * rate. Frames are only rendered while Panels has asked for one recently, so an idle
 * dashboard costs the Control Hub nothing. Panels always gets the newest frame
 * immediately, the way the SDK's own camera stream does.
 *
 * Give it the robot pose each loop with {@link #update(Pose, PathResult)}; the path is
 * projected from field coordinates back through the camera using that pose.
 */
public class PollenStreamOverlay implements VisionProcessor, CameraStreamSource {
    private static final Scalar POLLEN = new Scalar(255, 230, 0);
    private static final Scalar RED_NECTAR = new Scalar(255, 40, 40);
    private static final Scalar BLUE_NECTAR = new Scalar(40, 120, 255);
    private static final Scalar PATH = new Scalar(0, 255, 120);
    private static final Scalar SHADOW = new Scalar(0, 0, 0);
    private static final int SAMPLES_PER_SEGMENT = 16;
    /** Keep rendering this long after the last Panels request. */
    private static final long ACTIVE_NANOS = 2_000_000_000L;

    /** Immutable snapshot handed from the OpMode thread to the vision thread. */
    private static final class Snapshot {
        final Pose pose;
        final PathResult plan;

        Snapshot(Pose pose, PathResult plan) {
            this.pose = pose;
            this.plan = plan;
        }
    }

    private final PollenLocator locator;
    private final BallBlobPipeline detector;
    private final AtomicInteger framesSent = new AtomicInteger();
    private volatile Snapshot snapshot = new Snapshot(null, null);
    private volatile Bitmap latest;
    private volatile long lastRequestNanos = Long.MIN_VALUE;
    private volatile Continuation<? extends Consumer<Bitmap>> waiting;
    private final Mat small = new Mat();

    PollenStreamOverlay(PollenLocator locator, BallBlobPipeline detector) {
        this.locator = locator;
        this.detector = detector;
    }

    /** Latest robot pose and the path being followed (null for no path). */
    public void update(Pose robotPose, PathResult plan) {
        snapshot = new Snapshot(robotPose, plan);
    }

    /** Frames handed to Panels so far; stays at 0 if the stream never connected. */
    public int getFramesSent() {
        return framesSent.get();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) { }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        if (System.nanoTime() - lastRequestNanos > ACTIVE_NANOS) return null;

        double scale = Math.min(1.0, PollenVisionConfig.streamWidth / (double) input.cols());
        if (scale < 1.0) {
            Imgproc.resize(input, small, new Size(), scale, scale, Imgproc.INTER_AREA);
        } else {
            input.copyTo(small);
        }
        try {
            draw(small, snapshot, scale);
        } catch (RuntimeException e) {
            // A bad overlay should never take the detector or the camera down.
        }
        Bitmap bitmap = Bitmap.createBitmap(small.cols(), small.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(small, bitmap);
        latest = bitmap;

        Continuation<? extends Consumer<Bitmap>> continuation = waiting;
        if (continuation != null) {
            waiting = null;
            deliver(continuation, bitmap);
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) { }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        lastRequestNanos = System.nanoTime();
        Bitmap bitmap = latest;
        if (bitmap != null) {
            deliver(continuation, bitmap);
        } else {
            waiting = continuation;          // first request: answer with the next frame
        }
    }

    private void deliver(Continuation<? extends Consumer<Bitmap>> continuation, final Bitmap bitmap) {
        framesSent.incrementAndGet();
        continuation.dispatch(new ContinuationResult<Consumer<Bitmap>>() {
            @Override
            public void handle(Consumer<Bitmap> consumer) {
                consumer.accept(bitmap);
            }
        });
    }

    /** Draws onto a frame that is {@code scale} times the camera resolution. */
    private void draw(Mat frame, Snapshot snap, double scale) {
        drawPath(frame, snap, scale);
        int thin = Math.max(1, (int) Math.round(2 * scale));
        int thick = thin + 2;
        double font = Math.max(0.4, 1.2 * scale);
        for (BallBlob blob : detector.getDetections()) {
            boolean pollen = blob.color == BallColor.YELLOW;
            Scalar color = pollen ? POLLEN : blob.color == BallColor.RED ? RED_NECTAR : BLUE_NECTAR;
            Point center = new Point(blob.cx * scale, blob.cy * scale);
            int radius = (int) Math.round((Math.max(blob.rect.width, blob.rect.height) / 2.0 + 4) * scale);
            Imgproc.circle(frame, center, radius + 1, SHADOW, thick);
            Imgproc.circle(frame, center, radius, color, thin);

            String label = pollen ? "pollen" : "nectar";
            if (snap.pose != null) {
                Pose field = locator.project(blob.cx, blob.cy,
                        (pollen ? PollenVisionConfig.pollenDiameter : PollenVisionConfig.nectarDiameter) / 2,
                        snap.pose);
                if (field != null) {
                    label += String.format(Locale.US, " (%.0f, %.0f)", field.x(), field.y());
                }
            }
            Point textAt = new Point(center.x - radius, center.y - radius - 6);
            Imgproc.putText(frame, label, textAt, Imgproc.FONT_HERSHEY_SIMPLEX, font, SHADOW, thick);
            Imgproc.putText(frame, label, textAt, Imgproc.FONT_HERSHEY_SIMPLEX, font, color, thin);
        }
    }

    /** The Weaver route projected onto the floor, one arrow per visited pollen. */
    private void drawPath(Mat frame, Snapshot snap, double scale) {
        if (snap.plan == null || snap.pose == null) return;
        int thin = Math.max(1, (int) Math.round(3 * scale));
        int thick = thin + 2;
        for (PathCurve segment : snap.plan.getSegments()) {
            List<com.skeletonarmy.marrow.zones.Point> samples = segment.sample(SAMPLES_PER_SEGMENT);
            Point previous = null;
            for (int i = 0; i < samples.size(); i++) {
                com.skeletonarmy.marrow.zones.Point sample = samples.get(i);
                Point pixel = toPixel(sample.getX(), sample.getY(), snap.pose, scale);
                boolean last = i == samples.size() - 1;
                if (previous != null && pixel != null) {
                    if (last) {
                        Imgproc.arrowedLine(frame, previous, pixel, SHADOW, thick, Imgproc.LINE_AA, 0, 0.5);
                        Imgproc.arrowedLine(frame, previous, pixel, PATH, thin, Imgproc.LINE_AA, 0, 0.5);
                    } else {
                        Imgproc.line(frame, previous, pixel, SHADOW, thick, Imgproc.LINE_AA, 0);
                        Imgproc.line(frame, previous, pixel, PATH, thin, Imgproc.LINE_AA, 0);
                    }
                }
                previous = pixel;
            }
        }
    }

    private Point toPixel(double x, double y, Pose robotPose, double scale) {
        double[] uv = locator.toPixel(x, y, 0, robotPose);
        return uv == null ? null : new Point(uv[0] * scale, uv[1] * scale);
    }
}
