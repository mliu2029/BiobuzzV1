package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

/** Captures one stationary correction each time a usable tag is acquired after being lost. */
@Configurable
public final class TurretAngleEstimator {
    // OFF clears/bypasses the correction. Turn ON to arm one new stationary measurement.
    public static volatile boolean applyVisionCorrections = true;
    private static final double STILL_DWELL_MS = 200;
    private double offsetDegrees;
    private double visionChassisDegrees = Double.NaN;
    private boolean corrected;
    private boolean correctedWhileVisible;
    private long stillSinceNanos;
    private String status = "Waiting for robot and turret to stop";

    public void reset() {
        offsetDegrees = 0;
        visionChassisDegrees = Double.NaN;
        corrected = false;
        correctedWhileVisible = false;
        resetStillness();
        status = "Waiting for robot and turret to stop";
    }

    public void resetStillness() { stillSinceNanos = 0; }

    /** Re-arm the next sighting while keeping the previous angle correction for odometry. */
    public void tagLost() {
        correctedWhileVisible = false;
        resetStillness();
        status = corrected ? "Tag lost; holding correction, re-armed" : "Waiting for a usable 3D tag";
    }

    public void updateStillness(long nowNanos, double speedInchesPerSecond,
                               double turnDegreesPerSecond, double turretDegreesPerSecond) {
        if (!applyVisionCorrections) {
            reset();
            return;
        }
        if (correctedWhileVisible) return;
        if (!(Math.abs(speedInchesPerSecond) <= 1)
                || !(Math.abs(turnDegreesPerSecond) <= 3)
                || !(Math.abs(turretDegreesPerSecond) <= 3)) {
            resetStillness();
            status = "Waiting for robot and turret to stop";
            return;
        }
        if (stillSinceNanos == 0) stillSinceNanos = nowNanos;
        status = "Waiting for stillness and one usable 3D tag";
    }

    /** goalDegrees and cameraErrorCcwDegrees are both relative to the chassis, CCW positive. */
    public boolean observe(long nowNanos, double frameAgeMs, double goalDegrees,
                           double encoderDegrees, double cameraErrorCcwDegrees) {
        if (!applyVisionCorrections || correctedWhileVisible) return false;
        if (!Double.isFinite(frameAgeMs) || frameAgeMs < 0
                || !Double.isFinite(goalDegrees) || !Double.isFinite(encoderDegrees)
                || !Double.isFinite(cameraErrorCcwDegrees)) return false;
        // The image must have been captured after motion stopped; no pose history is needed.
        if (stillSinceNanos == 0
                || (nowNanos - stillSinceNanos) / 1e6 < Math.max(STILL_DWELL_MS, frameAgeMs)) {
            status = "Waiting for robot and turret to stay still";
            return false;
        }
        visionChassisDegrees = Calculations.normalize(goalDegrees - cameraErrorCcwDegrees);
        offsetDegrees = Calculations.normalize(visionChassisDegrees - encoderDegrees);
        corrected = true;
        correctedWhileVisible = true;
        status = "CORRECTED ONCE; holding until tag is lost";
        return true;
    }

    public double chassisDegrees(double encoderDegrees) {
        return Calculations.normalize(encoderDegrees + getOffsetDegrees());
    }

    public double getOffsetDegrees() { return applyVisionCorrections ? offsetDegrees : 0; }
    public double getVisionChassisDegrees() { return visionChassisDegrees; }
    public boolean hasCorrection() { return applyVisionCorrections && corrected; }
    public String getStatus() {
        return applyVisionCorrections ? status : "Correction OFF; turn ON to arm one measurement";
    }
}
