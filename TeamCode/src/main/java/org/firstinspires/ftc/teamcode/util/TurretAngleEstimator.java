package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

/** Calibrates the encoder angle from one stationary camera observation per tag acquisition. */
@Configurable
public final class TurretAngleEstimator {
    // OFF freezes the learned encoder zero. Turn ON to arm a new stationary measurement.
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
        status = corrected ? "Tag lost; holding correction, re-armed" : "Waiting for a usable tag";
    }

    public void updateStillness(long nowNanos, double speedInchesPerSecond,
                               double turnDegreesPerSecond, double turretDegreesPerSecond) {
        if (!applyVisionCorrections) {
            correctedWhileVisible = false;
            resetStillness();
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
        status = "Waiting for stillness and one usable tag";
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
        double measuredOffset = Calculations.normalize(visionChassisDegrees - encoderDegrees);
        offsetDegrees += Calculations.normalize(measuredOffset - offsetDegrees);
        corrected = true;
        correctedWhileVisible = true;
        status = "Encoder zero calibrated; holding until tag is lost";
        return true;
    }

    /** Continuous calibrated angle; do not wrap across a mechanical travel limit. */
    public double chassisDegrees(double encoderDegrees) {
        return encoderDegrees + offsetDegrees;
    }

    /** Inverse conversion used for the motor PID's raw encoder setpoint. */
    public double encoderDegrees(double chassisDegrees) {
        return chassisDegrees - offsetDegrees;
    }

    public double getOffsetDegrees() { return offsetDegrees; }
    public double getVisionChassisDegrees() { return visionChassisDegrees; }
    public boolean hasCorrection() { return corrected; }
    public String getStatus() {
        if (applyVisionCorrections) return status;
        return corrected ? "Learning OFF; holding calibrated encoder zero" : "Learning OFF";
    }
}
