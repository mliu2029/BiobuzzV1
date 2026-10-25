package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.data.HiveTarget;

public final class Calculations {
    private Calculations() { }

// calculation for turret
    public static double bearing(double x, double y, double headingRadians, HiveTarget target) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingRadians)) {
            return Double.NaN;
        }
        if (Math.hypot(target.x - x, target.y - y) < 1e-6) return Double.NaN;
        return normalize(Math.toDegrees(Math.atan2(target.y - y, target.x - x)
                - headingRadians));
    }

    public static double normalize(double degrees) {
        return ((degrees + 180) % 360 + 360) % 360 - 180;
    }

    public static double reachableAngle(double angle, double current, double min, double max) {
        if (!Double.isFinite(angle) || !Double.isFinite(current)
                || !Double.isFinite(min) || !Double.isFinite(max) || min >= max) {
            return Double.NaN;
        }
        double low = Math.ceil((min - angle) / 360);
        double high = Math.floor((max - angle) / 360);
        if (low > high) return Double.NaN;
        double turns = Math.max(low, Math.min(high, Math.rint((current - angle) / 360)));
        return angle + turns * 360;
    }

    public static boolean usableVision(HiveTarget target, int observedId, int pipeline,
                                       int expectedPipeline, double ageMs, double maxAgeMs,
                                       double txError, double acquireDegrees) {
        return target != null && target.acceptsTag(observedId) && pipeline == expectedPipeline
                && Double.isFinite(ageMs) && ageMs >= 0 && ageMs <= maxAgeMs
                && Double.isFinite(txError) && Math.abs(txError) <= acquireDegrees;
    }
}
