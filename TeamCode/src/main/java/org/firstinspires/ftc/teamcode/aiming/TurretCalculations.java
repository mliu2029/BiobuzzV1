package org.firstinspires.ftc.teamcode.aiming;

public final class TurretCalculations {
    private TurretCalculations() { }

    public enum HiveTarget {
        RED_CLOSE(58, 59, 34), BLUE_CLOSE(84, 59, 38),
        RED_FAR(58, 85, 30), BLUE_FAR(84, 85, 42);

        public final double x;
        public final double y;
        public final int firstTagId;

        HiveTarget(double x, double y, int firstTagId) {
            this.x = x;
            this.y = y;
            this.firstTagId = firstTagId;
        }

        public boolean acceptsTag(int id) {
            return id >= firstTagId && id < firstTagId + 4;
        }
    }

    public enum Alliance {
        RED(HiveTarget.RED_CLOSE, HiveTarget.RED_FAR),
        BLUE(HiveTarget.BLUE_CLOSE, HiveTarget.BLUE_FAR);

        public final HiveTarget close;
        public final HiveTarget far;

        Alliance(HiveTarget close, HiveTarget far) {
            this.close = close;
            this.far = far;
        }

        public HiveTarget targetForY(double y, double halfwayY) {
            if (!Double.isFinite(y) || !Double.isFinite(halfwayY)) return null;
            return y < halfwayY ? close : far;
        }

        public boolean correctTarget(HiveTarget target) {
            return target == close || target == far;
        }
    }

    /** Counterclockwise degrees from robot forward; includes the turret offsets. */
    public static double bearing(double x, double y, double headingRadians,
                                 double turretOffsetForward, double turretOffsetLeft, HiveTarget target) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingRadians)
                || !Double.isFinite(turretOffsetForward) || !Double.isFinite(turretOffsetLeft)) {
            return Double.NaN;
        }
        double turretX = x + turretOffsetForward * Math.cos(headingRadians)
                - turretOffsetLeft * Math.sin(headingRadians);
        double turretY = y + turretOffsetForward * Math.sin(headingRadians)
                + turretOffsetLeft * Math.cos(headingRadians);
        if (Math.hypot(target.x - turretX, target.y - turretY) < 1e-6) return Double.NaN;
        return normalize(Math.toDegrees(Math.atan2(target.y - turretY, target.x - turretX)
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
