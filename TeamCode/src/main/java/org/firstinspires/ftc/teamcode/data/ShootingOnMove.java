package org.firstinspires.ftc.teamcode.data;

/** Fixed-elevation ballistic solver. Inches, seconds, radians; field-frame linear velocity. */
public final class ShootingOnMove {
    public static final double GRAVITY = 9.80665 / 0.0254;

    private ShootingOnMove() { }

    public static final class Model {
        public final double launchAngleDegrees, heightDifferenceInches;
        public final double forwardInches, leftInches, releaseDelaySeconds;

        public Model(double angle, double heightDifference  , double forward, double left,
                     double releaseDelay) {
            launchAngleDegrees = angle;
            heightDifferenceInches = heightDifference;
            forwardInches = forward;
            leftInches = left;
            releaseDelaySeconds = releaseDelay;
        }
    }

    public static final class Solution {
        public final boolean valid;
        public final String status;
        public final double turretDegrees, flywheelVelocity, flightSeconds;
        public final double distanceInches, horizontalLaunchSpeed;

        private Solution(String status, double angle, double speed, double time,
                         double distance, double launchSpeed) {
            valid = status == null;
            this.status = valid ? "Ballistic solution" : status;
            turretDegrees = angle;
            flywheelVelocity = speed;
            flightSeconds = time;
            distanceInches = distance;
            horizontalLaunchSpeed = launchSpeed;
        }

        public static Solution invalid(String reason) {
            return new Solution(reason, Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN);
        }
    }

    public static Solution solve(double x, double y, double heading,
                                 double vx, double vy, double omega,
                                 double goalX, double goalY, Model model) {
        if (model == null) return Solution.invalid("Missing shot model");
        double[] values = {x, y, heading, vx, vy, omega, goalX, goalY,
                model.launchAngleDegrees, model.heightDifferenceInches,
                model.forwardInches, model.leftInches, model.releaseDelaySeconds};
        for (double value : values) {
            if (!Double.isFinite(value)) return Solution.invalid("Set launch angle / height; check pose and velocity");
        }
        if (model.launchAngleDegrees <= 0 || model.launchAngleDegrees >= 90
                || model.releaseDelaySeconds < 0 || model.releaseDelaySeconds > 1) {
            return Solution.invalid("Invalid launch angle or release delay");
        }

        double releaseHeading = heading + omega * model.releaseDelaySeconds;
        double c = Math.cos(releaseHeading), s = Math.sin(releaseHeading);
        double rx = c * model.forwardInches - s * model.leftInches;
        double ry = s * model.forwardInches + c * model.leftInches;
        double dx = goalX - x - vx * model.releaseDelaySeconds - rx;
        double dy = goalY - y - vy * model.releaseDelaySeconds - ry;
        // A release point away from the chassis rotation center inherits omega cross r.
        double releaseVx = vx - omega * ry;
        double releaseVy = vy + omega * rx;
        double distance = Math.hypot(dx, dy);
        double tangent = Math.tan(Math.toRadians(model.launchAngleDegrees));
        double height = model.heightDifferenceInches;
        double stationaryTimeSquared = 2 * (distance * tangent - height) / GRAVITY;
        if (distance < 1e-6 || !(stationaryTimeSquared > 0)) {
            return Solution.invalid("Too close for this launch angle / height");
        }

        // tan(theta)*|d-v*t| = height + g*t*t/2. Above this lower bound,
        // the residual is strictly decreasing and impact is on the descending arc.
        double low = Math.max(0.001, Math.max(Math.sqrt(2 * Math.max(0, height) / GRAVITY),
                Math.hypot(releaseVx, releaseVy) * tangent / GRAVITY));
        double high = 3.0;
        if (low >= high || residual(low, dx, dy, releaseVx, releaseVy, tangent, height) <= 0
                || residual(high, dx, dy, releaseVx, releaseVy, tangent, height) >= 0) {
            return Solution.invalid("No descending shot within 3 seconds");
        }
        for (int iteration = 0; iteration < 48; iteration++) {
            double mid = (low + high) / 2;
            if (residual(mid, dx, dy, releaseVx, releaseVy, tangent, height) > 0) low = mid;
            else high = mid;
        }
        double time = (low + high) / 2;
        double qx = dx - releaseVx * time, qy = dy - releaseVy * time;
        double launchSpeed = Math.hypot(qx, qy) / time;
        double stationarySpeed = distance / Math.sqrt(stationaryTimeSquared);
        // The existing empirical curve anchors a stationary shot. Assume local proportionality
        // between motor speed and muzzle speed when applying the ballistic correction.
        double flywheel = Calculations.velocity(distance) * launchSpeed / stationarySpeed;
        double angle = Calculations.normalize(Math.toDegrees(Math.atan2(qy, qx) - releaseHeading));
        if (!Double.isFinite(flywheel) || !Double.isFinite(angle) || launchSpeed <= 0) {
            return Solution.invalid("Invalid ballistic result");
        }
        return new Solution(null, angle, flywheel, time, distance, launchSpeed);
    }

    private static double residual(double t, double dx, double dy, double vx, double vy,
                                   double tangent, double height) {
        return tangent * Math.hypot(dx - vx * t, dy - vy * t) - height - GRAVITY * t * t / 2;
    }
}
