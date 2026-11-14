package org.firstinspires.ftc.teamcode.util;

import java.util.function.DoubleUnaryOperator;

/**
 * Shooting-on-the-move math using a "virtual goal", modeled on the Seattle Solvers (FTC 23511)
 * DECODE solver (MathFunctions.VirtualGoalSolver in github.com/FTC-23511/Decode-2026).
 *
 * <p>A ball leaves the shooter carrying the shooter's own velocity, so a shot aimed straight at
 * the goal lands off target while driving. Instead we aim at a virtual goal: the real goal shifted
 * backward by (shooter velocity x time until the ball arrives). The arrival time is the
 * mechanical delay plus the ball's flight time, and the flight time depends on the distance to
 * the virtual goal, so the shift is iterated until it settles.
 *
 * <p>Pure math with no hardware dependencies. Units: inches, seconds, radians. Field frame:
 * x/y in the same frame as the robot pose, heading counterclockwise from +x.
 */
public final class VirtualGoalSolver {
    private VirtualGoalSolver() { }

    public static final class Result {
        /** Field-frame angle from the turret to the virtual goal. */
        public final double headingRadians;
        /** Distance from the turret to the virtual goal (the "effective distance" of the shot). */
        public final double distanceInches;
        /** Virtual-goal heading minus real-goal heading, in [-pi, pi); after any clamping. */
        public final double leadRadians;
        /**
         * How fast the turret's robot-relative aim angle is changing (rad/s): the aim bearing's
         * rate in the field minus the robot's rotation. Use for velocity feedforward.
         */
        public final double relativeAngularVelocity;
        /** True when the flight time was unavailable, so no lead was applied (aim at real goal). */
        public final boolean fallback;
        /** True when the lead was limited by maxLeadRadians. */
        public final boolean leadClamped;

        Result(double heading, double distance, double lead, double relativeOmega,
               boolean fallback, boolean leadClamped) {
            this.headingRadians = heading;
            this.distanceInches = distance;
            this.leadRadians = lead;
            this.relativeAngularVelocity = relativeOmega;
            this.fallback = fallback;
            this.leadClamped = leadClamped;
        }
    }

    /**
     * @param robotX,robotY,robotHeading robot pose (inches, radians) - the odometry reference point
     * @param vx,vy robot field-frame velocity (in/s) at the odometry reference point
     * @param omega robot angular velocity (rad/s, counterclockwise positive)
     * @param goalX,goalY real goal position in the same field frame
     * @param turretOffsetX,turretOffsetY turret pivot relative to the robot reference point, in the
     *        robot frame (x forward, y left), inches
     * @param transferTimeSec delay from the shot decision until the ball leaves the robot
     * @param timeOfFlight maps distance to the virtual goal (inches) to ball flight time (s);
     *        return NaN when no shot is possible
     * @param iterations fixed-point iterations of the virtual goal (Seattle Solvers uses 5)
     * @param maxLeadRadians cap on the lead angle (guards against runaway when close and fast);
     *        pass Infinity for no cap
     * @return the solution, or null when an input is not finite or the turret is on the goal
     */
    public static Result solve(double robotX, double robotY, double robotHeading,
                               double vx, double vy, double omega,
                               double goalX, double goalY,
                               double turretOffsetX, double turretOffsetY,
                               double transferTimeSec, DoubleUnaryOperator timeOfFlight,
                               int iterations, double maxLeadRadians) {
        if (!allFinite(robotX, robotY, robotHeading, vx, vy, omega, goalX, goalY,
                turretOffsetX, turretOffsetY, transferTimeSec) || timeOfFlight == null
                || Double.isNaN(maxLeadRadians) || maxLeadRadians < 0) {
            return null;
        }
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);
        // Turret pivot relative to the robot reference point, in the field frame.
        double offX = turretOffsetX * cos - turretOffsetY * sin;
        double offY = turretOffsetX * sin + turretOffsetY * cos;
        double turretX = robotX + offX;
        double turretY = robotY + offY;
        // Turret velocity = robot velocity + (omega x offset): rotation moves an off-center turret.
        double baseVx = vx - omega * offY;
        double baseVy = vy + omega * offX;

        double realDx = goalX - turretX;
        double realDy = goalY - turretY;
        double realDistance = Math.hypot(realDx, realDy);
        if (realDistance < 1e-6) return null;
        double realHeading = Math.atan2(realDy, realDx);

        double virtualX = goalX;
        double virtualY = goalY;
        double distance = realDistance;
        boolean fallback = false;
        for (int i = 0; i < Math.max(1, iterations); i++) {
            double flight = timeOfFlight.applyAsDouble(distance);
            if (!Double.isFinite(flight) || flight < 0) {
                fallback = true;
                break;
            }
            double drift = transferTimeSec + flight;
            // Shift the target opposite to our velocity.
            virtualX = goalX - baseVx * drift;
            virtualY = goalY - baseVy * drift;
            distance = Math.hypot(virtualX - turretX, virtualY - turretY);
        }
        if (fallback) {
            // Safe state: face the real goal and cancel robot rotation so the turret stays
            // world-locked.
            return new Result(realHeading, realDistance, 0, -omega, true, false);
        }

        double dx = virtualX - turretX;
        double dy = virtualY - turretY;
        double heading = Math.atan2(dy, dx);
        double lead = normalizeSigned(heading - realHeading);
        boolean clamped = false;
        if (Math.abs(lead) > maxLeadRadians) {
            lead = Math.copySign(maxLeadRadians, lead);
            heading = realHeading + lead;
            clamped = true;
        }

        // Rate of change of the bearing to the (fixed) virtual goal as the turret moves with
        // velocity v:  dphi/dt = (vx*dy - vy*dx) / |d|^2.
        // (The Seattle Solvers source uses the opposite sign, (vy*dx - vx*dy); checked here
        // against a numeric derivative.)
        double distSq = dx * dx + dy * dy;
        double fieldOmega = distSq > 0.1 ? (baseVx * dy - baseVy * dx) / distSq : 0.0;
        return new Result(heading, Math.sqrt(distSq), lead, fieldOmega - omega, false, clamped);
    }

    /** Linear interpolation of ys over ascending xs, holding the end values outside the range. */
    public static double interpolate(double[] xs, double[] ys, double x) {
        if (xs == null || ys == null || xs.length == 0 || xs.length != ys.length
                || !Double.isFinite(x)) {
            return Double.NaN;
        }
        for (int i = 0; i < xs.length; i++) {
            if (!Double.isFinite(xs[i]) || !Double.isFinite(ys[i])
                    || (i > 0 && xs[i] <= xs[i - 1])) {
                return Double.NaN;
            }
        }
        if (x <= xs[0]) return ys[0];
        for (int i = 1; i < xs.length; i++) {
            if (x <= xs[i]) {
                double ratio = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + ratio * (ys[i] - ys[i - 1]);
            }
        }
        return ys[ys.length - 1];
    }

    private static double normalizeSigned(double radians) {
        double twoPi = 2 * Math.PI;
        double angle = ((radians + Math.PI) % twoPi + twoPi) % twoPi - Math.PI;
        return angle;
    }

    private static boolean allFinite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
