package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public final class HiveAimGeometry {
    private HiveAimGeometry() { }

    private static final double[] TAG_X_INCHES = {-6.5, -2.75, 2.75, 6.5};
    private static final double TOP_METERS = 7.1875 * 0.0254;
    // Area centroid of the main opening cutout in the front rim plane, measured from hive CAD.
    private static final double DEPTH_METERS = 5.76136 * 0.0254;

    public static final class AimPoint {
        public final double rightMeters;
        public final double downMeters;
        public final double forwardMeters;
        public final boolean upright;
        private AimPoint(double right, double down, double forward, boolean upright) {
            rightMeters = right;
            downMeters = down;
            forwardMeters = forward;
            this.upright = upright;
        }

        /** Horizontal angle from the camera's forward axis to the opening center. */
        public double errorDegrees() {
            return Math.toDegrees(Math.atan2(rightMeters, forwardMeters));
        }
    }

    public static AimPoint fromTag(HiveTarget cell, int tagId, Pose3D tagPose) {
        if (cell == null || !cell.acceptsTag(tagId) || tagPose == null
                || tagPose.getPosition() == null || tagPose.getOrientation() == null
                || tagPose.getPosition().unit == null) return null;
        Position p = tagPose.getPosition().toUnit(DistanceUnit.METER);
        YawPitchRollAngles angles = tagPose.getOrientation();
        double roll = angles.getRoll(AngleUnit.RADIANS);
        double pitch = angles.getPitch(AngleUnit.RADIANS);
        double yaw = angles.getYaw(AngleUnit.RADIANS);
        if (!Double.isFinite(p.x) || !Double.isFinite(p.y) || !Double.isFinite(p.z)
                || !Double.isFinite(roll) || !Double.isFinite(pitch) || !Double.isFinite(yaw)) return null;
        if (p.z <= 1e-6) return null;

        double sr = Math.sin(roll), cr = Math.cos(roll);
        double sp = Math.sin(pitch), cp = Math.cos(pitch);
        double sy = Math.sin(yaw), cy = Math.cos(yaw);
        double r00 = cy * cp, r01 = cy * sp * sr - sy * cr, r02 = cy * sp * cr + sy * sr;
        double r10 = sy * cp, r11 = sy * sp * sr + cy * cr, r12 = sy * sp * cr - cy * sr;
        double r20 = -sp, r21 = cp * sr, r22 = cp * cr;
        double dx = -TAG_X_INCHES[tagId - cell.firstTagId] * 0.0254;
        double dy = -TOP_METERS;
        double dz = DEPTH_METERS;
        double x = p.x + r00 * dx + r01 * dy + r02 * dz;
        double y = p.y + r10 * dx + r11 * dy + r12 * dz;
        double z = p.z + r20 * dx + r21 * dy + r22 * dz;
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z) || z <= 1e-6) {
            return null;
        }

        double topY = p.y - r11 * 0.01;
        double topZ = p.z - r21 * 0.01;
        boolean upright = topZ > 1e-6 && topY / topZ < p.y / p.z - 1e-7;
        return new AimPoint(x, y, z, upright);
    }
}
