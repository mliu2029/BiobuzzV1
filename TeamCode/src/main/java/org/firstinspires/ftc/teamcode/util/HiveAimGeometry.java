package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public final class HiveAimGeometry {
    private HiveAimGeometry() { }

    private static final double[] TAG_X_INCHES = {-6.5, -2.75, 2.75, 6.5};

    /** Tag offset from the opening center, viewed from the front, in inches. */
    public static double tagOffsetInches(HiveTarget cell, int tagId) {
        return cell != null && cell.acceptsTag(tagId) ? TAG_X_INCHES[tagId - cell.firstTagId] : 0;
    }

    /** Uses odometry range, or the existing 50-inch estimate when no usable pose is available. */
    public static double estimateDistanceInches(HiveTarget cell, Pose pose) {
        if (cell == null || pose == null) return 50;
        double distance = Math.hypot(cell.x - pose.x(), cell.y - pose.y());
        return Double.isFinite(distance) && distance >= 12 ? distance : 50;
    }
}
