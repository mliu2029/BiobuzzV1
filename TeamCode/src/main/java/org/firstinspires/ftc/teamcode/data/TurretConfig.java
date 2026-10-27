package org.firstinspires.ftc.teamcode.data;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.data.HiveTarget;

@Configurable
public final class TurretConfig {
    private TurretConfig() { }


    // goBILDA bare motor: 28 counts/rev
    public static double ticksPerDegree = 28.0 * (90.0 / 24.0) / 360.0;
    public static double minDegrees = 0;
    public static double maxDegrees = 0;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double staticPower = 0;
    public static double maxPower = 0.25;
    public static double toleranceDegrees = 2;
    public static double halfwayY = 70.75;

    public static boolean useVision = false;
    public static int pipeline = 0;
    // Desired tx for each tag in ascending ID order, when aimed at the opening.
    // Distinct tag offsets prevent a partial cluster from changing the aim point.
    public static double[] redCloseTx = {0, 0, 0, 0}; // 34..37
    public static double[] blueCloseTx = {0, 0, 0, 0}; // 38..41
    public static double[] redFarTx = {0, 0, 0, 0}; // 30..33
    public static double[] blueFarTx = {0, 0, 0, 0}; // 42..45
    // Normally a tag to the camera's right requires clockwise (negative CCW) motion.
    public static double txToCcwSign = -1;
    public static double acquireDegrees = 10;
    public static double maxFrameAgeMs = 100;

    public static double desiredTx(HiveTarget target, int tagId) {
        if (!target.acceptsTag(tagId)) return Double.NaN;
        double[] offsets;
        switch (target) {
            case RED_CLOSE: offsets = redCloseTx; break;
            case BLUE_CLOSE: offsets = blueCloseTx; break;
            case RED_FAR: offsets = redFarTx; break;
            default: offsets = blueFarTx; break;
        }
        int index = tagId - target.firstTagId;
        return offsets == null || index >= offsets.length ? Double.NaN : offsets[index];
    }
}
