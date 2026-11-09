package org.firstinspires.ftc.teamcode.data;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class TurretConfig {
    private TurretConfig() { }


    // goBILDA 1620 RPM Yellow Jacket: 28 counts/motor rev, exact internal ratio 1 + 46/17.
    // Installed external gearing: 24T motor gear driving the 92T turret gear (CAD confirmed).
    public static double ticksPerDegree = 28.0 * (1.0 + 46.0 / 17.0) * (92.0 / 24.0) / 360.0;
    public static double minDegrees = -150;
    public static double maxDegrees = 45;
    public static double p = 0.03;
    // Used only while a usable Limelight tag supplies the aiming setpoint.
    public static double visionP = 0.005;
    // SOTM tracking feedforward, motor power per degree/second; tune on the robot.
    public static double movingVelocityFeedforward = 0;
    public static double i = 0;
    public static double d = 0.001;
    public static double staticPower = 0.065;
    public static double maxPower = 0.75;
    public static double manualPower = 0.4;
    public static double toleranceDegrees = 2;
    // While tracking a tag: brake near the aim point; resume beyond the wider band.
    public static double visionSettleDegrees = 1;
    public static double visionResumeDegrees = 2;
    // Odometry test: choose the close hive below this field Y, far hive above it.
    public static double halfwayY = 70.75;
    public static int pipeline = 0;
    // Legacy raw-tag test offsets in ascending ID order; only the middle two are used.
    // Normal aiming uses full 3D poses and per-tag offsets to the common cell opening.
    // Zero raw offsets aim at the selected tag's center.
    public static double[] redCloseTx = {0, 0, 0, 0}; // 34..37
    public static double[] blueCloseTx = {0, 0, 0, 0}; // 38..41
    public static double[] redFarTx = {0, 0, 0, 0}; // 30..33
    public static double[] blueFarTx = {0, 0, 0, 0}; // 42..45
    // Normally a tag to the camera's right requires clockwise (negative CCW) motion.
    public static double txToCcwSign = -1;
    // Camera-only aiming can acquire visible tags without an odometry handoff window.
    public static double acquireDegrees = 90;
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
