package org.firstinspires.ftc.teamcode.data;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.data.HiveTarget;

@Configurable
public final class TurretConfig {
    private TurretConfig() { }


    // goBILDA 1620 RPM Yellow Jacket: 28 counts/motor rev, exact internal ratio 1 + 46/17.
    // Existing external gearing: 24T motor gear driving the 90T turret gear.
    public static double ticksPerDegree = 28.0 * (1.0 + 46.0 / 17.0) * (90.0 / 24.0) / 360.0;
    public static double minDegrees = -70;
    public static double maxDegrees = 70;
    public static double p = 0.01;
    public static double i = 0;
    public static double d = 0.001;
    public static double staticPower = 0;
    public static double maxPower = 0.75;
    public static double manualPower = 0.4;
    public static double toleranceDegrees = 2;
    public static int pipeline = 0;
    // Desired tx for each tag in ascending ID order, when aimed at the opening.
    // Zero offsets aim at the center of the selected tag.
    // Keep all four slots; aiming tracks one of the middle two IDs at a time.
    public static double[] redCloseTx = {0, 0, 0, 0}; // 34..37
    public static double[] blueCloseTx = {0, 0, 0, 0}; // 38..41
    public static double[] redFarTx = {0, 0, 0, 0}; // 30..33
    public static double[] blueFarTx = {0, 0, 0, 0}; // 42..45
    // Normally a tag to the camera's right requires clockwise (negative CCW) motion.
    public static double txToCcwSign = -1;
    // Camera-only aiming can acquire visible tags without an odometry handoff window.
    public static double acquireDegrees = 90;
    public static double maxFrameAgeMs = 100;

    // ---------------- Shooting on the move (virtual goal) ----------------
    // Modeled on Seattle Solvers' (FTC 23511) VirtualGoalSolver; see data/VirtualGoalSolver.java.
    // Uses Pedro odometry: follower.pose() (inches, radians) and follower.velocity() (in/s, rad/s).
    // The shooter is fixed-velocity, so only the aim is compensated; the effective distance is
    // exposed on Turret for a future distance -> flywheel velocity table.

    /** Whether shoot-on-the-move starts enabled. Gamepad 2 D-pad up toggles it at runtime. */
    public static boolean sotmDefaultOn = false;
    /**
     * false: keep aiming with the Limelight and add the odometry lead angle on top (needs only
     *        odometry velocity and rough distance, so it tolerates an uncalibrated pose).
     * true:  aim purely from odometry at the virtual goal, as Seattle Solvers does (needs an
     *        accurate pose, heading and HiveTarget coordinates; ignores the Limelight).
     */
    public static boolean sotmOdometryAim = false;
    /** Turret pivot relative to the odometry reference point, robot frame: forward (+) / left (+). */
    public static double sotmTurretOffsetXInches = 0;
    public static double sotmTurretOffsetYInches = 0;
    /** Delay from deciding to shoot until the ball leaves the robot. PLACEHOLDER: measure it. */
    public static double sotmBallTransferTimeSec = 0.15;
    /**
     * Ball flight time versus turret-to-goal distance (inches), linearly interpolated. Distances
     * must be strictly ascending. PLACEHOLDER VALUES: measure on this robot with the real ball.
     */
    public static double[] sotmTofDistancesInches = {0, 150};
    public static double[] sotmTofSeconds = {0.5, 0.5};
    /** Virtual-goal iterations (Seattle Solvers uses 5). */
    public static int sotmIterations = 5;
    /** Cap on the lead angle so a close, fast shot cannot swing the turret wildly. */
    public static double sotmMaxLeadDegrees = 45;
    /**
     * Seconds of turret aim-rate lead added to the target (Seattle Solvers: TURRET_VEL_LAG =
     * 0.067). 0 disables it; tune upward only after the basic lead is working.
     */
    public static double sotmVelocityLeadSec = 0;

    public static double timeOfFlightSeconds(double distanceInches) {
        return org.firstinspires.ftc.teamcode.data.VirtualGoalSolver.interpolate(
                sotmTofDistancesInches, sotmTofSeconds, distanceInches);
    }

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
