package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class TurretConfig {
    private TurretConfig() { }


    public static double ticksPerDegree = 28.0 * (1.0 + 46.0 / 17.0) * (92.0 / 24.0) / 360.0;
    public static double minDegrees = -150;
    public static double maxDegrees = 80;
    public static double p = 0.01;
    public static double visionP = 0.009;
    public static double movingVelocityFeedforward = 0;
    public static double i = 0;
    public static double d = 0.000125;
    public static double staticPower = 0.04;
    public static double maxPower = 0.75;
    public static double manualPower = 0.4;
    public static double toleranceDegrees = 2;
    public static double visionSettleDegrees = 1;
    public static double visionResumeDegrees = 2;
    public static double halfwayY = 70.75;
    public static int pipeline = 0;

    public static double txToCcwSign = -1;
    public static double acquireDegrees = 90;
    public static double maxFrameAgeMs = 100;

    // ---------------- Shooting on the move (virtual goal) ----------------
    // Modeled on Seattle Solvers' (FTC 23511) VirtualGoalSolver; see util/VirtualGoalSolver.java.
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
        return org.firstinspires.ftc.teamcode.util.VirtualGoalSolver.interpolate(
                sotmTofDistancesInches, sotmTofSeconds, distanceInches);
    }

}
