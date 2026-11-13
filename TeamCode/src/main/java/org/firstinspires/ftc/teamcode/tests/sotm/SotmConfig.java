package org.firstinspires.ftc.teamcode.tests.sotm;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class SotmConfig {
    private SotmConfig() { }

    // Required physical measurements. Flight time is calculated, never guessed.
    // CAD nominal fixed-hood exit tangent; verify the actual ball departure angle.
    public static volatile double launchAngleDegrees = 70.0;
    public static volatile double goalMinusReleaseHeightInches = Double.NaN;
    // Effective release point in chassis coordinates: forward +X, left +Y.
    // Zero assumes release at the odometry reference / turret axis.
    public static volatile double shooterForwardInches = 0;
    public static volatile double shooterLeftInches = 0;
    // Zero models instantaneous release; include measured feed/control delay if known.
    public static volatile double releaseDelaySeconds = 0;

    public static volatile double minDistanceInches = 12;
    public static volatile double maxDistanceInches = 120;
    public static volatile double maxFlywheelVelocity = 2200;
    public static volatile double maxRobotSpeedInchesPerSecond = 60;
    public static volatile double maxRobotTurnDegreesPerSecond = 180;
    public static volatile double maxTurretSpeedDegreesPerSecond = 180;
    public static volatile double flywheelTolerance = 75;
    public static volatile double turretRateToleranceDegreesPerSecond = 30;
    public static volatile double readyDwellMs = 100;
    public static volatile double maxLoopMs = 100;
    public static volatile double hiveSwitchHysteresisInches = 4;

    // Standalone starting pose; enable the auto handoff only after running autonomous.
    public static volatile boolean useAutonomousEndPose = false;
    public static volatile double startX = 70.75;
    public static volatile double startY = 70.75;
    public static volatile double startHeadingDegrees = 90;
}
