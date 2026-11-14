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
    public static double staticPowerRampDegrees = 2;
    public static double maxPower = 0.75;
    public static double manualPower = 0.4;
    public static double toleranceDegrees = 2;
    public static double halfwayY = 70.75;
    public static int pipeline = 0;

    public static double txToCcwSign = -1;
    public static double acquireDegrees = 10;
    public static double visionUpdateDegrees = 2;
    public static double maxFrameAgeMs = 100;

}
