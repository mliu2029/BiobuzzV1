package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class TurretConfig {
    private TurretConfig() { }


    public static final double ENCODER_COUNTS_PER_REVOLUTION = 8192.0;
    // Encoder shaft revolutions / turret revolution: 30T encoder / existing 92T turret.
    // Update this if either gear changes. Do not include the turret motor's gearbox.
    public static double encoderRevolutionsPerTurretRevolution = 92.0 / 30.0;
    // Set to -1 if raw counts decrease during positive (CCW) turret rotation.
    // This changes feedback polarity only, not motor power direction.
    public static int encoderDirection = 1;
    public static double minDegrees = -150;
    public static double maxDegrees = 80;
    public static double p = 0.01;
    public static double i = 0;
    public static double d = 0.000125;
    public static double staticPower = 0.04;
    public static double staticPowerRampDegrees = 2;
    public static double manualPower = 0.4;
    public static double toleranceDegrees = 2;
    public static double halfwayY = 70.75;
    public static int pipeline = 0;
    // Camera-to-launcher alignment trim, after correcting from the tag to the cell center.
    public static double visionOffsetDegrees = 0;
    public static double txToCcwSign = -1;
    public static double acquireDegrees = 10;
    public static double maxFrameAgeMs = 100;

    public static boolean isEncoderConfigured() {
        return Double.isFinite(encoderRevolutionsPerTurretRevolution)
                && encoderRevolutionsPerTurretRevolution > 0
                && (encoderDirection == 1 || encoderDirection == -1)
                && Double.isFinite(getDegreesPerEncoderTick())
                && getDegreesPerEncoderTick() != 0;
    }

    public static double getDegreesPerEncoderTick() {
        return encoderDirection * 360.0
                / (ENCODER_COUNTS_PER_REVOLUTION * encoderRevolutionsPerTurretRevolution);
    }
}
