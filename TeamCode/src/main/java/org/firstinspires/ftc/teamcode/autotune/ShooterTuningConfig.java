package org.firstinspires.ftc.teamcode.autotune;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Settings for the FtcAutoTune relay-feedback tuner, configured for the
 * shooter ("launcher") motor. Adapted from FtcAutoTune's TuningConfig template.
 *
 * <p>The tuner outputs kP/kI/kD/kF for a user-space loop whose output is motor
 * power in [-1, 1]. Those numbers are NOT the hub-internal scale that
 * {@code setPIDFCoefficients(RUN_USING_ENCODER, ...)} expects, so don't paste
 * them into {@link org.firstinspires.ftc.teamcode.mechanisms.Shooter}'s
 * current SDK PIDF call. They drop straight into NextFTC v2's
 * {@code NextMotor.velocityConstants.withP(kP).withI(kI).withD(kD).withV(kF)}
 * when velocities are given in ticks/sec (default {@code anglePerCount}).
 *
 * <p>Every field here is live-editable in Panels (http://192.168.43.1:8001,
 * Configurables tab). {@code ShooterVelocityAutoTune} reads them when you press
 * START, so change values during INIT, then start. Edits made while the tuner is
 * already running are picked up on the next INIT.
 */
@Configurable
public class ShooterTuningConfig {

    private ShooterTuningConfig() {}

    // ---- Hardware ------------------------------------------------------

    /** Hardware-config name of the shooter motor (see mechanisms/Shooter.java). */
    public static String MOTOR_NAME = "flywheel";

    /** Set true if positive power spins the shooter so the encoder reads negative. */
    public static boolean REVERSED = false;

    // ---- Relay test tuning ---------------------------------------------

    /**
     * Magnitude of the relay (bang-bang) output, 0 to 1. Pick the smallest
     * value that still produces a clean velocity oscillation. 0.3-0.7 is
     * typical for a flywheel.
     */
    public static double RELAY_AMPLITUDE = 0.5;

    /** Number of full oscillation cycles to average for the Ku/Tu calculation. */
    public static int CYCLES_TO_COLLECT = 6;

    /** Number of initial cycles to discard while the oscillation settles. */
    public static int CYCLES_TO_IGNORE = 2;

    /** Safety timeout for the relay test, in seconds, in case the system never oscillates. */
    public static double RELAY_TEST_TIMEOUT_S = 15.0;

    // ---- Velocity (PIDF) tuner -----------------------------------------

    /**
     * Encoder ticks per revolution of the shooter wheel. Only used to show RPM
     * on telemetry next to ticks/sec; it does not affect the tuned gains.
     * 28 = bare goBILDA/REV HD Hex motor shaft (6000 RPM). Change if the
     * shooter is geared.
     */
    public static double TICKS_PER_REV = 28.0;

    /**
     * Target velocity in ticks/sec. Matches the shooting speed used in
     * mechanisms/Shooter.java so the gains are tuned at the real operating point.
     */
    public static double VELOCITY_TARGET_TICKS_PER_SEC = 1400;

    /** Hysteresis band for the velocity relay test, in ticks/sec. */
    public static double VELOCITY_HYSTERESIS_TICKS_PER_SEC = 30;

    /**
     * Open-loop power levels for the feedforward (kF) characterization step.
     * The motor is held at each power and its steady-state velocity recorded.
     */
    public static double[] FEEDFORWARD_TEST_POWERS = {0.5, 0.75, 1.0};

    /** How long to hold each feedforward test power before sampling, in seconds. */
    public static double FEEDFORWARD_SETTLE_TIME_S = 1.5;

    /**
     * If false, every candidate has kI = 0 (PD rule family). Usual choice for
     * flywheels, where kF handles steady-state error and integral mainly risks windup.
     */
    public static boolean TUNE_INTEGRAL_TERM = false;

    // ---- Helpers -------------------------------------------------------

    /** Converts a ticks/sec value to RPM using {@link #TICKS_PER_REV}. */
    public static double toRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }
}
