package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;

/** Webcam mounting and field layout for {@link PollenLocator}. Inches and degrees. */
@Configurable
public final class PollenVisionConfig {
    private PollenVisionConfig() { }

    public static String webcamName = "webcam";
    // Hive Vision's HSV size gates were tuned at 1280x720.
    public static int cameraWidth = 1280;
    public static int cameraHeight = 720;
    // Used only when the SDK has no calibration for this webcam and resolution
    // (Logitech C920 is about 70.4 degrees across at 16:9).
    public static double horizontalFovDegrees = 70.4;

    // Lens position relative to the robot's center, Pedro robot frame (+x forward, +y left).
    public static double cameraForward = 6;
    public static double cameraLeft = 0;
    public static double cameraHeightIn = 8;
    // Degrees below horizontal; 0 looks straight ahead.
    public static double cameraPitch = 20;
    // Degrees counter-clockwise from the robot's front.
    public static double cameraYaw = 0;

    public static double pollenDiameter = 2.8;
    public static double nectarDiameter = 3.6;
    // Projections farther than this from the lens are too noisy to drive to.
    public static double maxRange = 96;

    // Existing autos use a 141.5-inch Pedro field.
    public static double fieldSize = 141.5;
    public static double hiveCenterX = 70.75;
    public static double hiveCenterY = 70.75;
    // HIVE legs: A-frames 49.46" apart along x, each frame's feet 38.95" apart along y.
    public static double hiveLegSpacingX = 49.46;
    public static double hiveLegSpacingY = 38.95;
    public static double hiveLegThickness = 1.5;

    // Weaver: half the physical intake width, and the robot footprint.
    public static double intakeHalfWidth = 4;
    public static double robotWidth = 18;
    public static double robotLength = 18;
    public static double clearance = 2;
    public static int maxPollenPerPath = 4;
}
