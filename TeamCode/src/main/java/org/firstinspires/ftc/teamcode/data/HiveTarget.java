package org.firstinspires.ftc.teamcode.data;

public enum HiveTarget {
    // Upward-facing cell opening centers from the BIOBUZZ field CAD, in Pedro inches.
    RED_CLOSE(58.01, 54.30, 34, 35, 36),
    BLUE_CLOSE(83.51, 54.30, 38, 39, 40),
    RED_FAR(58.01, 87.20, 30, 31, 32),
    BLUE_FAR(83.51, 87.20, 42, 43, 44);

    public final double x;
    public final double y;
    public final int firstTagId;
    public final int firstAimTagId;
    public final int secondAimTagId;

    HiveTarget(double x, double y, int firstTagId, int firstAimTagId, int secondAimTagId) {
        this.x = x;
        this.y = y;
        this.firstTagId = firstTagId;
        this.firstAimTagId = firstAimTagId;
        this.secondAimTagId = secondAimTagId;
    }

    public boolean acceptsTag(int id) {
        return id >= firstTagId && id < firstTagId + 4;
    }
}
