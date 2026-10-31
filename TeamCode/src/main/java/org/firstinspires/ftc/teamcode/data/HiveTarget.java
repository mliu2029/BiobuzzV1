package org.firstinspires.ftc.teamcode.data;

public enum HiveTarget {
    RED_CLOSE(58, 59, 34, 35, 36), BLUE_CLOSE(84, 59, 38, 39, 40),
    RED_FAR(58, 85, 30, 31, 32), BLUE_FAR(84, 85, 42, 43, 44);

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
        return id == firstAimTagId || id == secondAimTagId;
    }
}
