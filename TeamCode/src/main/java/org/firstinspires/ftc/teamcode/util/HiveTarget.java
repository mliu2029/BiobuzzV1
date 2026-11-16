package org.firstinspires.ftc.teamcode.util;

public enum HiveTarget {
    RED_CLOSE(58.01, 54.30, 34),
    BLUE_CLOSE(83.51, 54.30, 38),
    RED_FAR(58.01, 87.20, 30),
    BLUE_FAR(83.51, 87.20, 42);

    public final double x;
    public final double y;
    public final int firstTagId;

    HiveTarget(double x, double y, int firstTagId) {
        this.x = x;
        this.y = y;
        this.firstTagId = firstTagId;
    }

    public boolean acceptsTag(int id) {
        return id >= firstTagId && id < firstTagId + 4;
    }
}
