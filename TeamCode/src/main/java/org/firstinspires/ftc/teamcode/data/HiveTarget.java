package org.firstinspires.ftc.teamcode.data;

public enum HiveTarget {
    RED_CLOSE(58, 59, 34), BLUE_CLOSE(84, 59, 38),
    RED_FAR(58, 85, 30), BLUE_FAR(84, 85, 42);

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
