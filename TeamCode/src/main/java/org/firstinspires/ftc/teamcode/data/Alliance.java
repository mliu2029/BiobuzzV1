package org.firstinspires.ftc.teamcode.data;

public enum Alliance {
    RED(HiveTarget.RED_CLOSE, HiveTarget.RED_FAR),
    BLUE(HiveTarget.BLUE_CLOSE, HiveTarget.BLUE_FAR);

    public final HiveTarget close;
    public final HiveTarget far;

    Alliance(HiveTarget close, HiveTarget far) {
        this.close = close;
        this.far = far;
    }

    public HiveTarget targetForY(double y, double halfwayY) {
        if (!Double.isFinite(y) || !Double.isFinite(halfwayY)) return null;
        return y < halfwayY ? close : far;
    }

}
