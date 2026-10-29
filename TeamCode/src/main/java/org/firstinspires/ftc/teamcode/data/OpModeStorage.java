package org.firstinspires.ftc.teamcode.data;

import com.pedropathing.math.Pose;

public final class OpModeStorage {
    private OpModeStorage() { }

    public static Pose autonomousEndPose = new Pose(70.75, 70.75, 90);
    public static double autonomousEndTurretDegrees = 0.0;

    public static void saveAutonomousEndTurretDegrees(double degrees) {
        autonomousEndTurretDegrees = Double.isFinite(degrees) ? degrees : 0.0;
    }

    /** Use each auto handoff once so a later standalone TeleOp starts at zero. */
    public static double consumeAutonomousEndTurretDegrees() {
        double degrees = autonomousEndTurretDegrees;
        autonomousEndTurretDegrees = 0.0;
        return Double.isFinite(degrees) ? degrees : 0.0;
    }
}
