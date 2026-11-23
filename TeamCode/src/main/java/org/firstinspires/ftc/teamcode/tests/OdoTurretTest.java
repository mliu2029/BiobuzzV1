package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;

@Configurable
@NextTeleop(name = "odo turret test (USE THIS FOR DP)", group = "Tuning")
public class OdoTurretTest extends MainTeleOp {
    public static volatile double shooterVelocity = 0;
    public static volatile boolean autoShooterVelocity = true;
    public static volatile boolean blueAlliance = false;
    public static volatile boolean useVision = true;

    public OdoTurretTest(Robot robot) {
        super(robot, blueAlliance ? Alliance.BLUE : Alliance.RED);
        shooterVelocity = 0;
    }

    @Override
    protected void updateDrive() {
        if (autoShooterVelocity) {
            Follower follower = robot.drivetrain.getFollower();
            HiveTarget hive = robot.turret.getTarget();
            if (follower != null && hive != null) {
                Pose pose = follower.pose();
                double dist = Math.hypot(hive.x - pose.x(), hive.y - pose.y());
                shooterVelocity = Calculations.velocity(dist);
            }
        }
        if (robot.shooter.isEnabled()) {
            robot.shooter.setVelocity(shooterVelocity);
        }
        robot.turret.setOdometryVisionEnabled(useVision);
    }

    @Override
    protected void addAdditionalDriverTelemetry() {
        Follower follower = robot.drivetrain.getFollower();
        Pose pose = follower != null ? follower.pose() : null;
        HiveTarget hive = robot.turret.getTarget();
        double dist = pose != null && hive != null
                ? Math.hypot(hive.x - pose.x(), hive.y - pose.y()) : Double.NaN;
        addDriverData("Hive dist", Double.isFinite(dist)
                ? String.format(Locale.US, "%.1f in", dist) : "--");
        addDriverData("Vision enabled", useVision);
        addDriverData("Angle source", "REV through-bore encoder; manual/startup zero");
    }
}
