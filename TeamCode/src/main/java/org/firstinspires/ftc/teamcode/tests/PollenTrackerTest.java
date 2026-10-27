package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.zones.Point;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import java.util.List;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.vision.PollenLocator;
import org.firstinspires.ftc.teamcode.vision.PollenVisionConfig;

/**
 * Drive around and read pollen positions in Pedro coordinates. Cross plans a Weaver path
 * through the visible pollen and follows it; Circle cancels back to manual driving.
 * The robot starts at the autonomous end pose, so place it where that pose says.
 *
 * Telemetry and the webcam go to Panels: open the Camera Stream panel to see every
 * pollen and nectar circled, and the planned path drawn with an arrow per pollen.
 */
@NextTeleop(name = "Pollen Tracker Test", group = "Tuning")
public class PollenTrackerTest extends NextOpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Robot robot;
    private final PollenLocator pollenLocator;
    private Follower follower;
    private PathResult plan;
    private boolean following;
    private boolean lastCross;

    public PollenTrackerTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);
        pollenLocator = new PollenLocator(hardwareMap);
        PanelsCameraStream.INSTANCE.startStream(pollenLocator.getStream(), PollenVisionConfig.streamFps);
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower = robot.getDrivetrain().getFollower();
        Pose startingPose = OpModeStorage.autonomousEndPose;
        follower.setPose(startingPose != null ? startingPose : Pose.zero());
        follower.update();
    }

    @Override
    public void periodic() {
        Pose pose = follower.pose();

        if (gamepad1.cross && !lastCross) {
            plan = pollenLocator.planPath(pose);
            if (plan != null) {
                Path path = PedroPathingConverter.toPath(plan);
                follower.follow(path);
                following = true;
            }
        }
        lastCross = gamepad1.cross;
        if (gamepad1.circle || (following && !follower.isBusy())) {
            following = false;
            follower.stop();
        }

        if (!following) {
            double forward = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;
            double scale = 1.0 / Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate));
            follower.manual(forward * scale, -strafe * scale, -rotate * scale);
        }
        follower.update();
        pollenLocator.updateOverlay(pose, following ? plan : null);

        List<Pose> pollen = pollenLocator.getPollen(pose);
        panelsTelemetry.addData("Robot", format(pose));
        panelsTelemetry.addData("Following", following);
        panelsTelemetry.addData("Stream frames", pollenLocator.getStreamFramesSent());
        panelsTelemetry.addData("Pollen seen", pollen.size());
        for (int i = 0; i < pollen.size(); i++) {
            panelsTelemetry.addData("Pollen " + i, format(pollen.get(i)));
        }
        for (Pose nectar : pollenLocator.getNectar(pose)) {
            panelsTelemetry.addData("Nectar", format(nectar));
        }
        if (plan != null) {
            StringBuilder order = new StringBuilder();
            for (Point target : plan.getVisitedTargets()) {
                order.append(String.format("(%.1f, %.1f) ", target.getX(), target.getY()));
            }
            panelsTelemetry.addData("Last plan", order.toString());
            panelsTelemetry.addData("Skipped", plan.getUnvisitedTargets().size());
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        PanelsCameraStream.INSTANCE.stopStream();
        pollenLocator.close();
        robot.stopImmediately();
    }

    private static String format(Pose pose) {
        return String.format("x=%.1f y=%.1f h=%.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
