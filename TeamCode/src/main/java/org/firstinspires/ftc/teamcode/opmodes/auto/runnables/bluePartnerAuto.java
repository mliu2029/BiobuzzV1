package org.firstinspires.ftc.teamcode.opmodes.auto.runnables;

import com.pedropathing.follower.Follower;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.Routines;

@NextAutonomous(name = "Blue Partner Auto", group = "Pedro")
public class bluePartnerAuto extends NextOpMode {
    private final Robot robot;
    private Follower follower;
    private final org.firstinspires.ftc.teamcode.opmodes.auto.paths.bluePartnerAuto paths =
            new org.firstinspires.ftc.teamcode.opmodes.auto.paths.bluePartnerAuto();
    private final Routines routines;
    private String currentPath = "Ready";

    public bluePartnerAuto(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        routines = new Routines(robot, status -> currentPath = status);
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.initialize(hardwareMap);
        robot.Turret().initialize(hardwareMap, Alliance.BLUE);
        follower = robot.Drivetrain().getFollower();
        follower.setPose(paths.startPose);
        follower.update();
        routines.bluePartnerAuto(paths).schedule();
    }

    @Override
    public void periodic() {
        if (follower == null) return;
        follower.update();
        telemetry.addData("Current Path", currentPath);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (degrees)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void end() {
        try {
            if (follower != null) {
                OpModeStorage.autonomousEndPose = follower.pose();
            }
        } finally {
            robot.stopImmediately();
        }
    }
}
