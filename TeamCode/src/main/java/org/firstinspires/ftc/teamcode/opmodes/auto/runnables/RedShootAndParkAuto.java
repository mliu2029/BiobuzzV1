package org.firstinspires.ftc.teamcode.opmodes.auto.runnables;

import com.pedropathing.follower.Follower;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.Routines;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.ShootAndParkPaths;
import org.firstinspires.ftc.teamcode.robot.Robot;

// Registration remains disabled, as before the reorganization.
// @NextAutonomous(name = "red hive+park", group = "Pedro")
public class RedShootAndParkAuto extends NextOpMode {
    private final Robot robot;
    private final Follower follower;
    private final ShootAndParkPaths paths = new ShootAndParkPaths();
    private final Routines routines;
    private String currentPath = "Ready";

    public RedShootAndParkAuto(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);
        follower = robot.getDrivetrain().getFollower();
        follower.setPose(paths.startPose);
        follower.update();
        routines = new Routines(robot, status -> currentPath = status);
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        routines.shootAndPark(paths).schedule();
    }

    @Override
    public void periodic() {
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
            OpModeStorage.autonomousEndPose = follower.pose();
        } finally {
            robot.stopImmediately();
        }
    }
}
