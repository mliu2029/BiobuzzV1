package org.firstinspires.ftc.teamcode.opmodes.auto.runnables;

import com.pedropathing.follower.Follower;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.Routines;

/** Drives the supplied route without intake or shooter actions. */
@NextAutonomous(name = "Red Solo (2 Tip)", group = "Pedro")
public class redSolo2Tip extends NextOpMode {
    private final Robot robot;
    private final Follower follower;
    private final org.firstinspires.ftc.teamcode.opmodes.auto.paths.redSolo2Tip paths = new org.firstinspires.ftc.teamcode.opmodes.auto.paths.redSolo2Tip();
    private final Routines routines;
    private String currentPath = "Ready";

    public redSolo2Tip(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);
        follower = robot.getDrivetrain().getFollower();
        follower.setPose(paths.startPose);
        follower.update();
        routines = new Routines(robot, status -> currentPath = status);
        telemetry.update();
    }

    @Override
    public void start() {
        routines.redSolo2Tip(paths).schedule();
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
