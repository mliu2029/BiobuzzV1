package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

/** Alliance turret test with automatic aiming and the standard gamepad 1 drivetrain controls. */
public abstract class BioBuzzTurretTeleop extends TeleOp {
    private final Alliance alliance;

    protected BioBuzzTurretTeleop(Robot robot, Alliance alliance) {
        super(robot);
        this.alliance = alliance;
    }

    @Override
    public void disabledPeriodic() {
        telemetry.addLine("START initializes the turret and resets its forward zero.");
        addControlTelemetry();
        telemetry.addLine("PLAY starts driving and automatic turret aiming.");
        telemetry.addData("TeleOp starting pose", OpModeStorage.autonomousEndPose);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Limelight", "Not initialized until START");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start(); // Restore the pose and start continuous gamepad 1 driving.
        robot.Turret().initialize(hardwareMap, alliance);
        CommandGamepad operator = new CommandGamepad(gamepad2);
        operator.dpadUp().onTrue(robot.Turret().aimTurret);
        operator.circle().onTrue(robot.Turret().stopTurret);
        robot.Turret().aimTurret.schedule();
    }

    @Override
    public void periodic() {
        if (!started) return;
        addControlTelemetry();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Aiming at: ", robot.Turret().getTarget());
        telemetry.addData("Aim mode", robot.Turret().getMode());
        telemetry.addData("Limelight", robot.Limelight().isAvailable()
                ? "Configured" : "Unavailable (check USB/config)");
        telemetry.addData("Turret degrees", robot.Turret().getDegrees());
        telemetry.addData("Turret encoder ticks", robot.Turret().getEncoderTicks());
        telemetry.addData("Target degrees", robot.Turret().getTargetDegrees());
        telemetry.addData("Aligned: ", robot.Turret().isAligned());
        telemetry.addData("Pose", robot.Drivetrain().getFollower().pose());
        super.periodic();
    }

    private void addControlTelemetry() {
        telemetry.addLine("Gamepad 1: right stick drives/strafes; left stick X turns.");
        telemetry.addLine("Gamepad 2: Circle stops auto aim; D-pad up resumes it.");
    }
}
