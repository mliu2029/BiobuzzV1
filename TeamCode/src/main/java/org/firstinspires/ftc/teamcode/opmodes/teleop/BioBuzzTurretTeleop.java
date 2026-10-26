package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.data.Alliance;

public abstract class BioBuzzTurretTeleop extends TeleOp {
    private final Alliance alliance;

    protected BioBuzzTurretTeleop(Robot robot, Alliance alliance) {
        super(robot);
        this.alliance = alliance;
        robot.getTurret().initialize(hardwareMap, alliance);
        CommandGamepad operator = new CommandGamepad(gamepad2);
        operator.dpadUp().onTrue(robot.getTurret().aimTurret);
        operator.circle().onTrue(robot.getTurret().stopTurret);
    }

    @Override
    public void disabledPeriodic() {
        telemetry.addLine("Turret encoder reset to forward zero.");
        telemetry.addData("TeleOp starting pose", OpModeStorage.autonomousEndPose);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret encoder ticks", robot.getTurret().getEncoderTicks());
        telemetry.addData("Limelight", robot.getLimelight().isAvailable()
                ? "Configured" : "Unavailable (check USB/config)");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        robot.getTurret().aimTurret.schedule();
    }

    @Override
    public void periodic() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Aiming at: ", robot.getTurret().getTarget());
        telemetry.addData("Aim mode", robot.getTurret().getMode());
        telemetry.addData("Limelight", robot.getLimelight().isAvailable()
                ? "Configured" : "Unavailable (check USB/config)");
        telemetry.addData("Turret degrees", robot.getTurret().getDegrees());
        telemetry.addData("Turret encoder ticks", robot.getTurret().getEncoderTicks());
        telemetry.addData("Target degrees", robot.getTurret().getTargetDegrees());
        telemetry.addData("Aligned: ", robot.getTurret().isAligned());
        telemetry.addData("Pose", robot.getDrivetrain().getFollower().pose());
        super.periodic();
    }
}
