package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.aiming.TurretCalculations.Alliance;
import org.firstinspires.ftc.teamcode.aiming.TurretConfig;

public abstract class BioBuzzTurretTeleop extends StarterBotTeleop {
    private final Alliance alliance;

    protected BioBuzzTurretTeleop(Robot robot, Alliance alliance) {
        super(robot, 1.0, 25);
        this.alliance = alliance;
        robot.turret.initialize(hardwareMap, () -> robot.drivetrain.getFollower().pose(), alliance);
        CommandGamepad operator = new CommandGamepad(gamepad2);
        operator.dpadUp().onTrue(robot.turret.aimAutomatically());
        operator.circle().onTrue(robot.turret.stop());
    }

    @Override
    public void disabledPeriodic() {
        telemetry.addLine("Turret encoder reset to forward zero.");
        telemetry.addData("TeleOp starting pose", OpModeStorage.autonomousEndPose);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret encoder ticks", robot.turret.getEncoderTicks());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        robot.turret.aimAutomatically().schedule();
    }

    @Override
    public void periodic() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Halfway Y", TurretConfig.halfwayY);
        telemetry.addData("Aiming at: ", robot.turret.getTarget());
        telemetry.addData("Aim mode", robot.turret.getMode());
        telemetry.addData("Turret degrees", robot.turret.getDegrees());
        telemetry.addData("Turret encoder ticks", robot.turret.getEncoderTicks());
        telemetry.addData("Target degrees", robot.turret.getTargetDegrees());
        telemetry.addData("Aligned (angular only)", robot.turret.isAligned());
        telemetry.addData("Pose", robot.drivetrain.getFollower().pose());
        super.periodic();
    }
}
