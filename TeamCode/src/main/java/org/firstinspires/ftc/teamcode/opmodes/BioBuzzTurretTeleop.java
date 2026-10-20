package org.firstinspires.ftc.teamcode.opmodes;

import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Calculations.Alliance;

public abstract class BioBuzzTurretTeleop extends TeleOp {
    private final Alliance alliance;

    protected BioBuzzTurretTeleop(Robot robot, Alliance alliance) {
        super(robot);
        this.alliance = alliance;
        Turret.INSTANCE.initialize(hardwareMap, alliance);
        CommandGamepad operator = new CommandGamepad(gamepad2);
        operator.dpadUp().onTrue(Turret.INSTANCE.aimTurret);
        operator.circle().onTrue(Turret.INSTANCE.stopTurret);
    }

    @Override
    public void disabledPeriodic() {
        telemetry.addLine("Turret encoder reset to forward zero.");
        telemetry.addData("TeleOp starting pose", OpModeStorage.autonomousEndPose);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret encoder ticks", Turret.INSTANCE.getEncoderTicks());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        Turret.INSTANCE.aimTurret.schedule();
    }

    @Override
    public void periodic() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Aiming at: ", Turret.INSTANCE.getTarget());
        telemetry.addData("Aim mode", Turret.INSTANCE.getMode());
        telemetry.addData("Turret degrees", Turret.INSTANCE.getDegrees());
        telemetry.addData("Turret encoder ticks", Turret.INSTANCE.getEncoderTicks());
        telemetry.addData("Target degrees", Turret.INSTANCE.getTargetDegrees());
        telemetry.addData("Aligned: ", Turret.INSTANCE.isAligned());
        telemetry.addData("Pose", Drivetrain.INSTANCE.getFollower().pose());
        super.periodic();
    }
}
