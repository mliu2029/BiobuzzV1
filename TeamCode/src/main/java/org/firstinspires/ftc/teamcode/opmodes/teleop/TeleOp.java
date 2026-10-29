
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;

public abstract class TeleOp extends NextOpMode {
    private static final int VELOCITY_STEP = 25;
    protected final Robot robot;
    protected boolean started;
    private boolean transferOpen;

    protected TeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    private void configureControls() {
        CommandGamepad driver = new CommandGamepad(gamepad1);
        CommandGamepad driver2=new CommandGamepad(gamepad2);

        configureMechanismControls(driver);


        //debug tools for gamepad 2
        driver2.dpadLeft().onTrue(robot.Drivetrain().relocalize());
        driver2.triangle().onTrue(robot.Intake().stopIntake);
        driver2.circle().onTrue(robot.Intake().runIntake);
    }


    protected void configureMechanismControls(CommandGamepad driver) {
        driver.circle().onTrue(robot.Intake().runIntake);
        driver.square().onTrue(robot.Intake().runIntakeReverse);
        driver.dpadUp().onTrue(robot.Shooter().runFlywheel);
        driver.dpadDown().onTrue(robot.Shooter().stopFlywheel);
        driver.dpadRight().onTrue(robot.Shooter().instant(
                () -> robot.Shooter().adjustVelocity(VELOCITY_STEP)));
        driver.dpadLeft().onTrue(robot.Shooter().instant(
                () -> robot.Shooter().adjustVelocity(-VELOCITY_STEP)));
        driver.rightBumper().onTrue(Commands.instant(() -> setTransferOpen(true))
                .requiring(robot.Transfer()));
        driver.rightBumper().onFalse(Commands.instant(() -> setTransferOpen(false))
                .requiring(robot.Transfer()));
    }

    protected void setTransferOpen(boolean open) {
        transferOpen = open;
        robot.Transfer().setRollerVelocity(robot.Shooter().isEnabled()
                ? robot.Shooter().getTargetVelocity() : 0);
        if (open) {
            robot.Transfer().openImmediately();
        } else {
            robot.Transfer().closeImmediately();
        }
    }

    @Override
    public void start() {
        robot.initialize(hardwareMap);
        setTransferOpen(false);
        configureControls();
        Pose startingPose = OpModeStorage.autonomousEndPose;
        robot.Drivetrain().getFollower().setPose(
                startingPose != null ? startingPose : Pose.zero());
        robot.Drivetrain().getFollower().update();
        robot.startDrive(gamepad1);
        started = true;
    }

    @Override
    public void periodic() {
        if (!started) return;
        setTransferOpen(transferOpen);
        telemetry.addData("Shooter target (rad/s)", robot.Shooter().getTargetVelocity());
        telemetry.addData("Flywheel velocity (rad/s)", robot.Shooter().getFlywheelVelocity());
        telemetry.addData("Roller target (rad/s)", robot.Transfer().isFeeding()
                ? robot.Transfer().getRollerTargetVelocity() : 0.0);
        telemetry.addData("Roller velocity (rad/s)", robot.Transfer().getRollerVelocity());
        telemetry.addData("Transfer servo position", robot.Transfer().getPosition());
        telemetry.addData("Pose", robot.Drivetrain().getFollower().pose());
        telemetry.update();
    }

    @Override
    public void end() {
        started = false;
        transferOpen = false;
        robot.stopImmediately();
    }
}
