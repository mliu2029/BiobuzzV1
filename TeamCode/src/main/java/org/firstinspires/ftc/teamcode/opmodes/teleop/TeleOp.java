/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.data.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;

public abstract class TeleOp extends NextOpMode {
    private static final int VELOCITY_STEP = 25;
    protected final Robot robot;
    protected boolean started;

    protected TeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        telemetry.addData("Status", "Ready - press START");
        telemetry.update();
    }

    private void configureControls() {
        CommandGamepad driver = new CommandGamepad(gamepad1);
        CommandGamepad driver2=new CommandGamepad(gamepad2);

        configureMechanismControls(driver);

        configureTransferControls(driver);

        //debug tools for gamepad 2
        driver2.dpadLeft().onTrue(robot.Drivetrain().relocalize());
    }

    protected void configureTransferControls(CommandGamepad driver) {
        driver.rightBumper().onTrue(robot.Transfer().openBlocker);
        driver.rightBumper().onFalse(robot.Transfer().closeBlocker);
    }

    /** Override to keep tuning settings and gamepad mechanism controls in sync. */
    protected void configureMechanismControls(CommandGamepad driver) {
        driver.circle().onTrue(robot.Intake().runIntake);
        driver.square().onTrue(robot.Intake().runIntakeReverse);
        driver.dpadUp().onTrue(robot.Shooter().runFlywheel);
        driver.dpadDown().onTrue(robot.Shooter().stopFlywheel);
        driver.dpadRight().onTrue(robot.Shooter().instant(
                () -> robot.Shooter().adjustVelocity(VELOCITY_STEP)));
        driver.dpadLeft().onTrue(robot.Shooter().instant(
                () -> robot.Shooter().adjustVelocity(-VELOCITY_STEP)));

    }

    @Override
    public void start() {
        robot.initialize(hardwareMap);
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
        telemetry.addData("Shooter target (rad/s)", robot.Shooter().getTargetVelocity());
        telemetry.addData("Flywheel velocity (rad/s)", robot.Shooter().getFlywheelVelocity());
        telemetry.addData("Roller velocity (rad/s)", robot.Transfer().getRollerVelocity());
        telemetry.addData("Transfer servo position", robot.Transfer().getPosition());
        telemetry.addData("Pose", robot.Drivetrain().getFollower().pose());
        telemetry.update();
    }

    @Override
    public void end() {
        started = false;
        robot.stopImmediately();
    }
}
