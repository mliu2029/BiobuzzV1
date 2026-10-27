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

    protected TeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);

        CommandGamepad driver = new CommandGamepad(gamepad1);

        driver.circle().onTrue(robot.getIntake().runIntake);
        driver.square().onTrue(robot.getIntake().runIntakeReverse);
        driver.dpadUp().onTrue(robot.getShooter().runFlywheel);
        driver.dpadDown().onTrue(robot.getShooter().stopFlywheel);
        driver.dpadRight().onTrue(robot.getShooter().instant(
                () -> robot.getShooter().adjustVelocity(VELOCITY_STEP)));
        driver.dpadLeft().onTrue(robot.getShooter().instant(
                () -> robot.getShooter().adjustVelocity(-VELOCITY_STEP)));

        // driver.rightBumper().onTrue(robot.getTransfer().openBlocker);
        // driver.rightBumper().onFalse(robot.getTransfer().closeBlocker);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        Pose startingPose = OpModeStorage.autonomousEndPose;
        robot.getDrivetrain().getFollower().setPose(
                startingPose != null ? startingPose : Pose.zero());
        robot.getDrivetrain().getFollower().update();
        robot.startDrive(gamepad1);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter target (rad/s)", robot.getShooter().getTargetVelocity());
        telemetry.addData("Flywheel velocity (rad/s)", robot.getShooter().getFlywheelVelocity());
        telemetry.addData("Flywheel2 velocity (rad/s)", robot.getShooter().getRollerVelocity());
        telemetry.addData("Pose", robot.getDrivetrain().getFollower().pose());
        telemetry.update();
    }

    @Override
    public void end() {
        robot.stopImmediately();
    }
}
