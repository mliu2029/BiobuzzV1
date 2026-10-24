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

package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
// import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;

public abstract class TeleOp extends NextOpMode {
    private static final int VELOCITY_STEP = 25;
    protected final Robot robot;
    private final Command driveCommand;

    protected TeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);

        CommandGamepad driver = new CommandGamepad(gamepad1);
        driveCommand = Drivetrain.INSTANCE.drive(
                () -> -gamepad1.right_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_stick_x);

        driver.circle().onTrue(Intake.INSTANCE.runIntake);
        driver.square().onTrue(Intake.INSTANCE.runIntakeReverse);
        driver.dpadUp().and(driver.dpadDown().negate()).onTrue(Shooter.INSTANCE.runFlywheel);
        driver.dpadDown().onTrue(Shooter.INSTANCE.stopFlywheel);
        driver.dpadRight().onTrue(Shooter.INSTANCE.instant(
                () -> Shooter.INSTANCE.adjustVelocity(VELOCITY_STEP)));
        driver.dpadLeft().onTrue(Shooter.INSTANCE.instant(
                () -> Shooter.INSTANCE.adjustVelocity(-VELOCITY_STEP)));

        // driver.rightBumper().onTrue(Transfer.INSTANCE.openBlocker);
        // driver.rightBumper().onFalse(Transfer.INSTANCE.closeBlocker);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        Pose startingPose = OpModeStorage.autonomousEndPose;
        Drivetrain.INSTANCE.getFollower().setPose(
                startingPose != null ? startingPose : Pose.zero());
        Drivetrain.INSTANCE.getFollower().update();
        driveCommand.schedule();
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter target (ticks/s)", Shooter.INSTANCE.getTargetVelocity());
        telemetry.addData("Flywheel velocity (ticks/s)", Shooter.INSTANCE.getVelocity1());
        telemetry.addData("Flywheel2 velocity (ticks/s)", Shooter.INSTANCE.getVelocity2());
        telemetry.addData("Pose", Drivetrain.INSTANCE.getFollower().pose());
        telemetry.update();
    }

    @Override
    public void end() {
        robot.stopImmediately();
    }
}
