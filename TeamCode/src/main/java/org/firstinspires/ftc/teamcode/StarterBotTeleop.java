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

package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;

public abstract class StarterBotTeleop extends NextOpMode {
    protected final Robot robot;
    private final Command driveCommand;

    protected StarterBotTeleop(Robot robot, double driveScale, int velocityStep) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap, gamepad1);

        CommandGamepad driver = new CommandGamepad(gamepad1);
        driveCommand = robot.drivetrain.drive(
                () -> -gamepad1.right_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_stick_x,
                driveScale);

        // Shooter's default command handles D-pad up/down.
        driver.circle().onTrue(robot.intake.run());
        driver.square().onTrue(robot.intake.reverse());
        driver.dpadRight().onTrue(robot.shooter.instant(
                () -> robot.shooter.adjustVelocity(velocityStep)));
        driver.dpadLeft().onTrue(robot.shooter.instant(
                () -> robot.shooter.adjustVelocity(-velocityStep)));

        // One held command preserves left-bumper priority when both are pressed.
        driver.leftBumper().or(() -> gamepad1.right_bumper)
                .whileTrue(robot.transfer.run(() -> gamepad1.left_bumper ? -1.0 : 1.0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        driveCommand.schedule();
    }

    @Override
    public void periodic() {
        telemetry.addData("TargetVelocity", robot.shooter.getTargetVelocity());
        telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
        telemetry.update();
    }

    @Override
    public void end() {
        robot.stopImmediately();
    }
}
