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
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

public class Robot implements NextRobot {
    public final Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Transfer transfer = new Transfer();

    private final Set<Mechanism> mechanisms = Collections.unmodifiableSet(
            new LinkedHashSet<>(Arrays.asList(drivetrain, intake, shooter, transfer)));

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    /** Call from any NextOpMode constructor to begin with all outputs stopped. */
    public void initialize(HardwareMap hardwareMap) {
        initialize(hardwareMap, null);
    }

    public void initialize(HardwareMap hardwareMap, Gamepad driver) {
        drivetrain.initialize(hardwareMap);
        intake.stopImmediately();
        shooter.initialize(hardwareMap, driver);
        transfer.stopImmediately();
    }

    /** A composable stop command that owns every mechanism. */
    public Command stop() {
        return Commands.instant(this::stopImmediately).requiring(mechanisms.toArray());
    }

    public void stopImmediately() {
        drivetrain.stopImmediately();
        intake.stopImmediately();
        shooter.stopImmediately();
        transfer.stopImmediately();
    }
}
