
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
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

public class Robot implements NextRobot {
    public final Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Transfer transfer = new Transfer();
    public final Turret turret = new Turret();

    private final Set<Mechanism> mechanisms = Collections.unmodifiableSet(
            new LinkedHashSet<>(Arrays.asList(drivetrain, intake, shooter, transfer, turret)));

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    public void initialize(HardwareMap hardwareMap) {
        initialize(hardwareMap, null);
    }

    public void initialize(HardwareMap hardwareMap, Gamepad driver) {
        drivetrain.initialize(hardwareMap);
        intake.stopImmediately();
        shooter.initialize(hardwareMap, driver);
        transfer.stopImmediately();
    }

    public Command stop() {
        return Commands.instant(this::stopImmediately).requiring(mechanisms.toArray());
    }

    public void stopImmediately() {
        drivetrain.stopImmediately();
        intake.stopImmediately();
        shooter.stopImmediately();
        transfer.stopImmediately();
        turret.shutdown();
    }
}
