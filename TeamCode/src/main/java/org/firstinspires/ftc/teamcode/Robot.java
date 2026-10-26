package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
// import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
// import org.firstinspires.ftc.teamcode.subsystems.Transfer;
// import org.firstinspires.ftc.teamcode.subsystems.Turret;

/** Registers the drivetrain, intake, and shooter. */
public class Robot implements NextRobot {
    private final Set<Mechanism> subsystems = Collections.unmodifiableSet(
            new LinkedHashSet<>(Arrays.asList(Drivetrain.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE
                    // , Transfer.INSTANCE,
                    // Limelight.INSTANCE, Turret.INSTANCE
            )));

    @Override
    public Set<Mechanism> getMechanisms() {
        return subsystems;
    }

    public void initialize(HardwareMap hardwareMap) {
        Drivetrain.INSTANCE.initialize(hardwareMap);
        Shooter.INSTANCE.initialize(hardwareMap);
        Intake.INSTANCE.stopImmediately();
        // Transfer, turret, and camera hardware remain disabled.
        // Transfer.INSTANCE.stopImmediately();
        // Turret.INSTANCE.stopImmediately();
        // Limelight.INSTANCE.stopImmediately();
    }

    public Command stop() {
        return Commands.instant(this::stopImmediately).requiring(subsystems.toArray());
    }

    public void stopImmediately() {
        Drivetrain.INSTANCE.stopImmediately();
        Intake.INSTANCE.stopImmediately();
        Shooter.INSTANCE.stopImmediately();
        // Transfer.INSTANCE.stopImmediately();
        // Turret.INSTANCE.stopImmediately();
        // Limelight.INSTANCE.stopImmediately();
    }
}
