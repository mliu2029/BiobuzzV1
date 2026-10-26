package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;
import org.firstinspires.ftc.teamcode.mechanisms.*;

/** Owns the mechanisms shared by this OpMode. */
public class Robot implements NextRobot {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Limelight limelight = new Limelight();
    private final Turret turret = new Turret(drivetrain, limelight);
    private final Transfer transfer = new Transfer();

    private final Set<Mechanism> mechanisms = Collections.unmodifiableSet(
            new LinkedHashSet<>(Arrays.asList(drivetrain, intake, shooter, limelight, turret)));

    public Drivetrain getDrivetrain() { return drivetrain; }
    public Intake getIntake() { return intake; }
    public Shooter getShooter() { return shooter; }
    public Limelight getLimelight() { return limelight; }
    public Turret getTurret() { return turret; }
    public Transfer getTransfer() { return transfer; }

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    public void initialize(HardwareMap hardwareMap) {
        drivetrain.initialize(hardwareMap);
        shooter.initialize();
        intake.stopImmediately();
        turret.stopImmediately();
        limelight.stopImmediately();
    }

    public void startDrive(Gamepad gamepad) {
        drivetrain.drive(gamepad).schedule();
    }

    public Command stop() {
        return Commands.instant(this::stopImmediately).requiring(mechanisms.toArray());
    }

    public void stopImmediately() {
        drivetrain.stopImmediately();
        intake.stopImmediately();
        shooter.stopImmediately();
        turret.stopImmediately();
        limelight.stopImmediately();
    }
}
