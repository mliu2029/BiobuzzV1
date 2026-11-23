package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import java.util.Set;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class Robot implements NextRobot {
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Shooter shooter;
    public final Limelight limelight;
    public final Turret turret;
    public final Transfer transfer;
    public final Ramp ramp;

    public Robot() {
        NextMotor.Companion.getMotorEventLoop().clear();
        drivetrain = new Drivetrain();
        intake = new Intake();
        shooter = new Shooter();
        limelight = new Limelight();
        turret = new Turret(limelight);
        transfer = new Transfer();
        ramp = new Ramp();
    }

    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain, intake, shooter, transfer, limelight, turret, ramp);
    }

    public void prepareForInit(HardwareMap hardwareMap) {
        Scheduler.reset();
        turret.stopImmediately();
        drivetrain.stopImmediately();
        shooter.prepareForInit();
        intake.stopImmediately();
        transfer.prepareForInit();
        ramp.initialize();
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            motor.setPower(0);
        }
    }

    public void initialize(HardwareMap hardwareMap) {
        initialize(hardwareMap, null, 0.0);
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance) {
        initialize(hardwareMap, alliance, 0.0);
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance, double startingTurretDegrees) {
        drivetrain.initialize(hardwareMap);
        shooter.initialize();
        transfer.initialize();
        intake.initialize();
        ramp.initialize();
        if (alliance != null) {
            turret.initialize(hardwareMap, alliance, startingTurretDegrees);
        } else {
            turret.stopImmediately();
        }
    }

    public void startDrive(Gamepad gamepad) {
        drivetrain.drive(gamepad).schedule();
    }

    public void stopImmediately() {
        drivetrain.stopImmediately();
        intake.stopImmediately();
        shooter.stopImmediately();
        transfer.stopImmediately();
        turret.stopImmediately();
    }
}
