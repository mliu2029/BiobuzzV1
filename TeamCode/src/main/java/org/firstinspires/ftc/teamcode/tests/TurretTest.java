package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.Robot;

@NextTeleop(name = "Turret Test", group = "Tuning")
public class TurretTest extends NextOpMode {
    private static final double POWER = 0.3;
    // Assumes positive power increases the encoder count; flip the motor direction if not.
    private static final int MAX_POSITION = 55;
    private static final int MIN_POSITION = -55;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final DcMotorEx turret;

    private final DcMotorEx flywheel;

    private final DcMotorEx rollers;

    private final DcMotorEx intake;
    public TurretTest(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        rollers = hardwareMap.get(DcMotorEx.class, "rollers");
        intake = hardwareMap.get(DcMotorEx.class, "intake");



        // Position is measured from wherever the turret sits at INIT.
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);
        panelsTelemetry.setUpdateInterval(50);
        panelsTelemetry.addLine("Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void disabledPeriodic() {
        panelsTelemetry.addData("Position (ticks)", turret.getCurrentPosition());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void periodic() {
        int position = turret.getCurrentPosition();
        double output = 0;
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            output = POWER;
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            output = -POWER;
        }

        // Hard limits: only allow motion back toward the allowed range.
        if ((output > 0 && position >= MAX_POSITION) || (output < 0 && position <= MIN_POSITION)) {
            output = 0;
        }
        turret.setPower(output);

        if (gamepad1.squareWasPressed())   {
            flywheel.setPower(-0.7);
        }

        else if (gamepad1.circleWasPressed()){
            flywheel.setPower(0);
        }

        if (gamepad1.triangleWasPressed()){
            intake.setPower(-0.5);
            rollers.setPower(0.3);
        } if(gamepad1.xWasPressed()) {
            intake.setPower(0);
            rollers.setPower(0);

        }



        panelsTelemetry.addData("Position (ticks)", position);
        panelsTelemetry.addData("Velocity (ticks/s)", turret.getVelocity());
        panelsTelemetry.addData("Motor Power", output);
        panelsTelemetry.addData("At Limit", position >= MAX_POSITION || position <= MIN_POSITION);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void end() {
        turret.setPower(0);
        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.update(telemetry);
    }
}
