package org.firstinspires.ftc.teamcode.autotune;

import com.aaravdhawan25.pidautotuner.ftc.PIDMaster;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Auto-tunes PIDF gains for the shooter ("launcher") velocity loop using
 * FtcAutoTune's relay-feedback method for P/I/D and an open-loop sweep for kF.
 * Adapted from FtcAutoTune's VelocityPIDFTunerOpMode template.
 *
 * <p>Run this on its own, with nothing feeding the shooter. It takes over the
 * raw motor (resets the encoder, RUN_WITHOUT_ENCODER), so don't run it from
 * inside the NextFTC robot/mechanism code.
 *
 * <p>Telemetry goes through Panels (http://192.168.43.1:8001, Telemetry tab)
 * and is mirrored to the Driver Station by {@code panels.update(telemetry)}.
 *
 * <ol>
 *   <li>During INIT, adjust {@link ShooterTuningConfig} in the Panels Configurables tab
 *       (motor name, target ticks/sec, relay amplitude, timeout, REVERSED).</li>
 *   <li>Press START. Phase 1: the wheel oscillates its speed around the target
 *       (gives Ku and Tu). Phase 2: it runs open-loop at a few fixed powers
 *       (gives kF).</li>
 *   <li>Read the candidate gain sets off Panels or Driver Station telemetry.</li>
 *   <li>Hold gamepad1.a to live-test the "classic ZN" candidate at the target speed.</li>
 * </ol>
 */
@TeleOp(name = "AutoTune | Shooter Velocity PIDF", group = "AutoTune")
public class ShooterVelocityAutoTune extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // INIT: show the current ShooterTuningConfig values. They are @Configurable, so edit them
        // in the Panels Configurables tab now; they are read once when START is pressed.
        while (opModeInInit()) {
            double target = ShooterTuningConfig.VELOCITY_TARGET_TICKS_PER_SEC;
            panels.addLine("=== Shooter PIDF Auto Tuner ===");
            panels.addLine("Edit ShooterTuningConfig in Panels > Configurables, then press START.");
            panels.addData("Motor", ShooterTuningConfig.MOTOR_NAME);
            panels.addData("Target (ticks/s)", String.format("%.1f", target));
            panels.addData("Target (RPM)", String.format("%.1f", ShooterTuningConfig.toRPM(target)));
            panels.addData("Relay amplitude", ShooterTuningConfig.RELAY_AMPLITUDE);
            panels.addData("Relay hysteresis (ticks/s)", ShooterTuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC);
            panels.addData("Relay timeout (s)", ShooterTuningConfig.RELAY_TEST_TIMEOUT_S);
            panels.addData("Cycles to collect / ignore",
                    ShooterTuningConfig.CYCLES_TO_COLLECT + " / " + ShooterTuningConfig.CYCLES_TO_IGNORE);
            panels.addLine("Clear the shooter before starting. Phase 1: relay test (automatic).");
            panels.update(telemetry);
        }
        if (isStopRequested()) return;

        // START: snapshot the config and build the tuner from it.
        double targetTicksPerSec = ShooterTuningConfig.VELOCITY_TARGET_TICKS_PER_SEC;
        PIDMaster pid = new PIDMaster(
                hardwareMap, ShooterTuningConfig.MOTOR_NAME, ShooterTuningConfig.REVERSED, false,
                targetTicksPerSec, ShooterTuningConfig.VELOCITY_HYSTERESIS_TICKS_PER_SEC,
                ShooterTuningConfig.RELAY_AMPLITUDE, ShooterTuningConfig.CYCLES_TO_COLLECT,
                ShooterTuningConfig.CYCLES_TO_IGNORE, ShooterTuningConfig.RELAY_TEST_TIMEOUT_S,
                ShooterTuningConfig.FEEDFORWARD_TEST_POWERS, ShooterTuningConfig.FEEDFORWARD_SETTLE_TIME_S,
                ShooterTuningConfig.TUNE_INTEGRAL_TERM, ShooterTuningConfig.TICKS_PER_REV);

        try {
            // Phase 1 (relay test) + Phase 2 (feedforward sweep) run automatically.
            while (opModeIsActive() && !pid.isTuningComplete()) {
                pid.tuningStep(getRuntime());
                for (String line : pid.getTelemetryLines()) panels.addLine(line);
                panels.update(telemetry);
            }

            if (pid.timedOut() || !pid.isTuningSuccessful()) {
                while (opModeIsActive()) {
                    for (String line : pid.getTelemetryLines()) panels.addLine(line);
                    panels.update(telemetry);
                }
                return;
            }

            // Phase 3: show results, optionally live-test with gamepad1.a.
            while (opModeIsActive()) {
                for (String line : pid.getResultTelemetryLines()) panels.addLine(line);
                panels.addLine("");
                panels.addLine("Copy kP/kI/kD/kF into NextMotor.velocityConstants (withP/withI/withD/withV).");

                if (gamepad1.a) {
                    panels.addLine("");
                    for (String line : pid.liveTestStep(getRuntime())) panels.addLine(line);
                } else {
                    pid.stopLiveTest();
                }

                panels.update(telemetry);
            }
        } finally {
            pid.stop();
        }
    }
}
