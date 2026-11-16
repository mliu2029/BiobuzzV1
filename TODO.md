# BioBuzz project TODOs

Updated November 2026. These reflect current mechanism configurations, verified software integrations,
and remaining on-field validation tasks.

Current setup: drivetrain, intake, single-flywheel shooter, transfer, ramp, turret, and Limelight are registered in
[`Robot.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java).
The Pedro follower is initialized with live pose/velocity suppliers. TeleOp runs under `MainTeleOp` with red/blue variants (`TeleopRed`, `TeleopBlue`) supporting odometry turret aiming with 2D Limelight tag-offset compensation, driver haptics, and live telemetry.
Autonomous runs modular 2-tip autos (`RedAuto`, `BlueAuto`) with Marrow prompt configuration.

## P0 — Finish the active mechanisms

- [x] **Verify the main TeleOps' intake control on the robot.** `MainTeleOp` provides dedicated intake, reverse, and stop commands with automated jam detection.
- [x] **Confirm intake direction and commands.** Added explicit `runIntake`, `reverseIntake`, and `stopIntake` commands with current and velocity stall monitoring.
- [x] **Confirm shooter motor and encoder scaling.** Converted to NextFTC pure rotations using `Units.Rotations.of(1.0 / 28.0)` for 6000 RPM bare motor (28 CPR) with REVERSE direction.
- [x] **Tune shooter and transfer roller PID and feedforward in RPM.** Velocity control running natively in RPM with NextFTC units.
- [x] **Show shooter readiness to the driver.** Added `Shooter.isReady()` to telemetry and driver haptics.
- [x] **Bring shooter documentation and tests up to date.** `VelocityPidTuning` and mechanism classes calibrated and documented in RPM.

## P1 — Restore feeding and autonomous shooting

- [x] **Calibrate the transfer blocker.** Calibrated open (`0.13`) and blocked (`0.7`) servo positions, plus roller reverse-hold mode.
- [x] **Restore transfer integration after calibration.** Transfer registered, autonomous and TeleOp feed controls integrated with servo and roller velocity.
- [x] **Gate feeding on shooter readiness.** Readiness check and timeouts enforced before disc feeding.
- [x] **Add bounded path failure handling.** Pedro follower path execution monitored with timeout/stalled path protection.
- [x] **Restore autonomous OpMode registration after integration.** `@NextAutonomous` registered for `RedAuto` and `BlueAuto` using modular 2-tip strategy with pose handoff to TeleOp.

## P2 — Localization, turret, and vision

- [x] **Expose the calibration procedures in the tuner menu.** Pedro tuning procedures registered.
- [ ] **Validate Pinpoint measurements before accepting calibration.** Validate on-field motion against real-world measurements.
- [ ] **Verify localization and follower tuning on the field.** Tune line, curve, and hold gains on the field tiles.
- [x] **Restore turret software integration.** Hardware initialization, mechanism registration, TeleOp aiming, manual recenter, and limits active.
- [x] **Calibrate odometry-based turret aiming.** Bearing calculation with Chassis-relative wrap-around and wrap limits (`-150°` to `+80°`).
- [x] **Calibrate and enable Limelight 2D offset correction.** Option 2 implemented: 2D horizontal angle tracking compensated by physical AprilTag cluster offset to goal center ($X=0$), avoiding 3D SolvePnP pose jitter.

## P3 — Keep setup and validation repeatable

- [ ] **Refresh the team setup and controls guide.** Update docs with confirmed button bindings, tuning parameters, and auto procedures.
- [ ] **Record a repeatable robot checkout.** Document build and pre-match inspection steps for driving, intake, shooter, turret, and auto-to-TeleOp pose handoff.
