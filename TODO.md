# BioBuzz project TODOs

Based on the source checked on October 24, 2026. These are planned tasks, not
completed features or verified hardware settings. Start with P0, then P1.

Current setup: drivetrain, intake, dual-motor shooter, turret, and camera are registered in
[`Robot.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java).
The Pedro follower is initialized. Red/blue turret TeleOps initialize the turret
and optional camera; turret calibration is still required and vision defaults off.
Transfer remains disabled, and the autonomous OpMode annotations are commented out.

## P0 — Finish the active mechanisms

- [ ] **Verify the main TeleOps' intake stop control on the robot.** Red and blue
  turret TeleOps share `opmodes/teleop/BioBuzzTurretTeleop.java`. Gamepad 1 Cross/X
  stops intake; verify it works during PLAY without ending the OpMode.
- [ ] **Confirm intake direction and power on the robot.** Resolve the direction
  TODO in `mechanisms/Intake.java`. Check that the TeleOp's +1 intake command and
  the tuner's -0.7 command move game pieces in the intended directions.
- [ ] **Confirm both shooter motors and encoder signs.** `mechanisms/Shooter.java`
  currently looks up `flywheel` and `rollers`, with REVERSE and FORWARD directions
  respectively. Match the Robot Controller configuration and verify both report
  positive ticks/s when shooting. Confirm the tuner's 28 ticks/revolution matches
  the motors before trusting its RPM display.
- [ ] **Tune the dual-motor shooter PID and feedforward.** Use
  `opmodes/calib/VelocityPidTuning.java` to record each motor's speed and power during
  spin-up, steady running, and feeding. Resolve the shooter tuning TODO, save
  measured gains, and check that both motors recover to the target after a shot.
- [ ] **Show shooter readiness to the driver.** Add `Shooter.isReady()` and each
  motor's target error to TeleOp telemetry; choose an indicator or rumble for
  readiness. Verify readiness drops when either motor falls below its threshold.
- [ ] **Bring shooter documentation and tests up to date.** The team notes still
  describe `flywheel2`, two FORWARD directions, and zero default gains;
  `ShooterTest` also expects the first motor to be FORWARD. After confirming the
  intended hardware setup, update those expectations and run the shooter tests.

## P1 — Restore feeding and autonomous shooting

- [ ] **Calibrate the transfer blocker.** Measure distinct open and blocked servo
  positions in `mechanisms/Transfer.java`; both are currently 0.5. Verify the
  blocked position holds a piece and the open position releases it reliably.
- [ ] **Restore transfer integration after calibration.** Re-enable the servo,
  mechanism registration, TeleOp controls, and autonomous feed commands together.
  Verify INIT, button release, and STOP all close the blocker; remove or connect
  the tuner's currently inactive `runTransfer` setting.
- [ ] **Gate feeding on shooter readiness.** Replace the autos' fixed 500 ms
  spin-up assumption with a bounded readiness wait. Add a timeout that closes the
  blocker and stops feeding if both shooter motors do not reach speed. Verify
  both normal shooting and a motor that never reaches its target.
- [ ] **Tune feed duration and recovery between shots.** The autos currently wait
  5000 ms while feeding. Measure the required feed time and pause feeding when
  speed drops so later pieces are not launched before the shooter recovers.
- [ ] **Complete the second shooting cycle.**
  `opmodes/auto/runnables/RedTestReturnToShootAuto.java` returns to the shooting pose
  but goes straight to parking. Add a second readiness/feed/close sequence and
  verify it actually fires the newly collected pieces before parking.
- [ ] **Add bounded path failure handling.** `Drivetrain.followPath()` waits for
  follower completion without an explicit timeout in this wrapper. Define a time
  limit and stop behavior for a stalled path, then verify the auto cannot remain
  stuck indefinitely.
- [ ] **Restore autonomous OpMode registration after integration.** Re-enable the
  intended `@NextAutonomous` entries once feeding is functional. Check that each
  appears on the Driver Station, follows the existing field geometry, parks,
  stops all outputs, and saves its end pose for TeleOp.

## P2 — Localization, turret, and vision

- [ ] **Expose the calibration procedures in the tuner menu.** `pedro/Tuning.java`
  currently registers only `tests()`. Add entries for the existing Pinpoint,
  mecanum, and Foresight procedures and confirm they appear in the tuning UI.
- [ ] **Validate Pinpoint measurements before accepting calibration.** In
  `pedro/procedures/PinpointTuner.java`, show live direction readings, reject
  missing/nonfinite movement, and validate the requested 180-degree turn before
  calculating offsets. Add focused checks for invalid inputs and preserve the
  final valid readings when STOP resets the pose.
- [ ] **Verify localization and follower tuning on the field.** Check forward,
  strafe, and rotation readings against measured motion, then run the existing
  line, curve, and hold tests. Save hardware-verified offsets and gains in
  `pedro/Constants.java` only after the measurements agree.
- [x] **Restore turret software integration.** Re-enable hardware initialization,
  mechanism registration, red/blue TeleOps, aiming controls, telemetry, and STOP.
- [ ] **Calibrate odometry-based turret aiming first.** Measure encoder
  scale/sign, forward zero, travel limits, and PID/static power in
  `data/TurretConfig.java`; the current zero limits and gains are unfinished.
  Verify aiming and stopping across robot headings before enabling vision.
- [ ] **Calibrate and enable Limelight correction.** Confirm the pipeline,
  accepted tag IDs, camera sign, and per-tag aiming offsets. Verify wrong tags,
  stale frames, repeated frames, and target loss fall back to odometry correctly.
  Follow the existing [Limelight notes](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/notes/LIMELIGHT_README.md).

## P3 — Keep setup and validation repeatable

- [ ] **Refresh the team setup and controls guide.** Update
  [the team notes](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/notes/readme.md)
  with the confirmed hardware names, active OpModes, button bindings, tuning
  values, and enabled mechanisms. Distinguish currently usable controls from
  planned transfer and turret controls.
- [ ] **Record a repeatable robot checkout.** Document the build/install steps and
  a short INIT/PLAY/STOP test for driving, intake, shooter, blocker, localization,
  and auto-to-TeleOp pose transfer. Record software check results separately from
  physical robot results, including the battery voltage used for tuning.
