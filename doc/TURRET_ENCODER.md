# Turret relative encoder setup

The turret uses a REV Through Bore Encoder V2 (REV-11-3174) connected by the
6-pin-to-4-pin cable to the encoder input corresponding to its existing motor
power port. The code uses the hardware-map motor named **`turret`**. Plug the
REV encoder into that motor port's encoder input; confirm the configured hub
and port match the physical wiring.
No analog, PWM absolute input, or additional hardware-map sensor is needed.

REV specifies **8192 quadrature counts per encoder revolution** (2048 A/B cycles).
The FTC motor encoder input supports incremental feedback, not the absolute
pulse output. See [REV specifications](https://www.revrobotics.com/rev-11-3174/)
and [REV encoder documentation](https://docs.revrobotics.com/rev-crossover-products/sensors/tbe).

## Gear ratio and direction

`TurretConfig.encoderRevolutionsPerTurretRevolution` means **encoder shaft
revolutions per one turret revolution**. The 30T encoder gear meshing
with the existing 92T turret gear sets the default to `92.0 / 30.0`. If gears change,
update this value; do not use integer division or include the motor's internal
gearbox ratio. For a direct gear mesh it is `turret teeth / encoder gear teeth`.

```text
countsPerTurretRevolution = 8192 * encoderRevolutionsPerTurretRevolution
degreesPerTick = encoderDirection * 360 / countsPerTurretRevolution
turretDegrees = startingDegrees + (rawTicks - zeroTicks) * degreesPerTick
```

At this ratio there are approximately 69.78370 counts per turret degree.
An approximately 230-degree sweep produces approximately 16,050.3 counts.
Do not force the turret through its full travel to calibrate it.

`encoderDirection` accepts `+1` or `-1`. With motor output disabled, move the
turret a small known angle counterclockwise (viewed from above): turret angle
must increase. If raw counts decrease, set `encoderDirection = -1`. Gear mesh
and sensor mounting affect polarity, so the default `+1` must be verified on
the robot. This setting reverses feedback only. Positive motor power retains
its existing direction, expected to move the turret counterclockwise.

The ratio must be positive and finite, and direction must be exactly +1 or -1.
An invalid configuration stops turret output in manual and automatic modes;
raw tick telemetry remains available. Settings are configurable in Panels.
Live scale changes clear PID/vision state while preserving the captured raw
zero tick. Set the verified values in source for future app starts.

## Zero, limits, and PID

Initialization still stops the motor, resets the relative encoder count, and
selects `RUN_WITHOUT_ENCODER` with BRAKE. The startup physical position must
match the caller's `startingDegrees` (normally zero, or the existing explicit
autonomous handoff). Resetting ticks does not find physical center or a hard
stop. No absolute startup position or automatic homing is introduced.
Limelight no longer estimates the turret angle or learns an encoder-zero
correction. Vision still selects aiming targets; feedback and PID angle use
only the REV encoder with the explicit startup/manual zero.

The existing manual recenter control is unchanged: square enters manual mode,
bumpers move the turret, and square again treats the current position as zero
and resumes aiming. The zero is stored in raw ticks so ratio/direction changes
do not move that captured reference. Limits remain -150 to +80 degrees; they
depend on the correct physical zero and scale.

One SDK motor object supplies `turretMotor.getCurrentPosition()` to raw tick
telemetry and to NextMotor's position feedback. NextMotor scales those counts
to degrees before its PID calculation. Setpoints, PID errors, limits,
alignment tolerance, vision tracking, static-power taper, and measured velocity
all remain in turret angular units. The existing non-wrapping position mode is
retained for the limited-travel turret.

Odometry selects the alliance's close/far hive from robot Y and computes the
coarse target angle. When vision is enabled, a fresh tag for that selected hive
within `TurretConfig.acquireDegrees` refines the angle using its horizontal
error and the opening-center offset. The camera uses a deadband of
`TurretConfig.visionUpdateDegrees`; repeated reads of one frame hold its
setpoint. Missing, stale, wrong-pipeline, wrong-hive, or out-of-window camera
results leave odometry in control. Without a pose, automatic aiming stops.

One encoder PID handles both stages: P = 0.01, I = 0, D = 0.000125, and static
power = 0.04. These operate on degrees, so multiplying or dividing them by
the new encoder resolution would change the intended controller response.
External gear feedback can change backlash, noise, and velocity measurement;
retuning may still be needed after direction and angle scale are verified.

## Robot verification

1. Establish the same physical starting zero used previously, then initialize
   with turret aiming disabled. Check the hub/port and observe raw ticks.
2. Move a small measured CCW angle with motor output disabled. Verify positive
   angle and the expected tick change (e.g. about 697.8 ticks for 10 degrees).
   Adjust direction or the measured gear ratio before enabling PID.
3. Check a small manual movement away from each limit: positive power must
   increase turret angle. Check the existing recenter control at physical zero.
4. Enable aiming at a nearby reachable target. TeleOp (including odo turret
   test) and autonomous telemetry show raw ticks, actual angle, target angle,
   PID error in degrees, and commanded motor power. Start with small errors;
   verify error decreases, settling, holding, and existing limit behavior.
5. If it oscillates, reduce P and then adjust D as needed. Verify the
   static-power taper near zero error before changing friction compensation.

Offline checks cannot verify wiring, sign, physical zero, backlash, hard-stop
clearance, or final PID tuning.
