# Shooting on the move

Select **SOTM Red** or **SOTM Blue**. These use odometry for aiming. The existing
TeleOp Red, TeleOp Blue, odometry test, and autonomous routines retain their controls.

## Setup

Set these in `SotmConfig` (Panels or source) before using the mode:

- `launchAngleDegrees`: ball departure angle above horizontal for the fixed shooter geometry.
- `goalMinusReleaseHeightInches`: opening aim-point height minus ball release height.
- `startX`, `startY`, `startHeadingDegrees`: the actual starting field pose. Defaults are
  `(70.75, 70.75, 90 degrees)`. Center the turret before a standalone START. To use the
  last autonomous pose and turret angle, enable `useAutonomousEndPose` after running
  autonomous and leave the turret in its autonomous ending position.
- `shooterForwardInches`, `shooterLeftInches`: effective release-point offset from the
  odometry reference, in chassis coordinates. Zero assumes release at that reference.
- `releaseDelaySeconds`: feed/control delay to release. Zero assumes immediate release.

The launch-angle default is the CAD's nominal 70-degree hood exit tangent, pending
physical trajectory verification. Height difference deliberately starts unset. The automatic
flywheel target stays at zero until the geometry produces a valid solution. Math calculates
flight time from these dimensions; it cannot determine physical dimensions from odometry.
The shooter elevation is fixed. CAD provides a starting estimate of the departure angle;
verify the actual ball trajectory because wheel contact and compression can change it.
See [robot CAD measurements](ROBOT_GEOMETRY.md) for the release-height estimate,
Limelight mounting geometry, and the rotating-muzzle limitation. The goal height and
odometry reference must be established before copying release offsets into this model.

## Controls

| Control | Action |
| --- | --- |
| D1 D-pad Up | Enable automatic moving-shot flywheel speed |
| D1 D-pad Left / Right | Trim calculated speed by -25 / +25 ticks per second |
| D1 Right Bumper (hold) | Run the transfer while held |
| D2 D-pad Down | Stop the shooter |
| D2 D-pad Up / Right | Stop / resume turret aiming |
| D2 Square, then bumpers, then Square | Manual recenter, then confirm center and resume |

Drive, intake, ramp, and servo-free controls are inherited from the main TeleOp.
The alliance's close/far hive switches across `TurretConfig.halfwayY` with a four-inch
hysteresis band. `SOTM hive` reports the active target. Confirm the coordinates in
`HiveTarget` match the field and opening you intend to shoot into.

`SOTM shot readiness` reports READY with a valid, reachable solution, enabled automatic turret control, flywheel
speed within 75 ticks/s, turret error within `TurretConfig.toleranceDegrees`, and turret
rate error within 30 degrees/s, continuously for 100 ms. These and the range/speed/loop
limits are tunable. A delayed loop clears readiness. The driver controls when to feed,
including when the shooter is off.

## Model

The mode updates the follower once before calculating the shot each loop. Pedro's
`velocity()` supplies field-frame inches/s and angular radians/s; see the
[Pedro velocity reference](https://pedropathing.com/docs/pathing/reference/velocity).
The same pose, velocity, goal, and model snapshot produce both motor targets.

At predicted release, let `d` be the horizontal vector to the goal and `v` the release
point's inherited field velocity. An offset release point inherits `omega cross r`
in addition to chassis translation. For fixed elevation `theta` and height difference
`h`, the solver finds a descending impact time `t` satisfying:

```text
tan(theta) * |d - v*t| = h + g*t^2/2
horizontal launch velocity = d/t - v
```

The turret points along that launch velocity relative to the predicted chassis heading.
The required horizontal launch speed, divided by the stationary ballistic launch speed
at the same range, scales `Calculations.velocity(distance)`. With zero motion this
reproduces the existing distance regression. This assumes motor speed is locally
proportional to ball exit speed; it is an approximation, especially when compression,
slip, or battery behavior changes. Air drag and acceleration during release are omitted.
The release point is modeled as fixed to the chassis; a muzzle far from the turret axis
or substantial turret-induced muzzle velocity needs a fuller geometry model.

A short prediction supplies the turret target rate. The moving PID uses the difference
between desired and measured angular velocity for its D term. Tune
`TurretConfig.movingVelocityFeedforward` (power per degree/s, initially zero) if needed
to reduce tracking lag. Encoder travel limits still apply; unreachable angles stop aiming.

## Field validation

Verify the starting pose and axis signs, stationary shots, then lateral and toward/away
motion at low speed. Check rotation separately. Confirm that shot readiness drops when
you stop the turret, disable the flywheel, or move outside configured limits. Tune the speed
trim, tracking feedforward, tolerances, and delay from observed behavior before increasing
speed. Offline trajectory integration checks the model's mathematics; it does not verify
ball flight, the stationary regression, hardware calibration, or stale sensor data.
