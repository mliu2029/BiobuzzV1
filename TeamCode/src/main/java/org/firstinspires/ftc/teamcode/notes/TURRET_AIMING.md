# BioBuzz turret tuning guide

Tune in this order: **encoder and limits → Pedro pose → odometry PID → Limelight → driving tests**.
Finish each stage before moving to the next. Change one setting at a time and record
what changed. This guide describes the current code; numerical tuning examples are
starting experiments, not measured gains for your robot.

## 1. What the code does

The turret is at the robot center, with the Limelight mounted on the rotating turret.
There are no turret position offsets. Pedro supplies robot X/Y in inches and heading
in radians. The aiming calculation is:

```java
double angleDegrees = Math.toDegrees(
        Math.atan2(target.y - pose.y(), target.x - pose.x()) - pose.heading()
);
```

Zero degrees means robot forward. Positive turret angles are counterclockwise when
viewed from above. The code wraps the angle and chooses an equivalent angle inside
the measured rotation limits. If none is reachable, it stops output.

The alliance is fixed when the turret initializes. The robot's Y position selects
the close or far hive, including when crossing back:

| Alliance | Robot Y | Target coordinates | Accepted AprilTag IDs |
| --- | --- | --- | --- |
| Red | Below 70.75 | (58, 59), close | 34, 35, 36, 37 |
| Red | At or above 70.75 | (58, 85), far | 30, 31, 32, 33 |
| Blue | Below 70.75 | (84, 59), close | 38, 39, 40, 41 |
| Blue | At or above 70.75 | (84, 85), far | 42, 43, 44, 45 |

These are this project's configured targets and tag groups. Confirm the field
origin and physical target locations against your field. `halfwayY` is configurable;
changing it does not rescale the target coordinates. The switch has no hysteresis,
so pose noise at the boundary can alternate the target and reset PID/vision history.

Odometry first points toward the selected hive. With vision enabled, accepted tags
supply the final angular correction. Shooter velocity and feeding remain independent.
Hive tipping, opening height, projectile flight time, and moving-target prediction
are not handled by this controller.

## 2. Files, controls, and telemetry

All paths below are relative to this guide:

| File | Purpose |
| --- | --- |
| [data/TurretConfig.java](../data/TurretConfig.java) | Panels settings and source defaults |
| [util/Calculations.java](../util/Calculations.java) | Targets, alliance selection, angles, and vision acceptance |
| [mechanisms/Turret.java](../mechanisms/Turret.java) | Encoder reset, continuous PID, and vision correction |
| [mechanisms/Limelight.java](../mechanisms/Limelight.java) | Camera polling and pipeline selection |
| [opmodes/teleop/BioBuzzTurretTeleop.java](../opmodes/teleop/BioBuzzTurretTeleop.java) | Turret controls and telemetry |
| [data/OpModeStorage.java](../data/OpModeStorage.java) | Starting pose passed from autonomous |

Select **`red turret teleop`** or **`blue turret teleop`** on the Driver Station.
Aiming enables automatically at PLAY. Controller 2 **Circle** disables turret output;
controller 2 **D-pad up** resumes aiming and resets PID/vision history. Driver Station
STOP ends the OpMode. These controls do not disable the shooter or feed controls.
Keep intake, shooter, and transfer stopped during the initial turret tests.

Before every INIT, physically align the turret with robot forward. INIT resets the
encoder at its current physical position. It does not move the turret to forward,
find a home switch, or recover a lost reference. Circle does not reset the encoder.

During INIT, telemetry shows the starting pose, alliance, and raw encoder ticks.
After PLAY, watch:

| Telemetry | Meaning |
| --- | --- |
| `Aiming at:` | Selected alliance's close/far target |
| `Aim mode` | OFF, calibration required, odometry, Limelight, or invalid/unreachable target |
| `Turret encoder ticks` | Raw motor encoder position |
| `Turret degrees` | Encoder ticks divided by signed `ticksPerDegree` |
| `Target degrees` | Final setpoint after odometry/vision and travel checks |
| `Aligned:` | Angular error is within `toleranceDegrees` |
| `Pose` | Pedro position and heading |

`Aligned` is an angle check, not a settled-time check, vision-confidence check, or
proof that a shot will score. A turret can pass through the tolerance while moving.
The current telemetry does not show motor power, raw per-tag tx, or frame age.
Section 9 includes an optional diagnostic snippet for the camera values.

## 3. Settings and current defaults

Open `TurretConfig` in Panels while connected to the robot. On a Control Hub, Panels
is normally at `http://192.168.43.1:8001`. If your network differs, use its configured
Panels address. Confirm values on the robot before PLAY.

| Setting | Source default | What it controls |
| --- | --- | --- |
| `ticksPerDegree` | 0 | Signed encoder counts per turret degree; must be nonzero |
| `minDegrees` / `maxDegrees` | 0 / 0 | Measured clockwise/counterclockwise travel; must straddle zero |
| `p` | 0 | Power per degree of position error |
| `i` | 0 | Power from accumulated error over time |
| `d` | 0 | Power from change in error over time |
| `staticPower` | 0 | Extra power in the error direction to overcome friction |
| `maxPower` | 0.25 | Absolute output cap, greater than 0 and no greater than 1 |
| `toleranceDegrees` | 2 | Alignment indicator tolerance; does not turn PID off |
| `halfwayY` | 70.75 | Close/far selection boundary, in Pedro inches |
| `useVision` | false | Enables camera correction |
| `pipeline` | 0 | Limelight pipeline index |
| `redCloseTx` | Four zeros | Desired tx values for tags 34–37 |
| `redFarTx` | Four zeros | Desired tx values for tags 30–33 |
| `blueCloseTx` | Four zeros | Desired tx values for tags 38–41 |
| `blueFarTx` | Four zeros | Desired tx values for tags 42–45 |
| `txToCcwSign` | -1 | Converts corrected camera tx into turret CCW degrees; use -1 or +1 |
| `acquireDegrees` | 10 | Maximum absolute corrected tx accepted for each tag |
| `maxFrameAgeMs` | 100 | Maximum accepted staleness plus capture and targeting latency |

The zero encoder scale and zero limits intentionally prevent movement. Valid
calibration alone does not produce motion with all gains and `staticPower` at zero.
Gains and `staticPower` must be nonnegative and finite. Source defaults are not a
record of live Panels edits: write the final values back into `TurretConfig.java`.

## 4. Measure signed encoder ticks per degree

Configure the motor in the robot configuration as **`turret`**, with its encoder
connected. Mark a repeatable forward-zero position on the chassis and turret.
Leave `useVision = false` and `p = i = d = staticPower = 0` during measurement.

1. Put the turret at the forward mark, then INIT a turret TeleOp. Confirm raw ticks
   are approximately zero.
2. If the mechanism can be backdriven without forcing its gearbox, rotate it through
   a measured angle while still in INIT and read the encoder change. Otherwise use
   a separate controlled motor/jog test; this TeleOp has no manual turret jog control.
3. Calculate:

   ```text
   ticksPerDegree = encoder change / physical turret angle change in degrees
   ```

4. Treat counterclockwise rotation as a positive physical angle and clockwise as
   negative. For example, -900 encoder ticks over +90 degrees gives **-10 ticks/degree**.
   A return move of +900 ticks over -90 degrees should also give -10.
5. Repeat in both directions using several measured angles. Use the slope across
   the measurements; disagreement by direction suggests backlash, slipping, or
   measurement error. Fix those before PID tuning.
6. Enter the signed value in Panels. Return physically forward before the next INIT.
   Verify the displayed degrees agree with a physical angle after PLAY, with output
   still disabled or gains at zero.

If calculating from hardware specifications instead:

```text
abs(ticksPerDegree) = counts per motor output revolution
                     × motor output revolutions per turret revolution / 360
```

### Your 1150 RPM motor and 24-tooth / 90-tooth gears

The motor's 24-tooth gear drives the turret's 90-tooth gear, giving an external
reduction of `90 / 24 = 3.75:1`. The motor output turns 3.75 times per turret turn.
The goBILDA 1150 RPM Yellow Jacket specifies approximately 145.1 encoder counts per
motor output revolution ([manufacturer specification](https://www.gobilda.com/content/spec_sheets/5203-2402-0005_spec_sheet.pdf)).

```text
counts per turret revolution = 145.1 × 3.75 = 544.125
abs(ticksPerDegree) = 544.125 / 360 ≈ 1.51146
```

Use `+1.51146` if physical counterclockwise turret rotation increases encoder counts,
or `-1.51146` if it decreases counts. The gear sizes determine the magnitude, but
the installed motor orientation and direction configuration determine the sign.
The source remains at zero until that sign is verified.

A measured 90-degree turret turn should change the encoder by about 136 counts;
180 degrees should change it by about 272 counts. One count is about 0.66 turret
degrees. Use these as checks against the physical measurements above.

The 145.1-count specification already includes the motor's internal gearbox; do not
multiply by 5.2 again. Recalibrate after a motor, gearing, or encoder-direction change.

A physical CCW move must display positive turret degrees. The signed scale also
sets the output direction in this code. If powered movement goes away from the
setpoint, stop and inspect the encoder scale/direction setup before increasing P.

## 5. Set rotation limits

Measure travel relative to the same forward mark, including the restriction from
wires. `minDegrees` is negative clockwise travel; `maxDegrees` is positive CCW travel.
Both sides must exist: the current validation requires `minDegrees < 0 < maxDegrees`.

Set software limits inside the actual usable travel, with enough room for stopping
and mechanical variation. If travel is measured as -180 to +180 degrees, -170/+170
would be an example of reserving margin, not a required value for your mechanism.
Do not enter +/-180 solely because that is the planned range.

The controller stops outward power when an encoder limit is reached. That is not
predictive braking; momentum can carry the turret farther, so margin and a sensible
power cap still matter. It can command movement back into the allowed range.

Test near each software limit after small-angle PID tuning. An unreachable target
should show `TARGET OUT OF RANGE / INVALID POSE` and remove power. A narrower travel
range creates directions the turret cannot aim at; rotate the chassis to make them
reachable. Do not enlarge limits beyond physical travel to clear that message.

## 6. Verify Pedro before tuning PID

A turret with perfect motor control will still aim incorrectly if Pedro's pose is
wrong. Keep `useVision = false` for this stage. Check the robot center's X/Y against
measured field locations and verify heading while turning the chassis.

TeleOp restores `OpModeStorage.autonomousEndPose` at PLAY. The default `(0, 0, 0)` is
only a fallback, not automatic localization. The current autos save their final
pose in `end()`. Storage survives OpMode changes until the Robot Controller process
restarts; moving the robot by hand between auto and TeleOp invalidates that pose.
See [Pedro's pose handoff guide](https://pedropathing.com/docs/pathing/guide/teleop-usage#passing-your-position-from-autonomous-to-teleop).

For a stationary test without an auto, you can temporarily set the initial pose in
`data/OpModeStorage.java` to a physically measured test pose, then deploy and restart
the Robot Controller so an old static value cannot override the new initial value.
There is no pose-setting control in `TurretConfig`. Restore the normal `(0, 0, 0)`
fallback after testing and keep autonomous handoff enabled.

Example red-close geometry, if the robot is physically at these coordinates:

| Robot pose | Expected turret angle |
| --- | --- |
| X=58, Y=9, heading=90 degrees | 0 degrees, forward |
| X=58, Y=9, heading=0 degrees | +90 degrees, CCW |
| X=58, Y=9, heading=180 degrees | -90 degrees, clockwise |

In a `new Pose(...)`, pass heading in radians, such as `Math.toRadians(90)`.
For blue close, X=84 and Y=9 with heading=90 degrees should also aim forward.

Check the target switches on both sides of `halfwayY`, and that red never selects a
blue target or vice versa. Avoid tuning exactly at the boundary. Aiming from the
exact target coordinate is undefined; the controller rejects that pose.

## 7. Tune odometry PID

### First powered direction test

1. Keep vision off, I/D/static power zero, and start with a reduced cap such as
   `maxPower = 0.10`. This is an experimental cap, not a guarantee that the turret moves.
2. Start from a small reachable error, roughly 5–10 degrees. Create it by a small
   chassis rotation with valid Pedro tracking, or by offsetting a freely backdrivable
   turret while aiming is OFF. Do not re-INIT away from the forward mark.
3. Choose a small P from the initial power you intend to test:

   ```text
   p = desired initial power / error in degrees
   Example: 0.03 power / 10 degrees = 0.003
   ```

4. Press PLAY, then Circle if you need time to check the setup. D-pad up resumes
   aiming. Confirm the absolute error `abs(targetDegrees - turretDegrees)` decreases.
5. If it increases, press Circle or STOP. Correct the direction problem first.
   If it does not move, distinguish friction from bad wiring/encoder/pose before
   raising power. At the cap, raising P cannot provide more output.

### P: make it approach the target

With I, D, and static compensation still zero, increase P in small steps, for
example 20–25% per trial. Repeat small turns in both directions, then test larger
reachable errors. Aim for prompt response without repeated oscillation.

If the turret repeatedly overshoots, reduce P before adding D. If it stops a little
short because power becomes too small to overcome friction, leave that for the
static compensation step. Do not change target coordinates to compensate for an
encoder, pose, or motor-control problem.

### D: reduce overshoot

Add D only after P produces predictable movement. The installed controller measures
time in seconds; D multiplies the error rate in degrees/second. As a scale example,
a coefficient of 0.0001 contributes 0.01 power at an error rate of 100 degrees/second.
This explains the units; it is not a required starting gain.

Increase D gradually and compare the same test movement. Stop increasing when it
settles cleanly. Buzzing, twitching, noisy output, or slower response can indicate
excessive D. Sudden setpoint changes, especially a hive switch or vision acquisition,
also affect this derivative because it is based on error.

### Static compensation: overcome small-error friction

After P/D are stable, increase `staticPower` from zero in small steps, such as 0.005
power at a time, and repeat small approaches from both directions. Keep the smallest
value that helps it finish moving without repeated hunting around the target.

This term follows the sign of position error, not measured velocity. It is zero at
exactly zero error and flips sign across the target. Too much makes the turret
chatter even when P/D are otherwise reasonable. Check mechanical friction/backlash
before trying to solve everything with extra power.

### I: usually leave it at zero

Use I only for a repeatable remaining error under a steady load after P/D/static
compensation and mechanics are satisfactory. The current turret code does not set
an integrator limit or perform output-saturation anti-windup. Integral can accumulate
while output is capped, causing delayed overshoot.

If needed, use a very small value, test long holds and reversals, and revert to zero
if it becomes less predictable. Circle followed by D-pad up clears the accumulated
PID state. Gain edits alone do not reset that state.

### Power cap, tolerance, and repeated tests

Increase `maxPower` only if response is stable and capped output is demonstrably
limiting tracking speed. The configured 0.25 is a cap to test, not a required final
value. Recheck overshoot and both travel ends after changing it.

Keep `toleranceDegrees = 2` initially. Tighten it only after the physical mechanism
can repeat that accuracy. The tolerance only affects `Aligned`; PID continues
correcting inside it. Increasing tolerance does not improve aim.

Test 5-, 10-, 20-, and larger reachable angle changes in both directions. Log peak
overshoot, time to remain inside tolerance, and repeatability. Test with the normal
payload and both a fresh and partly discharged battery before accepting the gains.

## 8. Configure the Limelight pipeline

Do this after odometry aiming works. Keep `useVision = false` while configuring and
measuring the desired tag angles.

1. Connect the Limelight to a laptop by USB and open its Hardware Manager or
   `http://limelight.local:5801`. Configure an AprilTag pipeline and record its index.
2. Select the correct tag family for the physical field tags. Check that the expected
   IDs are detected and that tags remain readable at the distances you use.
3. Tune exposure/gain for clear tags with limited blur under your field lighting.
   Keep camera mounting and crosshair settings fixed after calibration.
4. Put the same pipeline index in `TurretConfig.pipeline`. While vision is off, this
   code does not request pipeline changes, so select it manually for measurements.
5. Connect the camera to the Control Hub and configure its hardware name as
   `limelight`. The code starts polling at 50 Hz; polling is not a guarantee of 50
   new camera frames per second.

The connection and interface steps follow the official
[Limelight 3A quick-start](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a).
Pipeline options are described in [Limelight pipeline setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/pipelines).

This controller uses per-tag horizontal angles, not Limelight `botpose` or MegaTag
localization. It does not replace Pedro's pose with a camera pose. Do not configure
a fixed chassis-to-camera transform expecting it to describe the camera through
all turret rotations.

## 9. Calibrate each tag's desired tx

A tag can be beside the scoring opening. Its correct tx is therefore not necessarily
zero when the turret is aimed correctly. Record each tag separately so a different
visible subset does not change the intended aim.

1. Disable aiming with Circle and leave vision correction off. Aim the actual
   shooter/turret direction at the intended opening using a repeatable physical
   reference. The camera crosshair alone is not the scoring reference.
2. Keep robot, turret, and hive still. Read the horizontal angle for each visible
   tag from `FiducialResult.getTargetXDegrees()` and average several fresh samples.
3. Store those readings in ascending tag-ID order:

   ```text
   redCloseTx  = {tx34, tx35, tx36, tx37}
   redFarTx    = {tx30, tx31, tx32, tx33}
   blueCloseTx = {tx38, tx39, tx40, tx41}
   blueFarTx   = {tx42, tx43, tx44, tx45}
   ```

4. Measure missing tags from a setup where they are visible. Do not substitute a
   guessed zero. Keep all four entries in each array.
5. Repeat at near/far shooting distances and different approach angles. If one fixed
   tx value cannot represent correct aim across them, record the limitation. The
   present model needs a geometric or distance-dependent correction to solve that;
   stronger PID gains will not solve it. Repeat this comparison for tipped hive states.

For example, readings of -6, -2, 2, and 6 degrees when correctly aimed would produce
`redCloseTx = {-6, -2, 2, 6}`. Those numbers are illustrative, not field measurements.

The current TeleOp does not display raw tx. If needed, temporarily add these imports
to `BioBuzzTurretTeleop.java`:

```java



```

Then add this diagnostic block inside `periodic()`, before `super.periodic()` sends
telemetry. This is an optional example; it is not already installed by this guide.

```java
LLResult result = robot.getLimelight().getLatestResult();
if (result != null) {
    telemetry.addData("LL valid", result.isValid());
    telemetry.addData("LL pipeline", result.getPipelineIndex());
    telemetry.addData("LL age ms", result.getStaleness()
            + result.getCaptureLatency() + result.getTargetingLatency());
    if (result.isValid() && result.getFiducialResults() != null) {
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            telemetry.addData("Tag " + tag.getFiducialId() + " tx",
                    tag.getTargetXDegrees());
        }
    }
}
```

Record fresh samples from the matching pipeline. Use the individual tag values,
not the aggregate `result.getTx()`, for these arrays. The SDK result and per-tag
methods are documented in [Limelight FTC programming](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).

## 10. Enable vision and verify its direction

The code computes:

```text
correctedTx = observed tag tx - that tag's desired tx
vision angle = current turret angle + txToCcwSign × average accepted correctedTx
```

Start nearly aimed, with a small known angular error and a reduced power cap. Enable
`useVision`, then resume aiming. Default `txToCcwSign = -1` means a positive corrected
tx requests a clockwise turret correction. Confirm that the physical movement reduces
the corrected tx error. If it moves away, disable aiming, turn vision off, and verify
camera orientation and the sign. Use +1 only if the measured convention requires it.
Do not change encoder sign to fix a vision-only direction problem.

Each accepted tag must belong to the selected hive, match the requested pipeline,
have a finite corrected tx within `acquireDegrees`, and pass the frame-age check.
The code calculates age as staleness + capture latency + targeting latency. Keep the
100 ms default initially; use measured values before changing it. Increasing the
threshold accepts older information and does not repair a slow pipeline.

`acquireDegrees = 10` applies to each corrected tag error, not the distance between
the vision setpoint and odometry setpoint. If vision never engages, inspect IDs,
actual pipeline, raw/desired tx, and age before widening that gate.

Expected results:

- Near the selected hive, `Aim mode` becomes `LIMELIGHT` and error settles.
- Cover the tags: mode returns to `ODOMETRY` and tracks the same physical opening
  within the measured accuracy of both methods.
- Uncover them: vision returns without a large jump.
- Hide different tags in the same cluster: aim remains consistent.
- Show only the opposite alliance or the other cell: those tags are ignored.
- Stop camera data: stale results fall back to odometry.

Pipeline changes are asynchronous; checking the returned pipeline index prevents
using a result from the previous pipeline. See [Limelight pipeline management](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming#2-pipeline-management).
Repeated reads of a frame hold its angle; a frame's correction is not added every
loop. A large repeated jump between vision and odometry points to disagreeing pose,
target geometry, or tx calibration before it points to a PID problem.

## 11. Test while driving

After stationary tests pass, increase driving speed gradually:

1. Translate slowly while aiming at one hive with vision off. The turret should
   maintain its direction toward the field target as robot position changes.
2. Turn the chassis slowly in both directions. The turret should counter-rotate
   while respecting travel limits.
3. Repeat with vision on, including temporary tag occlusion.
4. Cross the halfway boundary in both directions. Check target selection and the
   physical turn to the new target. The switch can be a large setpoint change.
5. Repeat on both alliances, at representative distances and battery levels.
6. Run autonomous, stop, and start the correct turret TeleOp without moving the
   chassis between them. Confirm pose handoff and turret zero separately. TeleOp
   INIT always resets the turret encoder, even after an auto that moved it.

This code uses the current encoder angle with the latest received image; it does
not reconstruct the turret angle at image capture. It also has no chassis-motion
feedforward or projectile lead calculation. Errors that grow with driving/turning
speed may be latency or model limitations. Record the speed where performance
becomes unacceptable instead of raising gains indefinitely.

## 12. Troubleshooting

| Symptom | Check first |
| --- | --- |
| `CALIBRATION REQUIRED` | Nonzero finite scale, limits straddling zero, finite nonnegative gains/static/tolerance, and 0 < maxPower <= 1 |
| Valid calibration but no movement | All gains zero, friction, low cap, hardware wiring, or target already reached |
| Moves away from target with vision off | Signed encoder calibration, direction configuration, and displayed angle versus physical CCW movement |
| Vision alone moves the wrong way | `txToCcwSign`, camera orientation, and desired tx values |
| Same physical aiming error everywhere | Forward-zero reference, chassis heading, and scoring reference |
| Error changes with turret rotation | Encoder scale, slipping, backlash, or cable load |
| Error changes with field position | Pedro pose, target coordinates, or fixed-tx geometry limitations |
| Oscillates around the target | Excessive P/staticPower, insufficient/excessive D, mechanical play, or changing setpoints |
| Overshoots after a long capped move | Integral buildup; return I to zero and reset aiming |
| Stops short by a small amount | Stiction, backlash, small P, or insufficient static compensation |
| `TARGET OUT OF RANGE / INVALID POSE` | Reachability inside real limits, valid pose, and robot not at the target coordinate |
| Always `ODOMETRY` with tags visible | `useVision`, camera hardware name, IDs, pipeline, desired tx, corrected-error gate, and frame age |
| Jumps when one tag disappears | Incorrect per-tag desired tx or geometry changing with viewing angle |
| Target alternates near halfway | Robot Y noise at the boundary; no switching hysteresis exists |
| Correct before STOP, wrong after INIT | Turret was not physically forward when its encoder reset |
| Wrong after moving robot between OpModes | Stored auto pose no longer matches the physical robot |
| `Aligned` true but shots miss | Pose/geometry/zero, shooter speed, height, trajectory, hive state, or turret still moving |

## 13. Save the tune and run an acceptance test

Record the tested motor/gearing, signed scale, measured limits, gains, cap, camera
mount, pipeline, tag tx arrays, lighting, distances, and battery conditions. Copy the
chosen values into `data/TurretConfig.java`, deploy, restart the Robot Controller,
and confirm the values match the saved tune. Export a backup of the Limelight
pipeline from its web interface as well.

Use a record like this for each revision:

| Item | Recorded result |
| --- | --- |
| Motor, gearbox, external reduction | |
| ticksPerDegree; min/max degrees | |
| P / I / D / staticPower / maxPower | |
| toleranceDegrees; halfwayY | |
| Pipeline; sign; acquireDegrees; maxFrameAgeMs | |
| Four desired-tx arrays | |
| Test pose, distance, hive state, battery | |
| Worst overshoot; settling time; final error | |
| Fastest acceptable chassis motion | |

The tune is ready for use when the following repeat on both alliances:

- Forward zero and encoder scale agree with physical measurements after each INIT.
- Small and large reachable moves settle from both directions without hunting.
- Approaches near each travel end remain inside the usable physical range.
- Close/far selection changes correctly in both directions across halfway.
- Vision acquisition, partial occlusion, and loss produce predictable aim.
- Auto-to-TeleOp pose handoff matches the robot's actual position.
- Circle removes turret power; D-pad up resumes the selected alliance's aim.
- Stationary and moving trials meet the team's measured aiming-error requirement.

A source build checks code compatibility. These physical tests establish the tune.
