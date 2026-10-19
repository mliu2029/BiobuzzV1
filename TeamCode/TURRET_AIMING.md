# BioBuzz turret aiming

`BioBuzz Red Turret Aim` and `BioBuzz Blue Turret Aim` add automatic turret aiming
to the current starter-bot TeleOp controls. Each TeleOp locks aiming to its alliance.
Existing OpModes do not initialize the turret hardware. DecodeV3 is a read-only
reference: its `atan2(targetY - robotY, targetX - robotX) - robotHeading`
calculation and odometry-to-Limelight switch are preserved, using this project's
NextFTC v2 and Pedro 3 APIs.

The three main turret files are `Turret` (hardware and tracking), `TurretConfig`
(tuning settings), and `TurretCalculations` (targets, alliances, and aiming math).
Import target types as `TurretCalculations.Alliance` and `TurretCalculations.HiveTarget`.

| Target | Pedro coordinates (inches) | AprilTag IDs |
| --- | --- | --- |
| Red close | (58, 59) | 34, 35, 36, 37 |
| Blue close | (84, 59) | 38, 39, 40, 41 |
| Red far | (58, 85) | 30, 31, 32, 33 |
| Blue far | (84, 85) | 42, 43, 44, 45 |

After START, aiming automatically selects the close target below `halfwayY` and
the far target at or above it, including when crossing back. `halfwayY = 70.75`
matches the existing autos' 141.5-inch field. Set it to 72 if your calibrated Pedro
coordinates instead use a 144-inch field. The mapping assumes low Y is the audience
side and high Y is opposite the audience; confirm that with your field origin.
Switching cells clears the old vision setpoint and PID history. Opposite-alliance
and other-cell tags cannot correct the selected target.

PS4 controller 2 Circle stops turret power; D-pad up resumes automatic aiming.
Shooter/feed controls remain independent; the angular `Aligned` flag does not
prove a shot will score or automatically authorize feeding.

## Hardware calibration

1. Configure the motor as `turret`, with its encoder connected. The optional camera
   is named `limelight`. Existing Pedro drivetrain/Pinpoint configuration is reused.
2. In Panels, set `TurretConfig.ticksPerDegree` to **signed ticks per turret degree**:
   motor encoder ticks per motor output revolution times external reduction / 360.
   Use negative values if a physical counterclockwise turret turn decreases counts.
   This depends on the exact goBILDA motor and gearing; DecodeV3's `-2.872` is not
   assumed valid. Encoder ticks are displayed during INIT for measurement.
   The planned goBILDA 1150 RPM Yellow Jacket has about 145.1 encoder ticks per motor
   output revolution, so `abs(ticksPerDegree) = 145.1 * externalReduction / 360`.
   External reduction means motor output revolutions per turret revolution; do not
   multiply by the motor's internal 5.2:1 gearbox a second time. Recalibrate if the
   motor changes. Alternatively, measure the encoder change over a known turret
   rotation and divide by that rotation in degrees.
3. Set measured `minDegrees` and `maxDegrees` around forward zero, allowing margin
   before cable/mechanical stops. Limits currently default to zero and must be set
   from measured travel, with margin inside the planned +/-180 degrees.
   `ticksPerDegree = 0` still prevents movement until calibrated. A target
   outside reachable travel stops the turret instead of reporting alignment at a stop.
4. Set `turretOffsetForwardInches` and `turretOffsetLeftInches` from the Pedro pose
   origin to the turret's rotation center: forward and left inches.
   Defaults assume the pivot is at that origin.
5. The turret must already be physically at forward zero before TeleOp INIT.
   `Turret.initialize(...)` automatically resets the motor encoder,
   then use `RUN_WITHOUT_ENCODER` for software PID control. Turret angle is the encoder
   reading divided by `ticksPerDegree`. There is no home-confirmation button or homing motion.
6. TeleOp restores `OpModeStorage.autonomousEndPose` at START, then calls
   `follower.update()` before scheduling driving and aiming. Every current autonomous
   saves `follower.pose()` in `stop()`. When no autonomous pose has been saved in the
   current Robot Controller process, the default is `(0, 0, 0)` (inches, radians).
   Pose storage survives OpMode changes, but not a Robot Controller process restart.
   The regular starter-bot TeleOps use the same handoff through `StarterBotTeleop`.
7. Begin with small position P and the 0.25 power cap, then tune the NextFTC software
   PID (`p/i/d`, power per degree and time derivatives) and `staticPower` on the robot.
   All gains default to zero. Confirm the encoder/power sign with small movements.
   While enabled, PID remains active even inside `toleranceDegrees`; the tolerance
   only controls the `Aligned` indicator. Static friction compensation is zero at
   exactly zero error and follows the error direction otherwise. Circle/STOP,
   invalid configuration, and unreachable targets still remove motor power.

## Vision correction

The Limelight mounting rotates with the turret. `useVision` remains false until
the pipeline and aim offsets are calibrated.

Configure an AprilTag pipeline and set `pipeline`. The cell tag groups above come
from section 9.9 of the BioBuzz manual. For each group, the four `*Tx` array entries
are the desired tag tx values in ascending tag-ID order when the shooter aims at
the opening. Calibrate them before enabling vision; the initial zeros do not account
for the tags' different positions. The controller averages the corrected errors of
the accepted visible tags. The default correction is negative CCW degrees per
positive tx; verify the sign physically.

Odometry acquires the target. A valid matching tag within `acquireDegrees` (10 by
default) switches to camera correction. Null/empty results, wrong IDs or pipelines,
and frames older than `maxFrameAgeMs` including reported processing latency fall
back to odometry. Repeated reads of a frame hold its angle rather than accumulating
the same correction every loop. This initial controller is not motion/latency
compensated; verify stationary aiming before testing robot movement.

Close/far selection uses robot Y, not the hive's state. There is no automatic
hive-tip detection, target height/hood control, shooter-speed model, or moving-hive
prediction. The hive pivot, tag-to-opening geometry, and meaning of close/far must
be established before adding that behavior. A fixed per-tag tx offset may need replacement
with a geometric transform if tipping or distance changes that relationship.

## Autonomous integration

After `robot.initialize(hardwareMap)`, initialize the turret with
`robot.turret.initialize(hardwareMap, () -> follower.pose(), Alliance.RED)` (or BLUE).
The alliance is required, and the turret must be physically forward because this
call resets the encoder. Then schedule
`robot.turret.aimAutomatically()`. `aimAt(HiveTarget.RED_CLOSE)` can instead select
a fixed target; a locked turret rejects targets from the other alliance.
For plain SDK OpModes call `robot.turret.update()` after
`follower.update()` every loop. NextFTC OpModes run it through the registered default
command. `robot.stopImmediately()` stops turret power and camera polling.

## API references

- [BioBuzz manual, sections 9.6 and 9.9](https://ftc-resources.firstinspires.org/ftc/game/cm-html)
- [goBILDA 1150 RPM motor specification](https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder/)
- [Limelight FTC programming](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
- [NextFTC software PID](https://nextftc.dev/control/)
- [NextMotor 0.2.1](https://kdoc.nextftc.dev/hardware/dev.nextftc.hardware.actuators/-next-motor/index.html)
- [Pedro 3 Follower](https://github.com/Pedro-Pathing/PedroPathing/blob/main/core/src/main/java/com/pedropathing/follower/Follower.java)
- [Passing position from Autonomous to TeleOp](https://pedropathing.com/docs/pathing/guide/teleop-usage#passing-your-position-from-autonomous-to-teleop)
