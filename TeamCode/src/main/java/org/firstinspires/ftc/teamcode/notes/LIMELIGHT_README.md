# Limelight tuning, step by step

This guide is for the BioBuzz turret with a Limelight mounted on the part that
rotates. If the camera stays still when the turret turns, the current aiming math
needs to change before following the powered aiming tests.

The camera finds AprilTags and measures their angles. The robot code uses those
angles to move the turret. First make the camera see tags reliably. Then teach the
robot what a correctly aimed shot looks like.

## 1. Know what you are adjusting

| Term | Plain meaning |
| --- | --- |
| AprilTag | The square black-and-white marker with an ID number |
| Pipeline | A saved camera preset; pipeline 0 is preset number 0 |
| Exposure | How long the camera collects light for each picture |
| Gain | Electronic brightness; more gain also adds grain |
| `tx` | A tag's horizontal angle in degrees |
| Desired `tx` | The tag angle you want to see when the shooter is aimed correctly |
| Odometry | The robot position and heading estimated by Pedro |
| PID | The controller that decides how much power to send to the turret motor |

The order is:

1. Get a live camera picture.
2. Make tag detection reliable.
3. Get the turret working with vision off.
4. Measure the desired angle for each tag.
5. Enable vision and test correction.
6. Repeat from different positions and while moving.

Keep the shooter and feeder stopped during the initial aiming tests. Turn motor
power off before repositioning the turret by hand, and do not force its gearbox.

## 2. Open the camera settings

1. Connect the Limelight to your laptop with a USB data cable.
2. Wait about 20 seconds for it to boot.
3. Open [the Limelight web interface](http://limelight.local:5801).
4. If it does not open, use Limelight Hardware Manager to scan for the camera and
   open it from there.
5. Check that the live picture changes when you move something in front of it.

At this stage, you only need the camera and a real target tag. The turret does not
need to move. Limelight saves camera configuration changes automatically.

See the [official Limelight 3A setup instructions](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a).

## 3. Set up pipeline 0

Select pipeline **0** in the camera interface. Use these starting settings; labels
may vary with the installed Limelight software.

| Setting | Starting choice |
| --- | --- |
| Pipeline type | AprilTag / Fiducial Markers |
| Resolution | 640 x 480 for this 2D aiming test |
| Tag family | The family matching the actual printed tags |
| Black level | 0 |
| Cropping | Full image to start |
| ID filter | Leave unrestricted while first checking detection |

Do not copy an FRC tag size or family just because it appears in an example. Match
your actual target. This turret uses individual tag angles, rather than Limelight
field-position estimates, for vision correction.

Point the camera at a tag. Look for a detection outline and the correct ID number.
If it cannot identify a stationary nearby tag, check the pipeline type, tag family,
lighting, and whether the entire tag is visible before adjusting robot code.

**Pass this step when:** the camera repeatedly reports the correct ID while the
camera and tag are stationary.

## 4. Adjust exposure and gain

A long exposure makes a brighter picture, but a moving tag can smear across the
image. A short exposure makes motion sharper, but the picture gets darker. Gain
can brighten that darker picture, at the cost of more grain.

1. Put the tag at a normal shooting distance under realistic lighting.
2. Lower exposure a little at a time while watching detection.
3. When detection becomes unreliable, raise exposure slightly.
4. Move the camera or target sideways at a realistic speed. Do not use the powered
   turret for this camera-only test yet.
5. If detection works while stopped but fails during motion, try shorter exposure
   and enough gain to keep the tag visible.
6. Repeat at the farthest shooting distance. A setting that works close up may
   lose small, distant tags.

Change one setting at a time. Record the last setting that worked. The goal is
reliable detection while moving, rather than a pretty camera picture.

If far tags are too small to detect, try a higher resolution or less detector
downscale, then recheck processing speed and frame age. These can improve range
but slow the pipeline. See [Limelight's AprilTag tuning guide](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags).

**Pass this step when:** the correct tags remain detected at near and far shooting
distances, both stationary and during representative motion.

## 5. Prepare the turret with vision off

Open [TurretConfig.java](../data/TurretConfig.java). At the time this guide was
written, the source contains:

```java
ticksPerDegree = 28.0 * (90.0 / 24.0) / 360.0; // About 0.291667
minDegrees = 0;
maxDegrees = 0;
p = 0;
i = 0;
d = 0;
staticPower = 0;
maxPower = 0.25;
useVision = false;
pipeline = 0;
```

These are source defaults, not proof of the current values on the robot. Check
Panels before testing. The encoder scale must be physically verified, including
its sign. The zero travel limits prevent aiming, and zero gains/static power do
not command movement even after the limits are entered.

Before automatic aiming:

1. With the motor off, line the turret up with robot forward.
2. Select `red turret teleop` or `blue turret teleop` for the correct alliance.
3. Press **INIT**. Initialization calls the current turret position zero; it does
   not physically move the turret to forward.
4. Verify signed encoder counts per degree using a measured physical rotation.
5. Enter measured travel limits inside the actual safe travel, allowing room for
   wires and stopping. The code requires `minDegrees < 0 < maxDegrees`.
6. Verify Pedro's displayed position and heading match the robot on the field.
7. Tune the turret's PID with `useVision = false` and a reduced power cap.

Follow the [full turret tuning guide](TURRET_AIMING.md) for those measurements and
PID tuning. Do not guess angle limits or copy gains from another robot.

**PLAY automatically enables aiming.** Controller 2 **Circle** stops turret
aiming; controller 2 **D-pad up** resumes it. Driver Station **STOP** ends the
OpMode. Circle does not stop the shooter or feeder.

TeleOp restores the saved autonomous pose at PLAY. A default pose or a pose saved
before you moved the robot by hand is not a measurement of its current position.

**Pass this step when:** with vision off, the turret moves in the correct direction,
aims toward the selected opening, and holds without shaking or hitting its limits.

## 6. Understand the angle you will record

For a normally oriented camera, a tag left of the image center has negative `tx`,
a centered tag has about zero, and a tag right of center has positive `tx`.
Verify the convention on the installed camera.

But the tag may be beside the opening, and the camera may be beside the shooter.
Making the tag angle zero may aim at the wrong place.

Example: suppose the shooter is correctly aimed, but tag 34 reads **-4 degrees**.
Its desired angle should be **-4**, so the robot tries to return it to that reading.
That is an illustration, not a measured value for this robot.

Each tag needs its own desired angle. Otherwise, hiding one tag and revealing
another can change where the turret aims.

## 7. Display individual tag angles

The current turret TeleOp shows aim mode and turret angles, but does not display
individual tag angles or frame age. Those readings are needed for this measurement.

For a temporary diagnostic, add these imports to
[BioBuzzTurretTeleop.java](../opmodes/teleop/BioBuzzTurretTeleop.java):

```java


```

Add this block inside its `periodic()` method, before `super.periodic()`:

```java
robot.getLimelight().selectPipeline(TurretConfig.pipeline);
LLResult result = robot.getLimelight().getLatestResult();
if (result == null) {
    telemetry.addLine("LL: no camera result");
} else {
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

This is an optional code example, not an installed feature. Build and deploy after
adding it. It selects the configured pipeline even when automatic aiming is off.
Wait until `LL pipeline` matches the requested pipeline before recording values.

The block runs after PLAY. Because PLAY enables aiming, keep gains and static power
at zero for a stationary measurement session and confirm the turret is stopped.
Save any existing tuned values before temporarily changing them. Do not reposition
a powered, holding motor by hand.

Use **each tag's own reading** from `getTargetXDegrees()`. The single overall
`result.getTx()` value does not provide separate measurements for all four tags.

## 8. Save the correctly aimed readings

1. Place the robot at a representative shooting location.
2. Keep automatic aiming off and point the shooter accurately at the opening.
3. Hold the robot and turret still while taking readings.
4. Record several fresh readings for each tag and use a representative stable value.
5. Enter them into the matching array in `TurretConfig.java`, in this exact order:

| Array | First value | Second | Third | Fourth |
| --- | --- | --- | --- | --- |
| `redCloseTx` | Tag 34 | Tag 35 | Tag 36 | Tag 37 |
| `blueCloseTx` | Tag 38 | Tag 39 | Tag 40 | Tag 41 |
| `redFarTx` | Tag 30 | Tag 31 | Tag 32 | Tag 33 |
| `blueFarTx` | Tag 42 | Tag 43 | Tag 44 | Tag 45 |

For example, if the four red-close tags read -6, -2, 2, and 6 degrees when the
shooter is correctly aimed, the array would be:

```java
public static double[] redCloseTx = {-6, -2, 2, 6};
```

**Do not copy those example numbers.** Use measurements from your robot. If you
cannot see every tag at once, collect the missing readings while preserving the
same robot position, turret angle, and target state. Record which entries are
still unmeasured; zero is an actual desired angle, not an ignored entry.

Fixed angular offsets may change with distance or viewing angle, especially when
the camera, tag, and opening are separated. Validate from several positions. If
the correct readings change substantially, the aiming model needs more geometry;
increasing PID gains will not fix it. Hive tipping is not automatically detected.

## 9. Turn vision on and check the direction

1. Restore the tuned turret gains if you temporarily set them to zero.
2. Start with a reduced motor-power cap and the turret only a few degrees off aim.
3. Set `pipeline = 0` and `useVision = true` in Panels or source, matching the
   camera preset you configured.
4. Resume aiming with controller 2 D-pad up.
5. Watch `Aim mode` and the physical turret.

| Aim mode | Meaning |
| --- | --- |
| `OFF` | Aiming has been stopped |
| `CALIBRATION REQUIRED` | Turret settings failed validation |
| `ODOMETRY` | It is using Pedro aiming without accepted camera correction |
| `LIMELIGHT` | It accepted camera correction |
| `TARGET OUT OF RANGE / INVALID POSE` | The Pedro target angle is invalid or unreachable |

The camera correction should reduce the difference between observed and desired
tag angles. If it moves farther away, stop aiming immediately.

If direction is correct with vision off but wrong only with vision on, check camera
orientation and `txToCcwSign`. It starts at **-1**; use **+1** only if the measured
camera convention requires it. Do not reverse the encoder calibration to fix a
vision-only direction problem.

## 10. If it sees a tag but stays in ODOMETRY

Check the `Limelight` telemetry during INIT and while running. `Unavailable
(check USB/config)` means the hardware map has no `Limelight3A` named `limelight`.
The TeleOp can still initialize; camera readings return null and aiming falls
back to odometry once the turret is calibrated. `Configured` only confirms the
hardware lookup succeeded, not that the camera is returning valid frames.

If the hardware list says `limelight: Digital Device`, remove that mistaken
digital-channel entry. Connect the camera by USB and scan the robot configuration
for the top-level Limelight device, name it `limelight`, and save and activate
the configuration. A digital-channel entry cannot provide camera data. Without
the null guard, this misconfiguration crashes INIT at `setPollRateHz(50)`.

The code intentionally ignores some readings. Check these in order:

1. `useVision` must be true, and the camera hardware name must be `limelight`.
2. The returned pipeline must match `TurretConfig.pipeline`.
3. The tag must belong to the selected target in the table above.
4. Each tag's observed angle minus its desired angle must be within
   `acquireDegrees`, currently **10 degrees**.
5. The calculated frame age must be within `maxFrameAgeMs`, currently **100 ms**.
   The code adds staleness, capture latency, and targeting latency.
6. The turret calibration and Pedro target must be valid before vision is evaluated.
   The final corrected angle must also fit inside the travel limits.

Read `Aiming at:` on the Driver Station. Red/blue comes from the selected TeleOp.
Within that alliance, Pedro Y below **70.75** selects close; Y at or above it selects
far. A visible tag from the other target will be ignored.

Do not immediately widen the 10-degree gate or increase the age limit. First check
which requirement is failing. A higher age limit accepts older pictures; it does
not make the camera faster. A 50 Hz polling setting is not a guarantee of 50 fresh
processed images per second.

## 11. Test the whole system

With the robot stationary:

1. Test a small aiming error from both directions.
2. Cover the tags. Mode should return to `ODOMETRY`.
3. Uncover them. Mode should return to `LIMELIGHT` without a large aim jump.
4. Hide different tags in the same group. The aim should stay consistent.
5. Repeat at near, far, left, and right shooting locations.

Then drive and turn slowly while aiming. Increase speed only after the slower
tests work. The code does not compensate for turret motion between image capture
and arrival, or calculate projectile lead while driving.

| Problem | Check first |
| --- | --- |
| No tags detected | Pipeline type, tag family, lighting, distance, full tag visibility |
| Tags disappear during motion | Exposure too long, lighting, motion blur |
| Tags disappear only far away | Tag size in the image, resolution, downscale, exposure |
| Tags visible but mode stays ODOMETRY | Enable setting, selected target, pipeline, angle gate, age |
| Turret moves away only with vision on | Camera orientation, desired angles, correction sign |
| Turret shakes with steady readings | Turret PID, friction compensation, backlash |
| Aim jumps when one tag disappears | Incorrect per-tag desired angles or changing geometry |
| Works at one distance but misses at another | Fixed-offset geometry, Pedro pose, target position |
| Aligned says true but shots miss | Shooter aim reference, speed, trajectory, motion, target state |

`Aligned` only checks angular error. It does not prove the turret has settled or
that a shot will score.

## 12. Save what worked

Camera settings save on the Limelight. Copy final robot settings from Panels into
`TurretConfig.java`; live edits do not automatically rewrite Java source. Rebuild,
deploy, restart, and confirm the saved settings still work. Export a backup of the
camera pipeline too.

| Item | Your tested value |
| --- | --- |
| Date, lighting, and shooting distances | |
| Camera mount and orientation | |
| Pipeline number and tag family | |
| Resolution, exposure, gain, downscale | |
| Signed ticks per degree and travel limits | |
| P / I / D / static power / power cap | |
| Red close desired angles, tags 34–37 | |
| Blue close desired angles, tags 38–41 | |
| Red far desired angles, tags 30–33 | |
| Blue far desired angles, tags 42–45 | |
| Correction sign, angle gate, maximum age | |
| Results while stationary and moving | |

These steps describe how to measure and test the system. A successful software
build alone does not establish camera accuracy, safe travel, or shot accuracy.

## References

- [Full BioBuzz turret tuning guide](TURRET_AIMING.md)
- [Limelight 3A setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a)
- [AprilTag pipeline tuning](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags)
- [Limelight FTC programming](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
