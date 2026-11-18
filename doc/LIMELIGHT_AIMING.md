# Simple 2D Limelight turret aiming

The turret uses one AprilTag pipeline and raw per-tag horizontal angles. It
tracks one usable alliance tag at a time and corrects its horizontal displacement
from the opening using odometry distance. Limelight overrides odometry whenever
a usable tag is available, including close/far hive selection. Odometry remains
the fallback when no tag is usable. There is no camera POI, 3D pose processing,
pipeline scanning, or SOTM in this path.

## Camera setup

Pipeline 0 uses AprilTag Classic 36h11, 82.55 mm tags, 640x480 at 90 fps,
detector downscale 1, and the existing calibration/exposure/gain settings.
Full 3D Targeting is **No** and the camera POI Forward/Right/Up offsets are all
zero. The ID filter includes 30 through 45. Robot code selects only alliance
hive tags. The robot uses each tag's raw angle rather than the camera's primary
result, so multiple visible hives do not share an angle accidentally.

The original pipeline is saved in [the camera backup](limelight/before-poi-2026-11-17.json).
A reusable 2D profile is [hive-2d.vpr](limelight/hive-2d.vpr). The camera remains
on firmware 2025.1; this setup does not require per-tag map POIs or a firmware
flash. The older field map is not used for this 2D correction.

## Distance-aware horizontal correction

The four tag centers, viewed from the front in ascending ID order, are
-6.5, -2.75, +2.75, +6.5 inches from the opening center. The correction is:

```text
openingErrorDegrees = rawTagTxDegrees
                    + degrees(atan2(-tagOffsetInches, rangeInches))
                    + TurretConfig.visionOffsetDegrees
```

Range is odometry distance to the observed hive. If pose/range is unavailable or
less than 12 inches, it uses the existing 50-inch fallback estimate. Accurate
range improves centering; the fallback is an approximation. A 6.5-inch offset
requires about 7.4 degrees at 50 inches and 3.7 degrees at 100 inches. A fixed
angle or pixel correction cannot account for that distance change.

This simple geometry approximates the camera-to-opening correction. Camera
pitch/roll, shallow viewing angles, turret position, and field tolerances can
leave residual error. It is not a full perspective projection. Check several
distances and view angles before judging shot accuracy.

## Tracking and control

A usable tag stays locked even if another tag becomes larger. When it is lost
or unusable, the largest usable alliance tag takes over. Each new camera frame
updates one encoder setpoint. Repeated polls hold that setpoint; timestamps use
the camera time with Control Hub receipt time as a fallback. Stale/wrong-pipeline
frames and unreachable targets fall back to odometry.

The camera update deadband is 1 degree. Native NextFTC position PID, existing
friction compensation, travel limits, and manual recenter remain. Alignment requires
both encoder and camera error within `toleranceDegrees` when vision is active.
Gains were not retuned remotely.

After 200 ms of stationary robot/turret motion, a fresh usable camera observation
can calibrate the turret's software encoder zero. Aiming, alignment, travel checks,
and handoff use that corrected angle. The motor PID target is converted back to
raw encoder units, and the held camera target is rebased when calibration changes
so the two corrections do not stack. Raw encoder ticks remain available for diagnostics.

In `odo turret test`, **D2 D-pad Up** toggles learning; **D1 Circle** only starts
the intake. Learning OFF holds the last calibrated zero. The `useVision` setting
separately controls camera aiming. Tag loss, pose resets, and switching aim modes
retain the zero and re-arm learning. Manual recenter establishes a new zero.

## On-robot calibration

1. Center the turret physically, verify encoder direction/zero, then view a hive
   from near and far shooting distances. Confirm the odometry range is sensible.
2. Cover three tags at a time and check that all four aim at the same opening.
   A common left/right bias can be trimmed with `visionOffsetDegrees`. A bias
   that changes with tag, distance, or viewing angle needs geometry/range checks.
3. Check tag handoff and loss-to-odometry fallback. Then tune quiet stationary
   PID settling and verify actual shots. Source tests and builds do not establish
   physical aiming accuracy.

## Retained CAD reference

Only the horizontal tag spacing above is used by current aiming code. The
following historical CAD measurements retain the provenance of the opening
geometry; vertical/depth values are not used in the 2D runtime correction.

The offsets use FTC SDK 12.0's `AprilTagGameDatabase.getBioBuzzCluster()`, checked
against the supplied `am-5865-red1- Red Cell (Audience).step` and
[FIRST's production artwork](https://ftc-resources.firstinspires.org/ftc/field/apriltag-art),
page 2. The STEP models the sticker as one strip, without printed tag geometry;
the artwork supplies the individual centers and IDs.

The tag centers are at X = -6.5, -2.75, +2.75, +6.5 inches in ascending ID order.
The artwork places the row 2.75 inches above the bottom edge of the 17-by-5-inch
sticker. Combined with the STEP placement, the row is 7.1875 inches from the
front rim plane in the sticker plane. This differs from the SDK's 7.1874-inch value
by 0.0001 inch (0.00254 mm).

The transformed STEP measurements, in assembly coordinates converted from
meters to inches, are:

- Sticker center X: -1.006935570272.
- Printed tag row Y: +5.877474447189.
- Front rim planar face Y: -1.310025552811 (face entity #51154).
- Their Y difference: 7.187500000000.
- Outward sticker surface Z: -5.870638668815.

These are model dimensions, not measurements of an assembled field. Source STEP
SHA-256: `08aa041d70cdc2bfc341c6de18176bb1a250823c25020ff0043f6354f7d338fe`.

The aim point is now the **area centroid of the main opening cutout in the
front rim plane**, measured directly from the supplied STEP using OpenCascade.
For face #51154, the largest of its 31 inner boundary loops is the main opening;
the other inner loops are small mounting cutouts. Filling that closed main
boundary as a planar face gives the following coordinates in mm and area in mm²:

| Measurement | CAD result |
| --- | ---: |
| Opening centroid X | -25.576163484914 |
| Opening centroid Y | -33.274649041412 |
| Opening centroid Z | -2.775667457085 |
| Main opening area | 142174.621906750 |
| Sticker bounding-box center X | -25.576163484914 |
| Printed tag-row Y | 149.287850958588 |
| Outward sticker plane Z | -149.114222187901 |

The imported units were checked against the sticker's 431.8 by 127 mm
(17 by 5 inch) bounding dimensions. The printed tag row is located from the
production artwork, not the sticker face's area centroid: its mounting cutouts
make those centers slightly different.

Subtracting the tag-row center from the opening centroid gives
`(0, -182.562500, +146.338554731)` mm, or
`(0, -7.187500, +5.761360422)` inches. The setup values below round depth to **5.76136 inches**.
This replaces the SDK's 5.622-inch depth with a CAD-derived value, an increase of
0.139360422 inches (3.539755 mm). The target is the opening's area centroid,
rather than its bounding-box center, the rim material's centroid, or the
centroid of the whole hive solid.

![CAD opening centroid compared with the previous SDK point](hive-opening-center.svg)

The CAD target is a design dimension. It does not validate camera pose accuracy,
assembled-field tolerances, launcher alignment, or ball flight. This model
independently covers the red audience cell; the other cells retain the same
SDK cluster layout. The supplied odometry field coordinates are unchanged.

In Limelight target coordinates (+X right, +Y down), the offsets from the red
audience tags to the retained aim point are:

| Tag | X (in) | Y (in) | Z (in) |
| --- | ---: | ---: | ---: |
| 34 | +6.5 | -7.1875 | +5.76136 |
| 35 | +2.75 | -7.1875 | +5.76136 |
| 36 | -2.75 | -7.1875 | +5.76136 |
| 37 | -6.5 | -7.1875 | +5.76136 |

Other cells use the same SDK cluster geometry with their respective IDs; the
supplied STEP independently covers only the red audience cell. The tag rotation
transforms the offset into camera coordinates before calculating the correction.


References: [Limelight AprilTag tracking](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags).
