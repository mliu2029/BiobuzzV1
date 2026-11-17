# Four-tag Limelight aiming

Turret auto aim now reads the camera's POI-adjusted `LLResult.getTx()`.
Limelight computes the tag-to-opening 3D transformation onboard. Robot code only
turns that angular error into a NextFTC position setpoint. It does not read 3D
poses or add a distance-based tag offset.

Camera setup has not been verified on the robot. `TurretConfig.limelightPoiConfigured`
defaults to `false`: vision reports that setup is required and odometry remains
available. Set it to `true` only after completing the camera setup below.

A usable Limelight POI overrides odometry, including close/far hive selection.
SDK 12.0 exposes the primary target's `tx` but no corresponding primary-tag ID.
Therefore, all fiducial results in a frame must belong to one alliance hive.
With mixed hives or opponent tags, the turret falls back to odometry. The visible
tag ID in telemetry is a retained reference tag, not a claim about which tag the
camera selected for its primary output. Limelight selects that primary tag;
robot code does not average or combine their raw angles.

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

## Camera setup

1. Back up the camera's existing pipelines and field map. Confirm that its firmware
   supports per-tag fmap POIs (introduced in Limelight OS 2026.1).
2. In the AprilTag pipeline selected by `TurretConfig.pipeline` (default 0),
   select 36h11, the correct calibration, and 82.55 mm / 3.25-inch tags.
   Enable the 3D processing required for POI targeting.
3. In the existing BioBuzz map, enter a separate opening POI for every hive tag.
   Preserve its existing tag transforms and field coordinates.
4. Enable **Use Map POI Offsets** (`use_fmap_poi`). Clear global UI and
   NetworkTables POI overrides: nonzero overrides take priority over map POIs.
5. Filter the pipeline to alliance hive IDs: red 30–37 or blue 38–45.
   If both alliances share one pipeline, code will reject opponent/mixed frames.
6. Verify each tag's POI marker lands at the same opening centroid in the map
   visualizer. Check the live POI overlay and `tx` while viewing from both sides
   and different ranges. Only then set `limelightPoiConfigured = true` in source
   (or temporarily in Panels).

The new fmap `poi` entry uses **[forward, left, down] in meters**, tag-relative.
This differs from the older target-space XYZ table above. Converting that
retained CAD offset as [Z, -X, Y] gives these starting values:

| Tag IDs | Forward (m) | Left (m) | Down (m) |
| --- | ---: | ---: | ---: |
| 30, 34, 38, 42 | +0.146338544 | -0.165100 | -0.1825625 |
| 31, 35, 39, 43 | +0.146338544 | -0.069850 | -0.1825625 |
| 32, 36, 40, 44 | +0.146338544 | +0.069850 | -0.1825625 |
| 33, 37, 41, 45 | +0.146338544 | +0.165100 | -0.1825625 |

These are CAD-derived setup values, not physically validated calibration.
Confirm the forward-axis direction and tag orientation in the installed firmware's
visualizer before enabling aim; do not copy the old XYZ tuple directly.
The red audience cell is the independently measured CAD source; the other cells
reuse the SDK cluster geometry. This table is an addition to the current map,
not a replacement map with invented tag poses.

## Robot behavior

The turret uses `result.getTx()` once for the opening. It never adds raw
`FiducialResult.getTargetXDegrees()`, an odometry distance offset, or another POI.
This prevents double correction. Camera crosshair/yaw must be aligned with the
launcher; camera pitch is not used as a robot-side transform for this feature.

Fresh, correct-pipeline POI frames inside `acquireDegrees` (10 degrees) override
odometry if the resulting turret angle is reachable. When the same frame is
polled repeatedly, its previous encoder setpoint is held. A fresh frame updates
the setpoint if its POI error exceeds `visionUpdateDegrees` (2 degrees).
Within that band, it holds the current angle for the fresh measurement.

No usable POI means odometry chooses the alliance close/far hive using robot Y.
Disabling vision also selects odometry. Native NextFTC position PID, encoder
travel limits, manual recenter, and the existing static-power ramp remain.
SOTM is not part of this aiming path.

A stationary POI observation can still teach `TurretAngleEstimator` the
odometry angle correction. Stillness is updated independently of which aiming
source wins. Manual recenter or a pose reset clears the correction.

## On-robot verification

Confirm that covering individual tags leaves the opening aim consistent, and
that losing all tags restores odometry. Check that repeated polls do not move
the target again, and that a visible far hive can override a close hive chosen
by odometry. Check turret encoder direction/zero and launcher alignment before
judging shots. Source tests and builds do not establish physical accuracy.

References:
[POI targeting](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d),
[per-tag POIs and override priority](https://docs.limelightvision.io/docs/docs-limelight/software-change-log),
[FTC result API](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).
