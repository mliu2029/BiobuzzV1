# Red Modular Auto

Select **Red Modular Auto** on the Driver Station. This replaces the displayed
name **Red Solo (2 Tip)** and uses preloaded pollen and initially upright hives.
The implementation is in `RedAuto.java`.

During INIT, [Marrow Prompter](https://marrow.skeletonarmyftc.com/dev-tools/prompter)
asks for the start side, auto type, and park side. Every mode then asks for optional
pickups **in execution order**, followed by 0–3 extra attempts per hive (default
selection: 1). Use D-pad up/down to navigate and A/Cross to select
each pickup, select DONE, then confirm the summary. B goes back. Each pickup may
be selected once; leave unwanted pickups unselected. START before completing the
menu leaves the robot stopped.

| Option | Behavior |
| --- | --- |
| Start CLOSE | Start at (53.0036, 6.0788, 90°) |
| Start FAR | Start at (64.2742, 134.4415, 270°), reused from the existing far partner auto |
| Solo: both hives | Shoot the starting-side hive first; use the selected pickups and retry policy |
| Partner: close hive only | Shoot the close hive, using optional selected pickups for retries, then park |
| Partner: far hive only | Shoot the far hive, using optional selected pickups for retries, then park |
| Park CLOSE | Park at (12.71, 93.61, 0°) |
| Park FAR | Park at (12, 118.64, 0°) |

With no pickups selected, any mode shoots the preload once and parks. With pickups
selected, partner mode reloads and retries only its chosen hive until it tips,
the attempt limit is reached, the pickups are exhausted, or time runs out. It
never switches to the other hive. A partner hive opposite the starting side first
receives a direct trip carrying the preload to that hive's shooting pose.
Far-start solo and far-start/far-hive partner mode shoot the preload in place at
FAR_START. Later solo reloads return to FAR_SHOT at (58.878, 128.2045, 270°).
There is no added 20-second delay in these modular partner modes.

The usual selection is Garden → Flower (46.83, 137.89) → Flower (2.35, 46.08).
Choose DONE without selecting pickups for a preload-only auto in any mode. In the
simulation, set all three pickup slots to Omit for the same behavior. The flower labels are field object coordinates,
not robot-center coordinates:

| Pickup | Robot-center pickup pose (inches, degrees) |
| --- | --- |
| Garden | (6.469, 7.35, 270°) |
| Flower (46.83, 137.89) | (47.2381, 132.3215, 90°) |
| Flower (2.35, 46.08) | (11.495, 46.8662, 180°) |

These pickup poses and the original travel corridors come from Chrome's `redTwoTipSolo.pp`.
The two original far shooting endpoints are consolidated at `(58.878, 128.2045,
270°)`. Explicit quadratic and cubic
curves now cover each pickup from either shooting pose. Their clearance,
rotation, intake reach, and timing need field validation. Both park coordinates
are user-specified robot-center positions; their headings remain 0°.

## Scenario paths in Pedro Visualizer

`doc/auto-routes/` contains 29 individual route `.pp.json` projects, eight preload-only
partner examples (both starts × both target hives × both parks), and a ZIP bundle:

| Scenario | Routes |
| --- | --- |
| Close shooting pose ↔ each of the three pickups | 6 |
| Far shooting pose ↔ each of the three pickups | 6 |
| Far start → each of the three pickups | 3 |
| Either start → the opposite hive's shooting pose with preload | 2 |
| Both shooting poses, far start, or any pickup → either park | 12 |

Open a project with **File → Open from computer** in Pedro Visualizer. The
`close-shot-all-options.pp.json` and `far-shot-all-options.pp.json` review projects
show every pickup round trip from that shooting pose, then its park route. These
are route catalogs for review, not single-match autonomous sequences. The
original project is retained as `original-red-two-tip-reference.pp.json`.
The `partner-*.pp.json` examples include a labeled two-second illustrative
shooting dwell; actual readiness and tip timing are handled by the robot code.

Runtime route selection uses the authored origin/destination pair directly.
There is no shortest-path search through pickup or shooting poses. For example,
after a close-hive retry, the robot drives directly to the far-flower approach;
it does not revisit the used garden or stop at the far shooting pose first.
Only the destination pickup's approach/withdrawal waypoint is included. Parking
also goes directly from the current location, without visiting another hive.

`two-tip-route-catalog.json` records the same robot poses and control points as
the Java implementation. The inline simulation uses the same 29 route cases.

## Behavior

1. Aim at and shoot the starting-side hive using the preload (solo), or the
   explicitly selected partner hive.
2. With no pickups selected, go directly to the chosen park after the preload attempt.
3. If the tip is unconfirmed and attempts remain, collect the next selected pickup and return to the
   same shooting pose for a retry.
4. In solo mode, after a likely tip or the attempt limit, use the next pickup to reload
   and shoot the other hive. Partner mode parks instead. A hive that tipped is never intentionally shot again.
5. Repeat until the selected pickups or allowed hive attempts are exhausted,
   then park through the defined route corridors.

The turret is stopped during collection and parking. Aiming resumes after the
collection dwell for the loaded return to the selected shooting pose. Collection
completion is inferred from the timed intake step; this does not add a ball sensor.

Each attempted shooting cycle consumes one attempt, including a cycle skipped
because vision or flywheel readiness timed out. Retries require a remaining
selected pickup. Collection and shooting may continue throughout the full
30-second autonomous period; all mechanisms stop at 30 seconds. Parking starts
when pickups or hive attempts are exhausted and can be interrupted by this limit.
Arrival must be within 3 inches and 10° of each waypoint. A failed path stops the
auto instead of advancing to another leg or shooting from the wrong location.

## Tip detection

The existing Limelight camera and selected Full 3D AprilTag pipeline serve both
aiming and detection. The auto watches the actual tag used to aim at the intended
hive. No additional camera or pipeline switch is used. Detection reads tag IDs
only; it does not request a 3D pose or calculate rotation.

After aim and flywheel readiness, the turret holds its encoder angle. After a
150 ms settling period, one fresh sighting of the watched tag arms detection
before feeding. Once armed, 200 ms without a fresh sighting gives TAG_LOST and
counts as tipped. Seeing another tag does not reset the timer. The standalone
tip test uses the same detector.

Duplicate frames cannot refresh the timer. Wrong-pipeline, stale, invalid, and
missing results all count as no usable sighting. If the watched tag was never
seen before feeding, the result remains UNKNOWN. Chassis translation over
0.5 inch, heading change over 0.5°, or turret movement over 0.5° invalidates the
trial and stops feeding. Normal odometry plus vision aiming resumes after the
observation window.

`RedAuto.collectionDwellMs` (default 800 ms) is configurable before START and
captured for that run. Tag loss is the success rule: an obstruction or camera
dropout can also trigger it, so it does not verify the hive's physical angle.

## Code ownership

- `modular.TwoTipPlan`: solo/partner mode, first-hive selection, bounded retries, and target selection.
- `modular.TwoTipRoutes`: 29 explicit scenario routes, both starts, pickup approach/withdrawal legs,
  and both park destinations.
- `HiveTipDetector`: fresh watched-tag visibility and the 200 ms loss timer.
- `RedAuto`: Prompter, collect/travel/shoot modules, timing, and telemetry.

The autonomous-to-TeleOp pose handoff remains owned by `core.BaseAuto`.
