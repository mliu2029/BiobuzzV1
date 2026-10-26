# BioBuzz robot code

The structure follows DecodeV3, adapted to NextFTC v2 0.2.1 and Pedro 3.

## Current hardware and shooter

`Robot` currently registers the drivetrain, intake, and shooter. Transfer, turret,
and camera initialization remain disabled; their controls below describe the full
robot configuration. The intake motor's hardware name is `intake`. Controller 1
Circle starts it at +1 and Square reverses it at -1; INIT and STOP set its power to
zero. In `Velocity Tuning`, `runIntake` runs the intake at -0.7 when true
and stops it when false; the transfer blocker remains disabled.

Configure one shooter motor named `flywheel`, with its encoder connected, and a
second motor named `rollers` in the Robot Controller hardware configuration. The
flywheel uses `RUN_WITHOUT_ENCODER` with software velocity PID in raw encoder
ticks/second and defaults to `REVERSE`; if needed, change its direction in
`Shooter.initializeMotors()` and confirm its measured velocity is positive in the
shooting direction. The rollers are initialized at zero power but not controlled yet.

Controller 1 D-pad up starts the flywheel; down stops it. Right/left changes the
target by 25 ticks/s. `Velocity Tuning` exposes P/I/D/F and `targetVelocity` in
Panels and reports the flywheel's speed and power. The initial target is 1375 ticks/s.
`Shooter.isReady()` requires the flywheel to reach at least target minus 100 ticks/s.
Invalid encoder readings or gains, STOP, and a zero/invalid target stop the flywheel.

- `subsystems/`: `Intake`, `Transfer`, `Shooter`, `Drivetrain`, `Turret`, and `Limelight`.
  Each exposes an `INSTANCE`, a private constructor, and named command fields or methods.
- `opmodes/`: TeleOps and autos. Autos declare poses and path fields, construct the
  paths in `buildPaths()`, and schedule the command sequence returned by `auto()`.
- `util/Calculations`: turret targets, alliance selection, and aiming math.
- `util/TurretConfig`: live turret calibration and PID settings in Panels.
- `tests/VelocityPidTuning`: the Panels shooter tuner, registered as `Velocity Tuning`.
- `Robot`: registers the subsystem instances with NextFTC and initializes/stops hardware.
- `util/OpModeStorage`: the autonomous end pose, defaulting to `(0, 0, 0)`.

## Framework lifecycle

NextFTC v2 uses `NextOpMode`, `@NextTeleop` / `@NextAutonomous`, and Ivy commands.
Its constructors initialize hardware; `start()` schedules the routine; `periodic()`
updates telemetry/follower; `end()` stops outputs. Subsystem `periodic()` methods,
the scheduler, PS4 triggers, and NextMotor outputs are advanced by NextFTC.

These are the equivalents of DecodeV3's constructor, `onInit()`,
`onStartButtonPressed()`, `onUpdate()`, and `onStop()` patterns. Do not mix in
DecodeV3's older `NextFTCOpMode`, `SubsystemComponent`, or `CommandManager` imports.

Shooter and turret PID loops run in their subsystems' `periodic()` methods.
Controller bindings live in `opmodes/TeleOp` and `BioBuzzTurretTeleop`.
Hardware is looked up during OpMode initialization, not while classes are scanned.
Every initialization stops outputs and resets the shooter control state. Turret
hardware and camera initialization are opt-in through the turret TeleOps.

## Subsystem commands

```java
Intake.INSTANCE.runIntake.schedule();
Intake.INSTANCE.runIntakeReverse.schedule();
Intake.INSTANCE.stopIntake.schedule();
Shooter.INSTANCE.runFlywheel.schedule(); // Uses the current target velocity.
Shooter.INSTANCE.runFlywheel(1375).schedule();
Shooter.INSTANCE.stopFlywheel.schedule();
Transfer.INSTANCE.openBlocker.schedule();
Transfer.INSTANCE.closeBlocker.schedule();
Turret.INSTANCE.aimTurret.schedule(); // Initialize the turret with an alliance first.
Turret.INSTANCE.stopTurret.schedule();
```

Creating a command does not execute it; schedule it or include it in a group.
The instantaneous commands leave their output/setpoint active until another command
changes it. Each command owns its subsystem. Driving reads joystick suppliers
continuously and stops when interrupted. TeleOp bindings close the blocker when
both L1 and R1 are released.

Transfer is a positional servo blocker with hardware name `transfer`. Set
`Transfer.OPEN_POSITION` and `Transfer.BLOCKED_POSITION` in Panels (range 0 to 1).
Both default to 0.5 as uncalibrated placeholders, so opening and closing currently
command the same position. INIT and STOP command the blocked position. Copy the
calibrated values into `subsystems/Transfer.java` to keep them after a restart.

## Autonomous format

See `opmodes/autonomous/RedTestAuto`, `RedTestReturnToShootAuto`, and `RedShootAndParkAuto`.
They preserve the existing poses, curve controls, heading interpolation, path order,
500 ms shooter wait, and 5000 ms feed period. The intake continues through the paths,
while the blocker closes after feeding. The existing `red shoot twice` sequence
still has its single opening feed period; this reorganization does not add a shot.

A sequence uses the same command logic as DecodeV3 with Ivy's group functions:

```java
private Command auto() {
    return sequential(
            Shooter.INSTANCE.runFlywheel,
            Commands.waitMs(500),
            Intake.INSTANCE.runIntake,
            Transfer.INSTANCE.openBlocker,
            Commands.waitMs(5000),
            Transfer.INSTANCE.closeBlocker,
            Drivetrain.INSTANCE.followPath(runToPark),
            robot.stop()
    );
}
```

Autos update the follower once per loop. NextFTC handles the scheduler, software
shooter PID, and motor event loop. `end()` saves `follower.pose()` into
`OpModeStorage.autonomousEndPose` and immediately stops the robot. TeleOp restores
that pose at START, then updates the follower before scheduling driving/aiming.
The saved pose lasts until the Robot Controller process restarts.

## PS4 controls

| Controller | Control | Action |
| --- | --- | --- |
| 1 | Right stick Y / X | Forward/backward / strafe |
| 1 | Left stick X | Turn |
| 1 | Circle | Start intake at +1 |
| 1 | Square | Reverse intake at -1 |
| 1 | D-pad up / down | Enable / stop shooter; down takes priority |
| 1 | D-pad right / left | Increase / decrease shooter target velocity by 25 ticks/s |
| 1 | R1 (hold) | Open transfer blocker; release to close |
| 1 | L1 (hold) | Close transfer blocker; L1 takes priority |
| 2, turret TeleOps | Circle | Stop turret aiming |
| 2, turret TeleOps | D-pad up | Resume automatic turret aiming |

The SDK calls PS4 L1/R1 `left_bumper` / `right_bumper`.
The Main, red turret teleop, and blue turret teleop use full-speed driving.

## Turret aiming

For camera setup and beginner-friendly aiming instructions, start with the
[Limelight tuning README](LIMELIGHT_README.md).

Select `red turret teleop` or `blue turret teleop` for your alliance. Aiming starts
at PLAY. Controller 2 Circle stops it; D-pad up resumes it. The Limelight is mounted
on the turret. Hardware names are `turret` for the motor and `limelight` for the camera.

The turret is assumed to be at the robot center represented by Pedro's pose.
`util/Calculations` finds the angle from that position to the hive and subtracts
the robot heading:

```java
double angleDegrees = Math.toDegrees(
        Math.atan2(target.y - pose.y(), target.x - pose.x()) - pose.heading()
);
```

Zero degrees is robot forward; positive angles turn counterclockwise. The code
wraps the angle and selects an equivalent angle inside the configured rotation
limits. `subsystems/Turret.periodic()` runs PID to reach and hold that angle,
including errors inside the alignment tolerance.

The selected alliance determines the hive. Pedro Y below `halfwayY` selects close;
Y at or above it selects far. The default halfway value is 70.75 inches.

| Target | Field position (inches) | AprilTag IDs |
| --- | --- | --- |
| Red close | (58, 59) | 34–37 |
| Red far | (58, 85) | 30–33 |
| Blue close | (84, 59) | 38–41 |
| Blue far | (84, 85) | 42–45 |

When `useVision` is enabled, fresh tags from the selected hive and pipeline correct
the odometry angle using their horizontal error. Each tag has a configurable desired
`tx`; accepted errors are averaged. Repeated reads of a frame hold its setpoint.
Missing, stale, or incorrect tags fall back to odometry.

Before using the turret:

1. Place it physically forward before INIT. Initialization resets its encoder to zero.
2. Set signed `ticksPerDegree` and measured `minDegrees` / `maxDegrees` in
   `util/TurretConfig`. The current scale is 1.51146 and needs physical verification;
   both limits default to zero, preventing movement until calibrated.
3. Tune `p/i/d` and `staticPower`, starting with the 0.25 power cap. Gains default to
   zero. `toleranceDegrees` controls the aligned indicator; PID stays active within it.
4. Configure the Limelight pipeline, per-tag desired `tx` values, and correction sign
   before enabling `useVision`, which defaults to false.

Close/far selection follows robot position. Hive tipping is not detected. See
[the turret tuning guide](TURRET_AIMING.md) for the detailed calibration procedure.

## Shooter tuning

Shooter uses NextFTC `PIDController` and `SimpleFeedforward`, with the SDK motor
in `RUN_WITHOUT_ENCODER`. The hardware name is `launcher`. Targets and feedback are
encoder ticks/second; the default target is 1375, with readiness at target minus 100.
Motor power is clamped to [0, 1], and invalid inputs stop output.

The software gains `Shooter.DEFAULT_P/I/D/F` start at zero and need physical tuning.
Do not copy REV Hub PIDF coefficients into this software loop.

1. Select `Velocity Tuning`. Edit its p/i/d/f, targetVelocity, and runIntake in Panels.
2. Set `targetVelocity` in encoder ticks/second; zero stops the shooter. Changes apply
   each loop. `f` is velocity feedforward, measured in power per tick/second.
3. Enable `runIntake` to run intake at -0.7; disable it to stop. `runTransfer`
   currently has no hardware effect because the transfer blocker is disabled.
   `runIntake` resets to false at INIT, PLAY, and STOP.
4. Graph target/measured velocity in Panels. Displayed RPM assumes 28 ticks/revolution;
   confirm this for the installed shooter motor.
5. Copy the tuned gains into `Shooter.DEFAULT_P/I/D/F` and rebuild. Panels edits do not
   automatically update source files.

Panels is available on robot Wi-Fi at http://192.168.43.1:8001 on a Control Hub,
or http://192.168.49.1:8001 on a phone RC. See [the turret tuning guide](TURRET_AIMING.md) for turret
calibration, tag groups, and current aiming limitations.

## Pollen tracking (webcam)

`vision/PollenLocator` runs [Hive Vision](https://github.com/hivevisionftc/Hive-Vision)'s
webcam HSV pipeline (`BallDetectorPipeline` / `BallBlobPipeline`, copied unchanged except
for the package) and projects each ball onto the floor in Pedro field coordinates:

```java
PollenLocator pollen = new PollenLocator(hardwareMap);
List<Pose> seen = pollen.getPollen(follower.pose());     // yellow pollen, field poses
PathResult route = pollen.planPath(follower.pose());     // Marrow Weaver route, or null
follower.follow(PedroPathingConverter.toPath(route));
```

`planPath` sends the visible pollen to Marrow's Weaver, which orders them and avoids the
HIVE legs and any red/blue nectar it sees. Set the webcam name, mounting (forward/left
offset, height, downward pitch, yaw), and field layout in `vision/PollenVisionConfig`
(editable in Panels). The webcam is named `Webcam 1` by default.

`Pollen Tracker Test` (Tuning group) shows each pollen's x/y. Cross follows a Weaver path
through the visible pollen; Circle returns to manual driving. To check the camera mounting,
place a pollen at a measured spot, drive until it is in view, and adjust `cameraPitch` and
`cameraHeightIn` until the reported x/y matches. Positions use the pose at read time, so
hold the robot still while reading them.
