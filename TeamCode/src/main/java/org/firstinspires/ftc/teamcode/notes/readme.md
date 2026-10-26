# BioBuzz robot code

The structure follows Hazmat: a Robot owns the mechanisms, OpModes select commands,
and autonomous geometry is separate from routines. Driving uses Pedro 3 with
NextFTC v2 0.2.1 and Ivy commands.

## Current hardware and dual-motor shooter

`Robot` currently registers the drivetrain, intake, shooter, turret, and camera.
Select `red turret teleop` or `blue turret teleop` to initialize the `turret` motor
and optional `limelight` camera for that alliance. Point the turret physically
forward before INIT, which resets its encoder. Aiming starts at PLAY; controller 2
Circle stops it and D-pad up resumes it. Set measured travel limits and tuned gains
in `TurretConfig` before expecting motion; the current zero limits report
`CALIBRATION REQUIRED`, and vision correction remains off by default.
Transfer hardware and its controls remain disabled.
The intake motor's hardware name is `intake`. Controller 1
Circle starts it at +1 and Square reverses it at -1; INIT and STOP set its power to
zero. In `Velocity Tuning`, `runIntake` runs the intake at +1 when true
and stops it when false; the transfer blocker remains disabled.

Configure two matching shooter motors and connect both encoders, named `flywheel`
and `rollers` in the Robot Controller hardware configuration. Both rely on the
SDK's initial `RUN_WITHOUT_ENCODER` mode with NextMotor velocity control and its default encoder scale (1 radian per
encoder tick). Setpoints and velocity telemetry use radians/second at that scale;
these are not calibrated shaft radians/second. The custom tick unit and explicit
angle-per-count/cache settings have been removed.
Shooter commands set both velocity targets directly; NextMotor maintains them until
another command changes the target or stops the motors.
NextFTC's motor hook runs each NextMotor's PID and feedforward controller using
plain named motors: `new NextMotor("flywheel")` and `new NextMotor("rollers")`.
Shooter uses only NextMotor references for direction, feedback, and stopping.
STOP sets zero throttle and updates both motors immediately. It uses NextMotor's
default 0.01 power cache tolerance, which can suppress a zero write when the cached
output is already within that tolerance of zero.
Motor output follows standard SDK signed power limits (-1 to 1); the custom output
adapter has been removed. The target and gains keep their
existing numeric values because the default encoder feedback has the same numeric
scale as the previous tick-based controller. The existing F gain maps to
NextMotor's velocity feedforward kV; kS and kA remain zero.
They share one target velocity and one set of P/I/D/F gains. Each motor has its own
PID controller and encoder feedback so a slower motor receives its own correction.
The `flywheel` direction is `REVERSE` and `rollers` is `FORWARD`, as configured in
`Shooter.initialize()`. Confirm each measured velocity is positive in the
intended shooting direction.

Controller 1 D-pad up starts both motors; down stops both. Right/left changes their
shared target by 25 rad/s. `Velocity Tuning` exposes shared P/I/D/F and
`targetVelocity` in Panels and reports each motor's speed and power separately. RPM estimates have been removed
because the default encoder scale does not represent calibrated shaft rotation.
The initial target is 1375 rad/s; the current default gains are in
`mechanisms/Shooter.java` and still need validation on the robot.
Use `getFlywheelVelocity()` / `getRollerVelocity()` and `getFlywheelPower()` /
`getRollerPower()` to inspect each motor. Power telemetry reports NextMotor's cached
output command clipped to the SDK's [-1, 1] range. Autonomous
shooting uses the existing timed sequence; there is no readiness-gating API.
Invalid gains, STOP, and a zero/invalid target stop both motors. There is no separate
encoder-fault monitor. Adjusting the saved target while stopped keeps the motors
stopped. Verify directions, encoder readings, gains, and stopping on
the robot; software tests cannot establish physical speed matching.

- `mechanisms/`: `Intake`, `Transfer`, `Shooter`, `Drivetrain`, `Turret`, and `Limelight`.
  Each is constructed by `robot/Robot` and exposes named command fields or methods.
  OpModes use getters such as `robot.getShooter()`; mechanisms are not static singletons.
- `opmodes/teleop/`: selectable TeleOps and their shared base classes.
- `opmodes/calib/`: calibration and tuning OpModes.
- `opmodes/auto/paths/`: autonomous poses and path construction.
- `opmodes/auto/commands/`: reusable actions in `AutoCommands` and sequences in `Routines`.
- `opmodes/auto/runnables/`: autonomous OpMode initialization, scheduling, and telemetry.
- `data/`: alliance/target enums, turret configuration, and saved OpMode pose.
- `util/Calculations`: aiming math; target and alliance enums live in `data/`.
- `data/TurretConfig`: live turret calibration and PID settings in Panels.
- `opmodes/calib/VelocityPidTuning`: the Panels shooter tuner, registered as `Velocity Tuning`.
- `robot/Robot`: constructs mechanisms, registers them with NextFTC, and initializes/stops hardware.
- `data/OpModeStorage`: the autonomous end pose, defaulting to `(0, 0, 0)`.

## Framework lifecycle

NextFTC v2 uses `NextOpMode`, `@NextTeleop` / `@NextAutonomous`, and Ivy commands.
Its constructors initialize hardware; `start()` schedules the routine; `periodic()`
updates telemetry/follower; `end()` stops outputs. Subsystem `periodic()` methods,
the scheduler, PS4 triggers, and NextMotor outputs are advanced by NextFTC.

These are the equivalents of DecodeV3's constructor, `onInit()`,
`onStartButtonPressed()`, `onUpdate()`, and `onStop()` patterns. Do not mix in
DecodeV3's older `NextFTCOpMode`, `SubsystemComponent`, or `CommandManager` imports.

Shooter commands update the setpoints; NextFTC's motor hook runs both NextMotor
velocity controllers each loop. Shooter needs no `periodic()` method. The turret PID loop runs in its own
`periodic()` method.
Controller bindings live in `opmodes/teleop/TeleOp` and `BioBuzzTurretTeleop`.
Hardware is looked up during OpMode initialization, not while classes are scanned.
Every initialization stops outputs and resets the shooter control state. Turret
hardware and camera initialization are opt-in through the turret TeleOps.

## Subsystem commands

```java
robot.getIntake().runIntake.schedule();
robot.getIntake().runIntakeReverse.schedule();
robot.getIntake().stopIntake.schedule();
robot.getShooter().runFlywheel.schedule(); // Uses the current target velocity.
robot.getShooter().stopFlywheel.schedule();
robot.getTransfer().openBlocker.schedule();
robot.getTransfer().closeBlocker.schedule();
robot.getTurret().aimTurret.schedule(); // Initialize the turret with an alliance first.
robot.getTurret().stopTurret.schedule();
```

Creating a command does not execute it; schedule it or include it in a group.
The instantaneous commands leave their output/setpoint active until another command
changes it. Each command owns its mechanism. TeleOp starts driving with
`robot.startDrive(gamepad1)`; the drivetrain continuously reads the sticks, calls
Pedro `follower.manual(...)`, and updates localization. It stops when interrupted.
Transfer controls remain disabled.

Transfer is a positional servo blocker with hardware name `transfer`. Set
`Transfer.OPEN_POSITION` and `Transfer.BLOCKED_POSITION` in Panels (range 0 to 1).
Both default to 0.5 as uncalibrated placeholders, so opening and closing currently
command the same position. INIT and STOP command the blocked position. Copy the
calibrated values into `mechanisms/Transfer.java` to keep them after a restart.

## Autonomous format

See the OpModes in `opmodes/auto/runnables/`. Each selects a method from
`opmodes/auto/commands/Routines`; its geometry lives in `opmodes/auto/paths/`.
The autonomous registration annotations remain disabled.
They preserve the existing poses, curve controls, heading interpolation, path order,
500 ms shooter wait, and 5000 ms feed period. The intake continues through the paths,
while transfer hardware remains disabled. The existing `red shoot twice` sequence
still has its single opening feed period; this reorganization does not add a shot.

A routine composes shared actions and paths:

```java
public Command shootAndPark(ShootAndParkPaths paths) {
    return sequential(
            commands.shootPreload(),
            commands.followPath(paths.runToPark, "Run to Park"),
            commands.finish()
    );
}
```

`AutoCommands` receives the same Robot that NextFTC injected into the OpMode.
Its preload action starts the shooter, waits 500 ms, starts the intake, and waits
5000 ms. Transfer remains disabled. No helper constructs another Robot.

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
| 1 | D-pad right / left | Increase / decrease shooter target velocity by 25 rad/s |
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
limits. `mechanisms/Turret.periodic()` runs PID to reach and hold that angle,
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
   `data/TurretConfig`. The goBILDA bare motor with 24T driving 90T gives about
   0.291667 ticks per degree; verify its sign physically.
   Both limits default to zero, preventing movement until calibrated.
3. Tune `p/i/d` and `staticPower`, starting with the 0.25 power cap. Gains default to
   zero. `toleranceDegrees` controls the aligned indicator; PID stays active within it.
4. Configure the Limelight pipeline, per-tag desired `tx` values, and correction sign
   before enabling `useVision`, which defaults to false.

Close/far selection follows robot position. Hive tipping is not detected. See
[the turret tuning guide](TURRET_AIMING.md) for the detailed calibration procedure.

## Shooter tuning

Shooter uses two plain NextMotor instances for `flywheel` and `rollers`, with
both SDK motors in `RUN_WITHOUT_ENCODER`. NextMotor owns each velocity controller
and its gains; Shooter does not keep a second copy of those gains. Targets and
feedback use rad/s with NextMotor's default 1-radian-per-tick scale. The default
target is 1375.
Motor power uses the SDK's [-1, 1] limits; zero/invalid targets and invalid gains
stop output. Encoder feedback goes directly to NextMotor without a separate fault monitor.

The software gains are the current `Shooter.DEFAULT_P/I/D/F` values and need
validation on the robot.
Do not copy REV Hub PIDF coefficients into this software loop.

1. Select `Velocity Tuning`. Edit its p/i/d/f, targetVelocity, and runIntake in Panels.
2. Set `targetVelocity` in encoder rad/s; zero stops the shooter. Changes apply
   each loop. `f` is velocity feedforward, measured in power per rad/s at the default encoder scale.
3. Enable `runIntake` to run intake at -0.7; disable it to stop. `runTransfer`
   currently has no hardware effect because the transfer blocker is disabled.
   `runIntake` resets to false at INIT, PLAY, and STOP.
4. Graph target/measured velocity and each motor's power in Panels. The default
   encoder scale does not give calibrated shaft RPM.
5. Copy the tuned gains into `Shooter.DEFAULT_P/I/D/F` and rebuild. Panels edits do not
   automatically update source files.

Panels is available on robot Wi-Fi at http://192.168.43.1:8001 on a Control Hub,
or http://192.168.49.1:8001 on a phone RC. See [the turret tuning guide](TURRET_AIMING.md) for turret
calibration, tag groups, and current aiming limitations.
