# StarterBot migration to NextFTC v2

The robot code from StarterBot now lives in BiobuzzV1. The StarterBot project was
used as the source and was not edited.

## Robot code

- `Robot.java` implements `NextRobot` and declares the drivetrain, intake, and shooter.
- `mechanisms/` contains these three NextFTC mechanisms. Each has a default command
  that runs its driver controls while the OpMode is active.
- `StarterBotTeleop.java` shares initialization, shooter speed adjustments, telemetry,
  and stop behavior across the three drive profiles.
- The three original TeleOp classes now use `NextOpMode` and `@NextTeleop`, keeping
  their original Driver Station names and group.

Hardware lookups occur when an OpMode is initialized, rather than when NextFTC
discovers the robot. Each initialization resets the shared robot's control state.
The FTC SDK hardware objects remain inside the mechanisms so the existing REV hub
velocity loop, encoder units, and PIDF gains (40, 0, 0, 12.5) stay the same.

This uses NextFTC v2 0.2.1, already configured in the project. See the
[NextFTC lifecycle](https://nextftc.dev/robot/nextopmode/) and
[mechanism documentation](https://nextftc.dev/robot/mechanisms/).

## Driver Station profiles

| TeleOp | Drive power limit | Shooter adjustment per press |
| --- | --- | --- |
| mec drive teleop | 100% | 25 encoder ticks/second |
| 30% mec drive teleop | 30% | 50 encoder ticks/second |
| 50% mec drive teleop | 50% | 50 encoder ticks/second |

All profiles use only the intake motor. The two intake servos from the original
slower profiles have been removed.

## Hardware configuration

Keep the same device names and motor types in the robot configuration.

| Hardware name | Device | Direction |
| --- | --- | --- |
| leftFrontDrive | Motor | Forward |
| rightFrontDrive | Motor | Reverse |
| leftBackDrive | Motor | Reverse |
| rightBackDrive | Motor | Forward |
| intake | Motor | Default |
| launcher | Motor with encoder | Default |
| windmillServo | Continuous rotation servo | Reverse |

## Controls

| PS4 controller 1 control | Action |
| --- | --- |
| Right stick Y | Forward/backward |
| Right stick X | Strafe |
| Left stick X | Rotate |
| Circle | Run intake forward; it continues after release |
| Square | Run intake in reverse; it continues after release |
| D-pad up | Enable shooter |
| D-pad down | Disable shooter |
| D-pad right / left | Increase / decrease both shooter velocity thresholds once per press |
| R1 | Feed while held, only when shooter velocity exceeds the minimum |
| L1 | Reverse windmill while held, even with shooter disabled |
| STOP | Stop all drive motors, intake motor, launcher, and windmill |

Square overrides circle, D-pad down overrides up, and L1 overrides R1
when both corresponding controls are held. Intake latching matches the
source code; there is no separate intake-off button.

## Deploy

Sync Gradle, connect the robot, and run **TeamCode** for the first full install.
The migration adds Pedro dependencies, so an existing robot installation needs
this full update. Use **Sloth Load** for later TeamCode-only changes. Perform
another full install after adding or changing dependencies.

## Pedro tuning

The `pedro/` tuning sources are copied unchanged, with the original `revhub:3.0.1`
and `tuning:1.0.1` dependencies added to TeamCode. They continue to use Pedro's
own tuning registration. `pedro/Constants.java` still returns `null` from
`create(...)`, as in StarterBot; configure its drivetrain and localizer before
using follower-based procedures. No autonomous paths existed in the source.

## Verification

The debug APK and Sloth bundle build successfully:

```sh
./gradlew :TeamCode:assembleDebug :TeamCode:assembleSloth
```

Compare the migrated controls with the original source:

```sh
python3 tools/verify_starterbot_migration.py /path/to/StarterBot
```

This checks 12,168 control frames for the retained hardware across the three profiles, including readiness
thresholds, simultaneous buttons, motor directions, PIDF gains, profile resets,
and stop outputs, and verifies that no intake servos are required. It uses simulated hardware and default-command execution;
real robot behavior and runtime registration still need to be checked on the robot.
