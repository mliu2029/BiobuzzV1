# NextFTC mechanism commands

All four robot mechanisms return Ivy Command objects, following
https://nextftc.dev/robot/mechanisms/. Creating a command does not apply its action;
schedule it or compose it with then()/with(). Each action requires its mechanism.
Drivetrain uses Pedro's follower for manual driving; intake and transfer use NextMotor.
Shooter uses NextFTC PIDController plus SimpleFeedforward to control DcMotorEx power.

Shooter's default command reads D-pad up/down when initialized with a gamepad.
Robot.initialize(hardwareMap) leaves that gamepad unset for autonomous and tuning;
Robot.initialize(hardwareMap, gamepad1) enables the supplied TeleOp controls.
NextOpMode runs the default commands and NextMotor updates automatically.
Other button bindings and drive speed scales live in StarterBotTeleop.

## PS4 controller bindings

| Controller | Control | Action |
| --- | --- | --- |
| 1 | Right stick Y / X | Forward/backward / strafe |
| 1 | Left stick X | Turn |
| 1 | Circle | Start intake |
| 1 | Square | Reverse intake |
| 1 | D-pad up / down | Enable / stop shooter |
| 1 | D-pad right / left | Increase / decrease shooter target velocity |
| 1 | R1 (hold) | Run transfer forward |
| 1 | L1 (hold) | Reverse transfer; takes priority over R1 |
| 2, turret TeleOps | Circle | Stop turret aiming |
| 2, turret TeleOps | D-pad up | Resume automatic turret aiming |

The SDK calls PS4 L1 and R1 `left_bumper` and `right_bumper`. Face-button bindings
use NextFTC's PlayStation names: Circle, Square, Cross, and Triangle.

The drivetrain creates its follower through Constants.create(hardwareMap), using
the same FL/FR/BL/BR motor directions and Pinpoint configuration as Pedro autonomous.
Each execution of the TeleOp drive command calls follower.manual() then follower.update().
Right stick controls forward/strafe; left-stick X controls turning, as before.
Lateral/turn signs are mapped to preserve the previous wheel outputs. Combined inputs
are normalized before applying the configured drive scale (1.0 in current TeleOps). Cancelling the drive
command or stopping the OpMode immediately stops and brakes Pedro's drivetrain.

## Autonomous example

This is a timed mechanism example, not a tuned path or competition routine.
Create a Java class in this package with this structure and choose your own times:

```java
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous(name = "Mechanism Example")
public class MechanismExample extends NextOpMode {
    private final Robot robot;
    private final Command routine;

    public MechanismExample(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        robot.initialize(hardwareMap);
        routine = robot.intake.run()
                .then(Commands.waitMs(1000),
                      robot.intake.stop(),
                      robot.shooter.runAtVelocity(1400),
                      Commands.waitMs(2000),
                      robot.transfer.on(),
                      Commands.waitMs(500),
                      robot.transfer.stop(),
                      robot.shooter.stopCommand());
    }

    @Override
    public void start() {
        routine.schedule();
    }

    @Override
    public void end() {
        robot.stopImmediately();
    }
}
```

The instantaneous power/setpoint commands finish immediately, leaving the requested
output active until a subsequent command changes it. Use an explicit stop after a
wait. drive(forward, strafe, rotate) works the same way; its inputs are robot-relative
normalized powers, not distances. The supplier overload of drive(), and transfer.run(),
are continuous commands that stop their outputs when cancelled.

For readiness-based feeding after tuning, use Commands.waitUntil(robot.shooter::isReady)
with a timeout you choose. A readiness wait has no built-in timeout.

## Shooter tuning

Select "Velocity Tuning" in the Tuning group. The shooter uses NextFTC
`PIDController` plus `SimpleFeedforward`, with `DcMotorEx` in `RUN_WITHOUT_ENCODER`
mode. The controller combines PID output and velocity feedforward, then clamps
motor power to [0, 1].

1. Edit `p`, `i`, `d`, and `f` in Panels Configurables. Their defaults are
   zero until tuned. The previous REV Hub gains use different units and must
   be retuned. `p`, `i`, and `d` control velocity error; `f` is motor power per
   tick/second of target velocity. With all gains at zero, the shooter stays stopped.
2. Set `targetVelocity` in encoder ticks/second; zero stops the shooter.
   Changes apply each loop. Non-finite gains stop output until valid gains return.
3. Enable `runTransfer` to run the intake at -0.7 and both transfer motors at 0.8.
   It resets to false at INIT, PLAY, and STOP.
4. Graph "Target Velocity (ticks/s)" and "Measured Velocity (ticks/s)" in Panels.
   RPM uses 28 encoder ticks per revolution; confirm that value for the motor.
5. Panels edits are temporary. Update `DEFAULT_P`, `DEFAULT_I`, `DEFAULT_D`, and
   `DEFAULT_F` in `Shooter.java` to retain tuned gains; the tuner uses those defaults.

Panels is available on the robot Wi-Fi at http://192.168.43.1:8001 on a Control Hub
(http://192.168.49.1:8001 on a phone RC).

Normal shooter control starts with a target of 1375 ticks/second. The readiness
threshold is 100 ticks/second below the target, with a minimum of zero.
`isReady()` also requires the shooter to be enabled, valid gains, and a positive target.
`stop()` stops immediately; use `stopCommand()` in command sequences.
The controller resets when the target, gains, or enabled state changes.
NextFTC calls `Shooter.periodic()` each loop; plain SDK autos call it after
`Scheduler.execute()` so velocity stays regulated throughout the routine.

TeleOp keeps circle/square latched intake direction, up/down shooter on/stop,
left/right target adjustment, held L1/R1 for transfer (L1 wins), and full-speed driving.
Use stopImmediately() only for initialization
and lifecycle cleanup; use stop() as a command during a routine.

The Pedro tuning procedures continue to use Pedro's own drivetrain/localizer APIs.
Robot mechanism wrappers do not replace Pedro's internal hardware abstractions.
