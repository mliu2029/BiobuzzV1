# NextFTC mechanism commands

All four robot mechanisms return Ivy Command objects, following
https://nextftc.dev/robot/mechanisms/. Creating a command does not apply its action;
schedule it or compose it with then()/with(). Each action requires its mechanism.
Drivetrain uses Pedro's follower for manual driving; intake and transfer use NextMotor.
Shooter uses the FTC SDK's DcMotorEx with the hub's built-in velocity PIDF controller.

Shooter's default command reads D-pad up/down when initialized with a gamepad.
Robot.initialize(hardwareMap) leaves that gamepad unset for autonomous and tuning;
Robot.initialize(hardwareMap, gamepad1) enables the supplied TeleOp controls.
NextOpMode runs the default commands and NextMotor updates automatically.
Other button bindings and drive speed scales live in StarterBotTeleop.

The drivetrain creates its follower through Constants.create(hardwareMap), using
the same FL/FR/BL/BR motor directions and Pinpoint configuration as Pedro autonomous.
Each execution of the TeleOp drive command calls follower.manual() then follower.update().
Right stick controls forward/strafe; left-stick X controls turning, as before.
Lateral/turn signs are mapped to preserve the previous wheel outputs. Combined inputs
are normalized before applying the 100%, 30%, or 50% drive scale. Cancelling the drive
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

FTC AutoTune v1.4.3 (core and FTC integration) is installed in TeamCode.
Perform a full Robot Controller APK install after this dependency change;
a Sloth-only upload cannot install the new library.

1. Select "Shooter AutoTune" in the Tuning group. It controls only `launcher`.
   Its settings are available in Panels Configurables as ShooterAutoTune;
   edit before PLAY. Defaults match the shooter target of 1400 ticks/second.
2. Keep the flywheel clear. PLAY starts a relay test that reverses power,
   followed by a feedforward sweep at 0.5, 0.75, and 1.0 motor power.
   Progress and candidate gains appear on Driver Station telemetry and Panels.
   A timeout stops the motor; adjust relayAmplitude/targetVelocity and rerun.
3. After successful tuning, hold gamepad1 A to test the classic ZN candidate;
   release A to stop. STOP always removes motor power.
4. Copy the chosen candidate's kP/kI/kD/kF into Shooter.P/I/D/F and rebuild.
   They start at zero because the previous REV Hub gains are incompatible.
   Until gains are entered, normal TeleOp/autonomous shooter output is zero.
   This version of PIDMaster tunes in ticks/second; RPM is display-only.
5. To check gains under feeding load, run "Velocity Tuning" and enter the
   candidate in p/i/d/f, then set targetVelocity in ticks/second. These are
   software gains, with motor power as output. targetVelocity=0 stops the
   shooter. runTransfer runs intake at -0.7 and both transfer motors at 0.8;
   it resets to false at INIT, PLAY, and STOP. Panels edits are not saved to source.

Panels is available on the robot Wi-Fi at http://192.168.43.1:8001 on a Control Hub
(http://192.168.49.1:8001 on a phone RC). Graph "Target Velocity (ticks/s)" and
"Measured Velocity (ticks/s)" in Velocity Tuning. RPM uses Shooter.TICKS_PER_REV,
currently 28 as in the previous tuner; confirm that value for your actual motor.
The AutoTune library's separate dashboard is optional; the shooter tuner uses Panels.

Shooter uses AutoTune's PIDFController with DcMotorEx in RUN_WITHOUT_ENCODER mode,
reads absolute encoder velocity in ticks/second, and writes power bounded to [0, 1].
Targets remain in ticks/second (default 1400, minimum 1300). Controller state resets
when the target/gains change or the shooter stops. Non-finite gains stop output
until valid values are restored. NextFTC calls Shooter.periodic() every loop;
plain SDK autonomous OpModes explicitly call it after Scheduler.execute().
Shooter.stop() stops immediately; use Shooter.stopCommand() in command sequences.

TeleOp keeps circle/square latched intake direction, up/down shooter on/stop,
left/right target adjustment, held bumpers for transfer (left wins), and the existing
100%, 30%, and 50% drive variants. Use stopImmediately() only for initialization
and lifecycle cleanup; use stop() as a command during a routine.

The Pedro tuning procedures continue to use Pedro's own drivetrain/localizer APIs.
Robot mechanism wrappers do not replace Pedro's internal hardware abstractions.
