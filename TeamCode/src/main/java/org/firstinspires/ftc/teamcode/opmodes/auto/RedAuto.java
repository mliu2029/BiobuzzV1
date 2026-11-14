package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import dev.nextftc.robot.opmode.NextAutonomous;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.core.BaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipPlan;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipPlan.Hive;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipPlan.Mode;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipPlan.Pickup;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipRoutes;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipRoutes.Stop;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.util.HiveTipDetector;

@Configurable
@NextAutonomous(name = "Red Modular Auto", group = "Pedro")
public class RedAuto extends BaseAuto {
    public static volatile double collectionDwellMs = 800;
    private static final double STOP_AFTER_MS = 30_000;

    private final Robot robot;
    private final Prompter prompter;
    private final TwoTipRoutes routes = new TwoTipRoutes();
    private List<Pickup> pickups = new ArrayList<>();
    private TwoTipPlan plan;
    private Follower follower;
    private Stop location = Stop.CLOSE_SHOT;
    private Hive hive = Hive.CLOSE;
    private Hive startSide = Hive.CLOSE;
    private Hive parkSide = Hive.FAR;
    private Mode mode = Mode.SOLO;
    private HiveTipDetector tipDetector;
    private Pose shotPose;
    private double shotTurretDegrees;
    private boolean monitoring;
    private boolean aborted;
    private int pickupIndex;
    private double pickupDwell;
    private long startedNanos;
    private String status = "Complete Marrow Prompter before START";
    private String lastTip = "No shot yet";

    public RedAuto(Robot robot) {
        super(robot);
        this.robot = robot;
        // NextOpMode isn't an SDK OpMode; Marrow 1.2.5 supports these injected inputs.
        prompter = new Prompter(telemetry, gamepad1, gamepad2);
        prompter.prompt("Start side", new OptionPrompt<Hive>("Start side", Hive.CLOSE, Hive.FAR));
        prompter.prompt("Auto type", new OptionPrompt<Mode>("Auto type", Mode.SOLO, Mode.PARTNER_CLOSE, Mode.PARTNER_FAR));
        prompter.prompt("Park side", new OptionPrompt<Hive>("Park side", Hive.FAR, Hive.CLOSE));
        prompter.prompt("Pickups", new MultiOptionPrompt<Pickup>(
                "Optional pickups IN ORDER; DONE with none = shoot and park", false, true, 3,
                Pickup.GARDEN, Pickup.FAR_FLOWER, Pickup.SIDE_FLOWER));
        prompter.prompt("Retries per hive", new OptionPrompt<Integer>(
                "Extra attempts per hive after reloading", 1, 0, 2, 3));
        prompter.showSummary().onComplete(() -> {
            startSide = prompter.get("Start side");
            parkSide = prompter.get("Park side");
            mode = prompter.get("Auto type");
            pickups = new ArrayList<>(prompter.<List<Pickup>>getOrDefault("Pickups", Collections.emptyList()));
            plan = new TwoTipPlan(prompter.<Integer>get("Retries per hive"));
            location = TwoTipRoutes.start(startSide);
            hive = mode.firstHive(startSide);
            status = "Start " + startSide + "; " + mode + "; park " + parkSide;
        });
    }

    @Override
    public void disabledPeriodic() {
        prompter.run();
        if (prompter.isCompleted()) {
            telemetry.addData("Auto", status);
            Pose start = TwoTipRoutes.start(startSide).pose;
            telemetry.addData("Start pose (in / deg)", "%.2f / %.2f / %.0f",
                    start.x(), start.y(), Math.toDegrees(start.heading()));
            telemetry.addLine("Preloaded pollen; upright hives; Full 3D pipeline.");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        if (!prompter.isCompleted() || plan == null) {
            status = "Stopped: finish and confirm Prompter before START";
            robot.stopImmediately();
            return;
        }
        if (!Double.isFinite(collectionDwellMs) || collectionDwellMs < 0
                || collectionDwellMs > 3000) {
            status = "Stopped: invalid collection dwell";
            return;
        }
        startAuto(Alliance.RED, location.pose);
        pickupDwell = collectionDwellMs;
        follower = robot.Drivetrain().getFollower();
        startedNanos = System.nanoTime();
        sequential(
                commands.init(),
                Commands.lazy(() -> hive == startSide ? Command.NOOP : travel(TwoTipRoutes.shooting(hive))),
                Commands.lazy(() -> aborted ? Command.NOOP : shootAndCheck()),
                Commands.lazy(this::nextCycle),
                Commands.lazy(this::park)
        ).raceWith(Commands.waitMs(STOP_AFTER_MS)).then(
                Commands.instant(() -> {
                    monitoring = false;
                    robot.stopImmediately();
                    if (elapsedMs() >= STOP_AFTER_MS) status = "Stopped at time limit";
                    else if (!aborted) status = "Finished; " + plan.summary();
                })
        ).schedule();
    }

    /** Every reload consumes one configured pickup. Tip/attempt decisions are made at runtime. */
    private Command nextCycle() {
        if (aborted || elapsedMs() >= STOP_AFTER_MS || pickupIndex >= pickups.size()) return Command.NOOP;
        Hive next = plan.next(hive, mode);
        if (next == null) { status = "Hive attempts complete"; return Command.NOOP; }
        Pickup pickup = pickups.get(pickupIndex++);
        hive = next;
        return sequential(
                collect(pickup),
                Commands.lazy(() -> aborted || elapsedMs() >= STOP_AFTER_MS ? Command.NOOP
                        : sequential(
                                travel(TwoTipRoutes.shooting(hive)),
                                Commands.lazy(() -> aborted || elapsedMs() >= STOP_AFTER_MS
                                        ? Command.NOOP : shootAndCheck()),
                                Commands.lazy(this::nextCycle)))
        );
    }

    private Command collect(Pickup pickup) {
        return sequential(
                Commands.instant(() -> {
                    status = "Collecting " + pickup;
                    robot.Transfer().setFeeding(false);
                    robot.Turret().stopImmediately();
                    robot.Intake().restart();
                }),
                travel(TwoTipRoutes.pickup(pickup)),
                Commands.lazy(() -> aborted ? Command.NOOP : sequential(
                        Commands.waitMs(pickupDwell),
                        Commands.instant(() -> {
                            robot.Turret().useOdometryAim(follower::pose, follower::velocity);
                            robot.Turret().setOdometryVisionEnabled(true);
                        })))
        );
    }

    private Command travel(Stop destination) {
        return Commands.lazy(() -> {
            if (aborted) return Command.NOOP;
            List<Command> steps = new ArrayList<>();
            for (TwoTipRoutes.Leg leg : routes.between(location, destination)) {
                steps.add(Commands.lazy(() -> {
                    if (aborted) return Command.NOOP;
                    if (!at(leg.from)) return abort("Outside route start " + leg.from);
                    return sequential(
                            commands.followPath(leg.path(follower.pose()), leg.to.toString()),
                            Commands.instant(() -> {
                                if (at(leg.to)) location = leg.to;
                                else {
                                    aborted = true;
                                    status = "Path failed: " + leg.to;
                                    robot.stopImmediately();
                                }
                            })
                    );
                }));
            }
            return sequential(steps.toArray(new Command[0]));
        });
    }

    private Command abort(String reason) {
        return Commands.instant(() -> { aborted = true; status = reason; robot.stopImmediately(); });
    }

    private boolean at(Stop stop) {
        Pose actual = follower.pose();
        return TwoTipRoutes.distance(actual, stop.pose) <= 3
                && Math.abs(Calculations.normalize(Math.toDegrees(actual.heading()-stop.pose.heading()))) <= 10;
    }

    private Command park() {
        if (aborted) return Command.NOOP;
        status = "Parking";
        return sequential(
                Commands.instant(() -> {
                    robot.Transfer().setFeeding(false);
                    robot.Turret().stopImmediately();
                }),
                travel(TwoTipRoutes.park(parkSide))
        );
    }

    private Command shootAndCheck() {
        return sequential(
                Commands.instant(() -> { status = "Aiming at " + hive; robot.Intake().stopImmediately(); }),
                Commands.waitUntil(this::shotReady).raceWith(Commands.waitMs(2000)),
                Commands.conditional(this::shotReady, stationaryShot(), Commands.instant(() -> {
                    plan.record(hive, false);
                    status = "Shot skipped: aim/speed unavailable; " + robot.Turret().getVisionStatus();
                    lastTip = "UNKNOWN (no shot)";
                }))
        );
    }

    /** Hold the camera still while checking whether the watched tag disappears. */
    private Command stationaryShot() {
        Command feed = Commands.waitMs(1000)
                .setEnd(condition -> robot.Transfer().setFeeding(false))
                .until(() -> tipDetector.tipped() || !cameraStill())
                .with(Commands.instant(() -> {
                    status = "Shooting " + hive;
                    robot.Transfer().setFeeding(true);
                    robot.Intake().restart();
                }));
        return sequential(
                Commands.instant(() -> {
                    shotPose = follower.pose();
                    shotTurretDegrees = robot.Turret().getDegrees();
                    int tag = robot.Turret().getVisionTagId();
                    int pipeline = robot.Turret().getVisionPipeline();
                    robot.Turret().useMovingAim(() -> shotTurretDegrees, () -> 0);
                    tipDetector = new HiveTipDetector(tag, pipeline);
                    status = "Waiting for tag " + tag;
                }),
                Commands.waitMs(150),
                Commands.instant(() -> monitoring = true),
                Commands.waitUntil(() -> tipDetector.isArmed()).raceWith(Commands.waitMs(800)),
                Commands.instant(() -> {
                    // Arm only after seeing the watched tag before feeding starts.
                    if (!tipDetector.isArmed()) tipDetector.invalidate();
                }),
                Commands.conditional(() -> cameraStill() && flywheelReady(), feed, Command.NOOP),
                Commands.waitUntil(() -> tipDetector.tipped()).raceWith(Commands.waitMs(600)),
                Commands.instant(() -> {
                    monitoring = false;
                    boolean tipped = tipDetector.tipped();
                    lastTip = tipDetector.state().toString();
                    plan.record(hive, tipped);
                    status = hive + ": " + lastTip;
                    robot.Transfer().setFeeding(false);
                    robot.Intake().restart();
                    robot.Turret().useOdometryAim(follower::pose, follower::velocity);
                    robot.Turret().setOdometryVisionEnabled(true);
                })
        );
    }

    private boolean flywheelReady() {
        double target = robot.Shooter().getTargetVelocity();
        return robot.Shooter().isEnabled() && target > 0
                && Math.abs(robot.Shooter().getFlywheelVelocity()-target) <= 75;
    }

    private boolean shotReady() {
        HiveTarget expected = hive == Hive.CLOSE ? HiveTarget.RED_CLOSE : HiveTarget.RED_FAR;
        return flywheelReady() && robot.Turret().getTarget() == expected
                && expected.acceptsTag(robot.Turret().getVisionTagId())
                && robot.Turret().isAligned() && robot.Turret().isVisionAligned()
                && Math.abs(robot.Turret().getVelocityDegreesPerSecond()) <= 5
                && follower.velocity().linSpeed() <= 1
                && Math.abs(follower.velocity().omega) <= Math.toRadians(1);
    }

    private boolean cameraStill() {
        Pose pose = follower.pose();
        return TwoTipRoutes.distance(pose, shotPose) <= 0.5
                && Math.abs(Calculations.normalize(Math.toDegrees(pose.heading()-shotPose.heading()))) <= 0.5
                && Math.abs(robot.Turret().getDegrees()-shotTurretDegrees) <= 0.5;
    }

    private double elapsedMs() { return (System.nanoTime()-startedNanos) / 1e6; }

    @Override
    public void periodic() {
        if (monitoring) {
            if (!cameraStill()) tipDetector.invalidate();
            tipDetector.update(robot.Limelight().getLatestResult(), System.nanoTime());
        }
        telemetry.addData("Modular auto", status);
        telemetry.addData("Hive attempts", plan == null ? "Not configured" : plan.summary());
        telemetry.addData("Tip result", monitoring ? tipDetector.state() : lastTip);
        if (follower == null) telemetry.update();
        super.periodic();
    }

    @Override
    public void end() {
        monitoring = false;
        super.end();
    }
}
