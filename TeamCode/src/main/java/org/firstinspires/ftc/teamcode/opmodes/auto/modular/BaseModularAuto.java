package org.firstinspires.ftc.teamcode.opmodes.auto.modular;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.opmodes.auto.core.BaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipPlan.Hive;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipPlan.Mode;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipPlan.Pickup;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.TwoTipRoutes.Stop;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.util.HiveTipDetector;

@Configurable
public abstract class BaseModularAuto extends BaseAuto {
    public static volatile double collectionDwellMs = 800;
    private static final double STOP_AFTER_MS = 30_000;
    private static final double MIN_TIME_FOR_CYCLE_MS = 7_500;

    protected final Alliance alliance;
    private final Prompter prompter;
    private final TwoTipRoutes routes = new TwoTipRoutes();
    private List<Pickup> pickups;
    private TwoTipPlan plan;
    private Follower follower;
    private Stop location;
    private Hive hive;
    private Hive startSide = Hive.CLOSE;
    private Hive parkSide = Hive.FAR;
    private Mode mode = Mode.SOLO;
    private HiveTipDetector tipDetector;
    private boolean monitoring;
    private boolean aborted;
    private int pickupIndex;
    private double pickupDwell;
    private long startedNanos;
    private String status = "Complete Marrow Prompter before START (or defaults will be used)";
    private String lastTip = "No shot yet";

    protected BaseModularAuto(Robot robot, Alliance alliance) {
        super(robot);
        this.alliance = Objects.requireNonNull(alliance, "Alliance cannot be null");

        // Initialize safe default configuration so START is always safe even if Prompter isn't completed.
        applyPlanConfiguration(
                Hive.CLOSE,
                Hive.FAR,
                Mode.SOLO,
                Arrays.asList(Pickup.GARDEN, Pickup.FAR_FLOWER, Pickup.SIDE_FLOWER),
                1
        );

        // Marrow Prompter handles interactive pre-match configuration
        prompter = new Prompter(telemetry, gamepad1, gamepad2);
        prompter.prompt("Start side", new OptionPrompt<>("Start side", Hive.CLOSE, Hive.FAR));
        prompter.prompt("Auto type", new OptionPrompt<>("Auto type", Mode.SOLO, Mode.PARTNER_CLOSE, Mode.PARTNER_FAR));
        prompter.prompt("Park side", new OptionPrompt<>("Park side", Hive.FAR, Hive.CLOSE));
        prompter.prompt("Pickups", new MultiOptionPrompt<>(
                "Optional pickups IN ORDER; DONE with none = shoot and park", false, true, 3,
                Pickup.GARDEN, Pickup.FAR_FLOWER, Pickup.SIDE_FLOWER));
        prompter.prompt("Retries per hive", new OptionPrompt<>(
                "Extra attempts per hive after reloading", 1, 0, 2, 3));
        prompter.showSummary().onComplete(() -> applyPlanConfiguration(
                prompter.get("Start side"),
                prompter.get("Park side"),
                prompter.get("Auto type"),
                prompter.getOrDefault("Pickups", Collections.emptyList()),
                prompter.get("Retries per hive")
        ));
    }

    private void applyPlanConfiguration(Hive startSide, Hive parkSide, Mode mode,
                                        List<Pickup> selectedPickups, int retries) {
        this.startSide = startSide;
        this.parkSide = parkSide;
        this.mode = mode;
        this.pickups = new ArrayList<>(selectedPickups);
        this.plan = new TwoTipPlan(retries);
        this.location = TwoTipRoutes.start(startSide);
        this.hive = mode.firstHive(startSide);
        this.status = "Configured: Start " + startSide + "; " + mode + "; park " + parkSide;
    }

    public HiveTarget getTargetHive(Hive hive) {
        if (alliance == Alliance.RED) {
            return hive == Hive.CLOSE ? HiveTarget.RED_CLOSE : HiveTarget.RED_FAR;
        } else {
            // 180-degree field rotational symmetry maps Red Close to Blue Far and Red Far to Blue Close
            return hive == Hive.CLOSE ? HiveTarget.BLUE_FAR : HiveTarget.BLUE_CLOSE;
        }
    }

    @Override
    public void disabledPeriodic() {
        prompter.run();
        if (prompter.isCompleted()) {
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Auto", status);
            Pose start = TwoTipRoutes.startPose(startSide, alliance);
            telemetry.addData("Start pose (in / deg)", "%.2f / %.2f / %.0f",
                    start.x(), start.y(), Math.toDegrees(start.heading()));
            telemetry.addLine("Preloaded pollen; upright hives; 2D tracking.");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        if (!prompter.isCompleted()) {
            status = "Warning: Prompter not confirmed; using defaults (" + status + ")";
        }
        pickupDwell = Math.max(0, Math.min(3000, Double.isFinite(collectionDwellMs) ? collectionDwellMs : 800));

        Pose startPose = TwoTipRoutes.startPose(startSide, alliance);
        startAuto(alliance, startPose);
        follower = robot.drivetrain.getFollower();
        startedNanos = System.nanoTime();
        sequential(
                initAuto(),
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
        if (aborted || pickupIndex >= pickups.size()) return Command.NOOP;
        if (elapsedMs() >= STOP_AFTER_MS - MIN_TIME_FOR_CYCLE_MS) {
            status = "Time budget low; parking";
            return Command.NOOP;
        }
        Hive next = plan.next(hive, mode);
        if (next == null) { status = "Hive attempts complete"; return Command.NOOP; }
        Pickup pickup = pickups.get(pickupIndex++);
        hive = next;
        return sequential(
                collect(pickup),
                Commands.lazy(() -> aborted || elapsedMs() >= STOP_AFTER_MS - MIN_TIME_FOR_CYCLE_MS ? Command.NOOP
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
                    robot.transfer.setFeeding(false);
                    robot.turret.stopImmediately();
                    robot.intake.restart();
                }),
                travel(TwoTipRoutes.pickup(pickup)),
                Commands.lazy(() -> aborted ? Command.NOOP : sequential(
                        Commands.waitMs(pickupDwell),
                        Commands.instant(() -> {
                            robot.turret.useOdometryAim(follower::pose, follower::velocity);
                            robot.turret.setOdometryVisionEnabled(true);
                        })))
        );
    }

    private Command travel(Stop destination) {
        return Commands.lazy(() -> aborted ? Command.NOOP
                : sequential(routes.between(location, destination).stream()
                        .map(this::travelLeg)
                        .toArray(Command[]::new)));
    }

    /** Adaptive route execution: tolerates minor odometry drift without hard-aborting. */
    private Command travelLeg(TwoTipRoutes.Leg leg) {
        return Commands.lazy(() -> {
            if (aborted) return Command.NOOP;
            Pose currentPose = follower.pose();
            // Generous start check - Pedro dynamically plans from currentPose to target
            if (!at(leg.from, 8.0, 30.0)) {
                status = "Adapting route start from " + leg.from;
            }
            return sequential(
                    followPath(leg.path(currentPose, alliance), leg.to.toString()),
                    Commands.instant(() -> {
                        if (at(leg.to, 3.5, 12.0)) {
                            location = leg.to;
                        } else if (at(leg.to, 6.5, 20.0)) {
                            // Arrived within extended tolerance; record location and proceed
                            location = leg.to;
                            status = "Arrived at " + leg.to + " (adapted)";
                        } else {
                            // Point off target; update location to target so next leg can continue
                            location = leg.to;
                            status = "Recovered arrival at " + leg.to;
                        }
                    })
            );
        });
    }

    private boolean at(Stop stop, double distanceTol, double headingTol) {
        Pose actual = follower.pose();
        Pose target = TwoTipRoutes.transformPose(stop.pose, alliance);
        return TwoTipRoutes.distance(actual, target) <= distanceTol
                && Math.abs(Calculations.normalize(Math.toDegrees(actual.heading() - target.heading()))) <= headingTol;
    }

    private boolean at(Stop stop) {
        return at(stop, 3.5, 12.0);
    }

    private Command park() {
        if (aborted) return Command.NOOP;
        status = "Parking (" + parkSide + ")";
        return sequential(
                Commands.instant(() -> {
                    robot.transfer.setFeeding(false);
                    robot.turret.stopImmediately();
                }),
                travel(TwoTipRoutes.park(parkSide))
        );
    }

    private Command shootAndCheck() {
        return sequential(
                Commands.instant(() -> {
                    status = "Aiming at " + hive;
                    HiveTarget expected = getTargetHive(hive);
                    Pose current = follower.pose();
                    double dist = Math.hypot(expected.x - current.x(), expected.y - current.y());
                    double speed = Double.isFinite(dist) && dist > 10 ? Calculations.velocity(dist) : Shooter.defaultRPM;
                    robot.shooter.setVelocity(speed);
                }),
                // Wait up to 1.5 seconds for vision alignment
                Commands.waitUntil(this::shotReady).raceWith(Commands.waitMs(1500)),
                Commands.conditional(this::shotReady,
                        stationaryShot(),
                        // Graceful fallback: attempt odometry-aligned shot if vision timed out
                        Commands.conditional(this::odometryShotReady,
                                fallbackOdometryShot(),
                                Commands.instant(() -> {
                                    plan.record(hive, false);
                                    status = "Shot skipped: aim/speed unavailable; " + robot.turret.getVisionStatus();
                                    lastTip = "UNKNOWN (no shot)";
                                })))
        );
    }

    /** Simplified stationary shot: smooth tracking without turret mode swap or chopped feed. */
    private Command stationaryShot() {
        return sequential(
                Commands.instant(() -> {
                    robot.drivetrain.stopImmediately();
                    status = "Shooting " + hive;
                    int tag = robot.turret.getVisionTagId();
                    int pipeline = robot.turret.getVisionPipeline();
                    tipDetector = new HiveTipDetector(tag, pipeline);
                    monitoring = true;
                    robot.transfer.setFeeding(true);
                    robot.intake.restart();
                }),
                Commands.waitMs(1000),
                Commands.instant(() -> {
                    robot.transfer.setFeeding(false);
                    monitoring = false;
                    boolean tipped = tipDetector != null && tipDetector.tipped();
                    lastTip = tipDetector == null ? "UNKNOWN" : tipDetector.state().toString();
                    plan.record(hive, tipped);
                    status = hive + ": " + lastTip;
                })
        );
    }

    /** Fallback shot using odometry when camera vision is occluded or unavailable. */
    private Command fallbackOdometryShot() {
        return sequential(
                Commands.instant(() -> {
                    status = "Shooting (odometry fallback) " + hive;
                    robot.transfer.setFeeding(true);
                    robot.intake.restart();
                }),
                Commands.waitMs(1000),
                Commands.instant(() -> {
                    robot.transfer.setFeeding(false);
                    plan.record(hive, false);
                    lastTip = "UNCONFIRMED (odometry shot)";
                    status = hive + ": " + lastTip;
                })
        );
    }

    private boolean flywheelReady() {
        return robot.shooter.isReady();
    }

    private boolean shotReady() {
        HiveTarget expected = getTargetHive(hive);
        return flywheelReady() && robot.turret.getTarget() == expected
                && expected.acceptsTag(robot.turret.getVisionTagId())
                && robot.turret.isAligned() && robot.turret.isVisionAligned()
                && Math.abs(robot.turret.getVelocityDegreesPerSecond()) <= 5
                && follower.velocity().linSpeed() <= 1
                && Math.abs(follower.velocity().omega) <= Math.toRadians(1);
    }

    private boolean odometryShotReady() {
        HiveTarget expected = getTargetHive(hive);
        return flywheelReady() && robot.turret.getTarget() == expected
                && robot.turret.isAligned()
                && Math.abs(robot.turret.getVelocityDegreesPerSecond()) <= 10
                && follower.velocity().linSpeed() <= 1.5
                && Math.abs(follower.velocity().omega) <= Math.toRadians(2);
    }

    private double elapsedMs() { return (System.nanoTime() - startedNanos) / 1e6; }

    @Override
    public void periodic() {
        if (monitoring && tipDetector != null) {
            tipDetector.update(robot.limelight.getLatestResult(), System.nanoTime());
        }
        telemetry.addData("Modular auto", status);
        telemetry.addData("Hive attempts", plan == null ? "Not configured" : plan.summary());
        telemetry.addData("Tip result", monitoring && tipDetector != null ? tipDetector.state() : lastTip);
        if (follower == null) telemetry.update();
        super.periodic();
    }

    @Override
    public void end() {
        monitoring = false;
        super.end();
    }
}
