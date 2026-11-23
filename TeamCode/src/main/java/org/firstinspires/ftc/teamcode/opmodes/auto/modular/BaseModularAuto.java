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
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain.PathResult;
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
    // Timed estimate for 4 balls on every shot; calibrate at 1925 roller RPM.
    public static volatile double feedMs = 1000;
    public static volatile double tipObservationMs = 500;
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
    private boolean shotUnavailable;
    private int pickupIndex;
    private double pickupDwell;
    private double feedDuration;
    private double tipObservationDuration;
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
            telemetry.addLine("Feed 4 balls on every shooting cycle (timed).");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        if (!prompter.isCompleted()) {
            status = "Warning: Prompter not confirmed; using defaults (" + status + ")";
        }
        pickupDwell = Math.max(0, Math.min(3000, Double.isFinite(collectionDwellMs) ? collectionDwellMs : 800));
        feedDuration = Math.max(0, Math.min(3000, Double.isFinite(feedMs) ? feedMs : 1000));
        tipObservationDuration = Math.max(0, Math.min(3000,
                Double.isFinite(tipObservationMs) ? tipObservationMs : 500));

        Pose startPose = TwoTipRoutes.startPose(startSide, alliance);
        startAuto(alliance, startPose);
        follower = robot.drivetrain.getFollower();
        shotUnavailable = false;
        startedNanos = System.nanoTime();
        sequential(
                initAuto(),
                Commands.instant(this::aimAtHive),
                Commands.lazy(() -> hive == startSide ? skip() : travel(TwoTipRoutes.shooting(hive))),
                Commands.lazy(() -> aborted ? skip() : shootAndCheck()),
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

    // Ivy 1.1.1's Command.NOOP never finishes; skipped sequence steps must complete.
    static Command skip() {
        return Commands.instant(() -> { });
    }

    /** Every reload consumes one configured pickup. Tip/attempt decisions are made at runtime. */
    private Command nextCycle() {
        if (aborted || shotUnavailable || pickupIndex >= pickups.size()) return skip();
        if (elapsedMs() >= STOP_AFTER_MS - MIN_TIME_FOR_CYCLE_MS) {
            status = "Time budget low; parking";
            return skip();
        }
        Hive next = plan.next(hive, mode);
        if (next == null) { status = "Hive attempts complete"; return skip(); }
        Pickup pickup = pickups.get(pickupIndex++);
        hive = next;
        return sequential(
                collect(pickup),
                Commands.lazy(() -> aborted || elapsedMs() >= STOP_AFTER_MS - MIN_TIME_FOR_CYCLE_MS ? skip()
                        : sequential(
                                travel(TwoTipRoutes.shooting(hive)),
                                Commands.lazy(() -> aborted || elapsedMs() >= STOP_AFTER_MS
                                        ? skip() : shootAndCheck()),
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
                Commands.lazy(() -> aborted ? skip() : sequential(
                        Commands.waitMs(pickupDwell),
                        Commands.instant(this::aimAtHive)))
        );
    }

    private void aimAtHive() {
        robot.turret.useOdometryAim(follower::pose, getTargetHive(hive));
        robot.turret.setOdometryVisionEnabled(true);
    }

    private Command travel(Stop destination) {
        return Commands.lazy(() -> aborted ? skip()
                : sequential(routes.between(location, destination).stream()
                        .map(this::travelLeg)
                        .toArray(Command[]::new)));
    }

    /** Pedro owns arrival checks; stop the routine if a leg fails. */
    private Command travelLeg(TwoTipRoutes.Leg leg) {
        return Commands.lazy(() -> {
            if (aborted) return skip();
            return sequential(
                    followPath(leg.path(follower.pose(), alliance), leg.to.toString()),
                    Commands.instant(() -> {
                        PathResult result = robot.drivetrain.getLastPathResult();
                        if (result != PathResult.COMPLETED) {
                            aborted = true;
                            monitoring = false;
                            status = "Stopped: " + leg.from + " -> " + leg.to + " (" + result + ")";
                            robot.stopImmediately();
                            return;
                        }
                        location = leg.to;
                        status = "Arrived at " + leg.to;
                    })
            );
        });
    }

    private Command park() {
        if (aborted) return skip();
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
                Commands.waitUntil(this::shotReady).raceWith(Commands.waitMs(1500)),
                Commands.conditional(this::shotReady, shoot(), Commands.instant(() -> {
                    shotUnavailable = true;
                    status = "Aim/speed unavailable; parking with current load";
                    lastTip = "UNKNOWN (no shot)";
                }))
        );
    }

    /** Feed four balls, then observe a tip when vision supplied a watched tag. */
    private Command shoot() {
        return sequential(
                Commands.instant(() -> {
                    robot.drivetrain.stopImmediately();
                    status = "Shooting " + hive + "; 4 balls (timed)";
                    int tag = robot.turret.getVisionTagId();
                    tipDetector = tag >= 0 ? new HiveTipDetector(tag, robot.turret.getVisionPipeline()) : null;
                    monitoring = tipDetector != null;
                    robot.transfer.setFeeding(true);
                    robot.intake.restart();
                }),
                Commands.waitMs(feedDuration),
                Commands.instant(() -> robot.transfer.setFeeding(false)),
                Commands.lazy(() -> {
                    if (!monitoring) return skip();
                    status = "Watching for tip at " + hive;
                    return Commands.waitMs(tipObservationDuration);
                }),
                Commands.instant(() -> {
                    monitoring = false;
                    boolean tipped = tipDetector != null && tipDetector.tipped();
                    lastTip = tipDetector == null ? "UNCONFIRMED (no vision)" : tipDetector.state().toString();
                    plan.record(hive, tipped);
                    status = hive + ": " + lastTip;
                })
        );
    }

    private boolean shotReady() {
        return robot.turret.isAligned() && robot.shooter.isReady();
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
