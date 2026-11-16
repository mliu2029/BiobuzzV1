package org.firstinspires.ftc.teamcode.opmodes.auto.modular;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;

/** Explicit routes: pickup and shooting poses are destinations, never transit hubs. */
public final class TwoTipRoutes {
    public static final double FIELD_SIZE_INCHES = 141.5;
    private static final PoseFactory POSES = PoseFactory.degrees();
    public enum Stop {
        CLOSE_SHOT(53.0036, 6.0788, 90),
        FAR_START(64.2742, 134.4415, 270),
        GARDEN_PREP(6.5879, 16.2961, 270),
        GARDEN(6.469, 7.35, 270),
        FAR_SHOT(58.878, 128.2045, 270),
        FAR_FLOWER_PREP(46.75, 118.75, 90),
        FAR_FLOWER(47.2381, 132.3215, 90),
        SIDE_FLOWER_PREP(23.4507, 47.1668, 180),
        SIDE_FLOWER(11.495, 46.8662, 180),
        CLOSE_PARK(12.71, 93.61, 0),
        FAR_PARK(12, 118.64, 0);
        public final Pose pose;
        Stop(double x, double y, double heading) { pose = POSES.of(x, y, heading); }
    }

    public static final class Leg {
        public final Stop from;
        public final Stop to;
        private final List<Pose> controls;
        private Leg(Stop from, Stop to, Pose... controls) {
            this.from = from;
            this.to = to;
            this.controls = Collections.unmodifiableList(Arrays.asList(controls.clone()));
        }
        public Path path(Pose actualStart) {
            return path(actualStart, Alliance.RED);
        }

        public Path path(Pose actualStart, Alliance alliance) {
            Pose target = targetPose(alliance);
            if (controls.isEmpty()) return Paths.line(actualStart, target).linear(actualStart, target);
            Pose[] points = new Pose[controls.size() + 2];
            points[0] = actualStart;
            for (int i = 0; i < controls.size(); i++) {
                points[i + 1] = transformPose(controls.get(i), alliance);
            }
            points[points.length - 1] = target;
            return Paths.curve(points).linear(actualStart, target);
        }

        public Pose targetPose(Alliance alliance) {
            return transformPose(to.pose, alliance);
        }

        public Pose startPose(Alliance alliance) {
            return transformPose(from.pose, alliance);
        }

        public List<Pose> controlPoints() { return controls; }
        private Leg reversed() {
            Pose[] reversed = new Pose[controls.size()];
            for (int i = 0; i < reversed.length; i++) reversed[i] = controls.get(reversed.length - 1 - i);
            return new Leg(to, from, reversed);
        }
    }

    private final EnumMap<Stop, EnumMap<Stop, List<Leg>>> routes = new EnumMap<>(Stop.class);
    public TwoTipRoutes() {
        pair(leg(Stop.CLOSE_SHOT, Stop.GARDEN_PREP, 30.7223, 33.5367),
                leg(Stop.GARDEN_PREP, Stop.GARDEN));
        pair(leg(Stop.FAR_SHOT, Stop.GARDEN_PREP, 23.0036, 98.1166),
                leg(Stop.GARDEN_PREP, Stop.GARDEN));

        // Cross the left field corridor straight to the flower approach, without visiting a hive.
        pair(leg(Stop.CLOSE_SHOT, Stop.FAR_FLOWER_PREP, 20, 38, 25, 99),
                leg(Stop.FAR_FLOWER_PREP, Stop.FAR_FLOWER));
        pair(leg(Stop.FAR_SHOT, Stop.FAR_FLOWER_PREP, 51.4368, 113.8484),
                leg(Stop.FAR_FLOWER_PREP, Stop.FAR_FLOWER));

        pair(leg(Stop.CLOSE_SHOT, Stop.SIDE_FLOWER_PREP, 25, 21),
                leg(Stop.SIDE_FLOWER_PREP, Stop.SIDE_FLOWER));
        pair(leg(Stop.FAR_SHOT, Stop.SIDE_FLOWER_PREP, 28.7152, 98.3556),
                leg(Stop.SIDE_FLOWER_PREP, Stop.SIDE_FLOWER));

        // The far start is the existing partner-auto start, not the later far shooting pose.
        register(leg(Stop.FAR_START, Stop.GARDEN_PREP, 24, 112, 18, 45),
                leg(Stop.GARDEN_PREP, Stop.GARDEN));
        register(leg(Stop.FAR_START, Stop.FAR_FLOWER_PREP, 53, 115),
                leg(Stop.FAR_FLOWER_PREP, Stop.FAR_FLOWER));
        register(leg(Stop.FAR_START, Stop.SIDE_FLOWER_PREP, 29, 101),
                leg(Stop.SIDE_FLOWER_PREP, Stop.SIDE_FLOWER));

        // Cross-field partner selection carries the preload directly to its chosen hive.
        register(leg(Stop.CLOSE_SHOT, Stop.FAR_SHOT, 24, 38, 25, 103));
        register(leg(Stop.FAR_START, Stop.CLOSE_SHOT, 25, 103, 24, 38));

        // Finish from the actual stop; parking never requires an empty trip to FAR_SHOT.
        register(leg(Stop.CLOSE_SHOT, Stop.FAR_PARK, 25, 32, 30, 90));
        register(leg(Stop.FAR_SHOT, Stop.FAR_PARK, 40.278, 111.0821));
        register(leg(Stop.GARDEN, Stop.GARDEN_PREP),
                leg(Stop.GARDEN_PREP, Stop.FAR_PARK, 27, 40, 30, 93));
        register(leg(Stop.FAR_FLOWER, Stop.FAR_FLOWER_PREP),
                leg(Stop.FAR_FLOWER_PREP, Stop.FAR_PARK, 28, 104));
        register(leg(Stop.SIDE_FLOWER, Stop.SIDE_FLOWER_PREP),
                leg(Stop.SIDE_FLOWER_PREP, Stop.FAR_PARK, 32, 88));
        register(leg(Stop.FAR_START, Stop.FAR_PARK, 41, 108));

        register(leg(Stop.CLOSE_SHOT, Stop.CLOSE_PARK, 30, 32));
        register(leg(Stop.FAR_SHOT, Stop.CLOSE_PARK, 30, 102));
        register(leg(Stop.FAR_START, Stop.CLOSE_PARK, 35, 106));
        register(leg(Stop.GARDEN, Stop.GARDEN_PREP), leg(Stop.GARDEN_PREP, Stop.CLOSE_PARK, 26, 42, 26, 73));
        register(leg(Stop.FAR_FLOWER, Stop.FAR_FLOWER_PREP),
                leg(Stop.FAR_FLOWER_PREP, Stop.CLOSE_PARK, 27, 96));
        register(leg(Stop.SIDE_FLOWER, Stop.SIDE_FLOWER_PREP),
                leg(Stop.SIDE_FLOWER_PREP, Stop.CLOSE_PARK, 27, 76));
    }

    private static Leg leg(Stop from, Stop to, double... xy) {
        if (xy.length % 2 != 0) throw new IllegalArgumentException("Control points need X/Y pairs");
        Pose[] controls = new Pose[xy.length / 2];
        for (int i = 0; i < controls.length; i++) controls[i] = POSES.of(xy[2*i], xy[2*i+1], 0);
        return new Leg(from, to, controls);
    }

    private void pair(Leg... forward) {
        register(forward);
        Leg[] reverse = new Leg[forward.length];
        for (int i = 0; i < reverse.length; i++) reverse[i] = forward[reverse.length - 1 - i].reversed();
        register(reverse);
    }

    private void register(Leg... legs) {
        for (int i = 1; i < legs.length; i++) {
            if (legs[i-1].to != legs[i].from) throw new IllegalArgumentException("Disconnected route");
        }
        Stop from = legs[0].from;
        Stop to = legs[legs.length - 1].to;
        EnumMap<Stop, List<Leg>> destinations = routes.get(from);
        if (destinations == null) {
            destinations = new EnumMap<>(Stop.class);
            routes.put(from, destinations);
        }
        destinations.put(to, Collections.unmodifiableList(new ArrayList<>(Arrays.asList(legs))));
    }

    /** Only explicitly authored scenario routes can be followed. No shortest-path detours. */
    public List<Leg> between(Stop from, Stop to) {
        if (from == to) return Collections.emptyList();
        EnumMap<Stop, List<Leg>> destinations = routes.get(from);
        List<Leg> route = destinations == null ? null : destinations.get(to);
        if (route == null) throw new IllegalArgumentException("No scenario route from " + from + " to " + to);
        return route;
    }

    public static Stop pickup(TwoTipPlan.Pickup pickup) {
        switch (pickup) {
            case GARDEN: return Stop.GARDEN;
            case FAR_FLOWER: return Stop.FAR_FLOWER;
            case SIDE_FLOWER: return Stop.SIDE_FLOWER;
            default: throw new IllegalArgumentException("Unknown pickup");
        }
    }

    public static Stop start(TwoTipPlan.Hive side) {
        return side == TwoTipPlan.Hive.CLOSE ? Stop.CLOSE_SHOT : Stop.FAR_START;
    }

    public static Stop shooting(TwoTipPlan.Hive hive) {
        return hive == TwoTipPlan.Hive.CLOSE ? Stop.CLOSE_SHOT : Stop.FAR_SHOT;
    }

    public static Stop park(TwoTipPlan.Hive side) {
        return side == TwoTipPlan.Hive.CLOSE ? Stop.CLOSE_PARK : Stop.FAR_PARK;
    }

    public static Pose transformPose(Pose pose, Alliance alliance) {
        if (alliance == Alliance.RED || pose == null) return pose;
        double x = FIELD_SIZE_INCHES - pose.x();
        double y = FIELD_SIZE_INCHES - pose.y();
        double headingDegrees = Calculations.normalize(Math.toDegrees(pose.heading()) + 180.0);
        return POSES.of(x, y, headingDegrees);
    }

    public static Pose startPose(TwoTipPlan.Hive side, Alliance alliance) {
        return transformPose(start(side).pose, alliance);
    }

    public static Pose shootingPose(TwoTipPlan.Hive hive, Alliance alliance) {
        return transformPose(shooting(hive).pose, alliance);
    }

    public static Pose parkPose(TwoTipPlan.Hive side, Alliance alliance) {
        return transformPose(park(side).pose, alliance);
    }

    public static Pose pickupPose(TwoTipPlan.Pickup pickup, Alliance alliance) {
        return transformPose(pickup(pickup).pose, alliance);
    }

    public static double distance(Pose a, Pose b) { return Math.hypot(a.x()-b.x(), a.y()-b.y()); }
}
