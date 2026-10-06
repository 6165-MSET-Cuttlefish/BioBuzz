package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Turns a visit order into the two drives of a {@link RouteOptimizer.Route}. The intake leg is one curve from the
 * start with the robot's centre passing through each essential ball ({@link IntakeCurvePlanner}), facing along it;
 * the return is a straight line back, turning evenly to the return heading. A leg whose footprint would leave the
 * area or touch an obstacle is rebuilt as Pedro's spline through the {@link VisibilityGraphPlanner} detour waypoints, and
 * failing that as straight hops through them.
 */
public final class RoutePathBuilder {
    /** Points closer than this are one point: Pedro rejects a zero-length line, and a tiny hop has no usable heading. */
    public static final double COINCIDENT_IN = 1.0;
    // Headings tried when checking a ball can be driven over, in RouteOptimizer.ballProblem: every 5 degrees.
    static final int HEADING_STEPS = 72;
    // Joins within this of each other in heading are driven without a turn in place.
    private static final double SMOOTH_JOIN = Math.toRadians(1);
    private static final double TURN_CHECK_STEP = Math.toRadians(2);

    private RoutePathBuilder() {}

    /** How a leg was built, from the simplest. */
    public enum Reroute { NONE, SPLINE, POLYLINE, FORCED_SPLINE }

    private static final class Leg {
        final List<Segment> segments;
        final Reroute reroute;
        final double length;

        Leg(List<Segment> segments, Reroute reroute) {
            this.segments = segments;
            this.reroute = reroute;
            this.length = totalLength(segments);
        }
    }

    /** The geometry chosen for one visit order, before any Pedro path is built. */
    static final class Shape {
        final List<Segment> intake;
        final Reroute intakeReroute;
        final List<Segment> back;
        final Reroute returnReroute;
        final Pose intakeEnd;
        final int essentialStops;
        final double intakeLength;
        final double returnLength;
        final List<Move> moves;
        private final RobotShape robot;

        Shape(Leg intake, Leg back, Pose intakeEnd, int essentialStops, List<Move> moves, RobotShape robot) {
            this.intake = intake.segments;
            this.intakeReroute = intake.reroute;
            this.back = back.segments;
            this.returnReroute = back.reroute;
            this.intakeEnd = intakeEnd;
            this.essentialStops = essentialStops;
            this.intakeLength = intake.length;
            this.returnLength = back.length;
            this.moves = moves;
            this.robot = robot;
        }

        double length() {
            return intakeLength + returnLength;
        }
    }

    private static final class Move {
        final Segment drive;
        final Pose turnTo;
        final boolean intake;

        Move(Segment drive, Pose turnTo, boolean intake) {
            this.drive = drive;
            this.turnTo = turnTo;
            this.intake = intake;
        }
    }

    /**
     * One thing for the follower to do: drive {@link #path} to a settled stop, or turn in place to
     * {@link #turnTo}. A plan splits wherever the heading jumps, so the robot faces each drive before starting it.
     */
    public static final class Step {
        /** Null for a turn. */
        public final Path path;
        /** Null for a drive. */
        public final Pose turnTo;
        /** Sampled length of a drive in inches; 0 for a turn. */
        public final double length;
        public final boolean intake;
        /** Always the opposite of {@link #intake}. */
        public final boolean back;

        Step(Path path, Pose turnTo, double length, boolean intake) {
            this.path = path;
            this.turnTo = turnTo;
            this.length = length;
            this.intake = intake;
            this.back = !intake;
        }
    }

    public static final class Plan {
        /** Null when there are no balls. */
        public final Path intake;
        /** Null when the intake leg already ends at the return position. */
        public final Path back;
        /** The robot's centre on the last ball, facing the way it arrived. */
        public final Pose intakeEnd;
        public final int essentialStops;
        public final Reroute intakeReroute;
        public final Reroute returnReroute;
        /** Sampled lengths in inches, of the robot's centre. */
        public final double intakeLength;
        public final double returnLength;
        /** Where the middle of the intake goes on the intake leg: x then y, Pedro inches; empty without balls. */
        public final double[][] intakeTrack;
        /** The intake leg then the return, as {@link RouteRun} drives them. */
        public final List<Step> steps;

        Plan(Shape shape) {
            this.intake = toPath(shape.intake);
            this.back = toPath(shape.back);
            this.intakeEnd = shape.intakeEnd;
            this.essentialStops = shape.essentialStops;
            this.intakeReroute = shape.intakeReroute;
            this.returnReroute = shape.returnReroute;
            this.intakeLength = shape.intakeLength;
            this.returnLength = shape.returnLength;
            this.intakeTrack = track(shape.intake, shape.robot);
            List<Step> steps = new ArrayList<>(shape.moves.size());
            for (Move m : shape.moves) {
                steps.add(m.drive == null ? new Step(null, m.turnTo, 0, m.intake)
                        : new Step(m.drive.toPath(), null, m.drive.length(), m.intake));
            }
            this.steps = steps;
        }
    }

    /** The Pedro paths for exactly the geometry the route was scored on. */
    public static Plan build(RouteOptimizer.Route route) {
        return new Plan(route.shape);
    }

    private static Path toPath(List<Segment> segments) {
        if (segments.isEmpty()) return null;
        if (segments.size() == 1) return segments.get(0).toPath();
        Path[] parts = new Path[segments.size()];
        for (int i = 0; i < parts.length; i++) parts[i] = segments.get(i).toPath();
        return Paths.path(parts);
    }

    /** Shapes for many visit orders of the same balls, sharing every detour they have in common. */
    static final class Planner {
        private static final int START = -1;

        private final Pose start;
        private final Pose returnPose;
        private final Ball[] balls;
        private final Region area;
        private final List<Obstacle> obstacles;
        private final RobotShape robot;
        private final double gap;
        private final boolean forceSplineOnly;
        private final double graphClearance;
        private final Map<Long, VisibilityGraphPlanner.Leg> detours = new HashMap<>();

        Planner(Pose start, Ball[] balls, Pose returnPose, Region area, List<Obstacle> obstacles, RobotShape robot,
                double gap, boolean forceSplineOnly) {
            this.start = start;
            this.balls = balls;
            this.returnPose = returnPose;
            this.area = area;
            this.obstacles = obstacles;
            this.robot = robot;
            this.gap = gap;
            this.forceSplineOnly = forceSplineOnly;
            graphClearance = Math.max(robot.widthIn, robot.intakeWidthIn) / 2 + gap;
        }

        /** No drive for {@code order} is shorter: every one passes its essential stops in turn, then the return pose. */
        double lowerBound(int[] order) {
            if (order.length == 0) return 0;
            Ball[] ordered = new Ball[order.length];
            for (int i = 0; i < order.length; i++) ordered[i] = balls[order[i]];
            double total = 0;
            Pose at = start;
            for (int s : IntakeCurvePlanner.essentialStops(start.x(), start.y(), ordered, robot.intakeWidthIn)) {
                Pose next = ordered[s].toPose();
                total += at.distance(next);
                at = next;
            }
            return total + at.distance(returnPose);
        }

        /** Null if no leg built for {@code order} keeps the footprint inside the area and clear of the obstacles. */
        Shape shape(int[] order) {
            Ball[] ordered = new Ball[order.length];
            for (int i = 0; i < order.length; i++) ordered[i] = balls[order[i]];

            Leg intake;
            int essential = 0;
            if (order.length == 0) {
                intake = new Leg(Collections.<Segment>emptyList(), Reroute.NONE);
            } else {
                int[] stops = IntakeCurvePlanner.essentialStops(start.x(), start.y(), ordered, robot.intakeWidthIn);
                essential = stops.length;
                intake = intakeLeg(order, ordered, stops);
                if (intake == null) return null;
            }

            Pose intakeEnd = start;
            if (!intake.segments.isEmpty()) {
                Segment last = intake.segments.get(intake.segments.size() - 1);
                intakeEnd = new Pose(last.end().x(), last.end().y(), last.endHeading());
            }
            Leg back = returnLeg(intakeEnd, order.length == 0 ? START : order[order.length - 1]);
            if (back == null) return null;

            List<Move> moves = moves(intake.segments, back.segments);
            if (moves == null) return null;
            return new Shape(intake, back, intakeEnd, essential, moves, robot);
        }

        private Leg intakeLeg(int[] order, Ball[] ordered, int[] stops) {
            List<Pose> stopPoses = new ArrayList<>(stops.length);
            for (int s : stops) stopPoses.add(ordered[s].toPose());
            Segment direct = Segment.tangent(IntakeCurvePlanner.throughCurve(start, stopPoses));
            if (direct.fits(robot, area, obstacles, gap)) return new Leg(Collections.singletonList(direct), Reroute.NONE);

            // Detours always go through every ball: safety over the few inches the intake's width would save.
            List<Pose> waypoints = new ArrayList<>();
            waypoints.add(start);
            int from = START;
            for (int to : order) {
                VisibilityGraphPlanner.Leg detour = detour(from, to);
                if (detour == null) return null;
                for (int j = 1; j < detour.waypoints.size(); j++) addDistinct(waypoints, detour.waypoints.get(j));
                from = to;
            }
            Segment spline = Segment.tangent(IntakeCurvePlanner.through(waypoints));
            if (spline.fits(robot, area, obstacles, gap)) return new Leg(Collections.singletonList(spline), Reroute.SPLINE);
            if (forceSplineOnly) return new Leg(Collections.singletonList(spline), Reroute.FORCED_SPLINE);

            List<Segment> hops = new ArrayList<>();
            for (int i = 1; i < waypoints.size(); i++) hops.add(Segment.tangent(waypoints.get(i - 1), waypoints.get(i)));
            return allFit(hops) ? new Leg(hops, Reroute.POLYLINE) : null;
        }

        private Leg returnLeg(Pose from, int fromBall) {
            if (from.distance(returnPose) < COINCIDENT_IN) return new Leg(Collections.<Segment>emptyList(), Reroute.NONE);
            Segment line = Segment.turning(from.heading(), returnPose.heading(), from, returnPose);
            if (line.fits(robot, area, obstacles, gap)) return new Leg(Collections.singletonList(line), Reroute.NONE);

            VisibilityGraphPlanner.Leg detour = detour(fromBall, Integer.MAX_VALUE);
            if (detour == null) return null;
            List<Pose> waypoints = new ArrayList<>();
            waypoints.add(from);
            for (int j = 1; j < detour.waypoints.size(); j++) addDistinct(waypoints, detour.waypoints.get(j));
            if (waypoints.size() < 2) return new Leg(Collections.<Segment>emptyList(), Reroute.NONE);

            Segment spline = Segment.turning(from.heading(), returnPose.heading(), IntakeCurvePlanner.through(waypoints));
            if (spline.fits(robot, area, obstacles, gap)) return new Leg(Collections.singletonList(spline), Reroute.SPLINE);

            List<Segment> hops = new ArrayList<>();
            for (int i = 1; i < waypoints.size(); i++) {
                Pose a = waypoints.get(i - 1);
                Pose b = waypoints.get(i);
                hops.add(i == waypoints.size() - 1
                        ? Segment.turning(Math.atan2(b.y() - a.y(), b.x() - a.x()), returnPose.heading(), a, b)
                        : Segment.tangent(a, b));
            }
            return allFit(hops) ? new Leg(hops, Reroute.POLYLINE) : null;
        }

        // Keyed by ball indices (START for the start, MAX_VALUE for the return pose), as they don't depend on the order.
        private VisibilityGraphPlanner.Leg detour(int from, int to) {
            long key = ((long) from << 32) ^ (to & 0xffffffffL);
            if (detours.containsKey(key)) return detours.get(key);
            Pose a = from == START ? start : balls[from].toPose();
            Pose b = to == Integer.MAX_VALUE ? returnPose : balls[to].toPose();
            VisibilityGraphPlanner.Leg leg = VisibilityGraphPlanner.planPath(a, b, area, obstacles, graphClearance);
            detours.put(key, leg);
            return leg;
        }

        // Null if a turn in place between drives doesn't fit.
        private List<Move> moves(List<Segment> intake, List<Segment> back) {
            List<Move> moves = new ArrayList<>();
            double heading = start.heading();
            for (int i = 0; i < intake.size() + back.size(); i++) {
                boolean inIntake = i < intake.size();
                Segment s = inIntake ? intake.get(i) : back.get(i - intake.size());
                if (!turn(moves, s.start(), heading, s.startHeading(), inIntake)) return null;
                moves.add(new Move(s, null, inIntake));
                heading = s.endHeading();
            }
            Pose end = back.isEmpty() ? (intake.isEmpty() ? start : intake.get(intake.size() - 1).end())
                    : back.get(back.size() - 1).end();
            return turn(moves, end, heading, returnPose.heading(), false) ? moves : null;
        }

        private boolean turn(List<Move> moves, Pose at, double from, double to, boolean intake) {
            double sweep = Angle.normalizeSigned(to - from);
            if (Math.abs(sweep) <= SMOOTH_JOIN) return true;
            int n = (int) Math.ceil(Math.abs(sweep) / TURN_CHECK_STEP);
            for (int i = 0; i <= n; i++) {
                if (!robot.fits(at.x(), at.y(), from + sweep * i / n, area, obstacles, gap)) return false;
            }
            moves.add(new Move(null, new Pose(at.x(), at.y(), Angle.normalize(to)), intake));
            return true;
        }

        private boolean allFit(List<Segment> segments) {
            for (Segment s : segments) {
                if (!s.fits(robot, area, obstacles, gap)) return false;
            }
            return true;
        }
    }

    private static double[][] track(List<Segment> intake, RobotShape robot) {
        int n = 0;
        for (Segment s : intake) n += s.xs().length;
        double[][] track = new double[2][n];
        int k = 0;
        for (Segment s : intake) {
            double[] xs = s.xs();
            double[] ys = s.ys();
            double[] hs = s.headings();
            for (int i = 0; i < xs.length; i++, k++) {
                track[0][k] = xs[i] + robot.intakeOffsetIn * Math.cos(hs[i]);
                track[1][k] = ys[i] + robot.intakeOffsetIn * Math.sin(hs[i]);
            }
        }
        return track;
    }

    private static void addDistinct(List<Pose> points, Pose p) {
        if (points.get(points.size() - 1).distance(p) > 1e-9) points.add(p);
    }

    private static double totalLength(List<Segment> segments) {
        double total = 0;
        for (Segment s : segments) total += s.length();
        return total;
    }
}
