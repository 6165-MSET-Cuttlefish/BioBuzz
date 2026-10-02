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
 * Turns a visit order into the two drives of a {@link RouteOptimizer.Route}. The intake leg drives forward so the
 * intake passes over every ball: it aims the intake at each essential ball along the direction from the ball
 * before, putting the robot's centre {@link RobotShape#intakeOffsetIn} behind it, and sweeps up the rest on the
 * way. The return then drives back to the return pose and turns to its heading. Every leg tries a smooth curve,
 * then turning in place first, then a detour around the obstacles, and is used only if the robot's whole
 * footprint stays inside the area and clear of every obstacle at every sample.
 */
public final class RoutePathBuilder {
    /** Points closer than this are one point: Pedro rejects a zero-length line, and a tiny hop has no usable heading. */
    public static final double COINCIDENT_IN = 1.0;
    // A turn in place is ranked like driving this far per radian, so a route isn't chosen on length alone.
    static final double TURN_COST_IN_PER_RAD = 10;
    // How far a leg will drive straight out of a tight spot, or into one, to reach room to turn around.
    private static final double MAX_ESCAPE_IN = 36;
    private static final double ESCAPE_STEP_IN = 1;
    // A detour lines the intake up this far before a ball where it can, so the ball comes in straight.
    private static final double STRAIGHT_APPROACH_IN = 6;
    // Headings tried when aiming the intake at a ball, here and in RouteOptimizer.ballProblem: every 5 degrees.
    static final int HEADING_STEPS = 72;
    // Cubic handle lengths tried, as fractions of the chord; a third is the natural spline's. Below 1: at 1 a straight
    // cubic stops dead halfway, where Pedro's follower can't normalize the tangent and throws.
    private static final double[] HANDLE_REACHES = {1.0 / 3, 0.5, 0.75, 0.95};
    // Ways to drive straight out of a tight spot relative to the heading: forward, backing up, then strafing.
    private static final double[] SIDEWAYS = {0, Math.PI, Math.PI / 2, -Math.PI / 2};
    // Rounding at the end of a curve leaves the intake a hair short of the ball it was aimed at.
    private static final double HIT_TOLERANCE_IN = 0.01;
    // Sideways over forward motion of a ball relative to the intake as it goes in: 0.75 is about 37 degrees off head-on.
    private static final double HEAD_ON = 0.75;
    // Joins within this of each other in heading and in direction of travel are driven straight through.
    private static final double SMOOTH_JOIN = Math.toRadians(1);
    // Aiming off the intake's middle keeps the ball's edge this far inside the intake's edge, for the follower's error.
    private static final double SIDE_MARGIN_IN = 1;
    private static final double SIDE_STEP_IN = 0.5;

    private RoutePathBuilder() {}

    /** How a leg was built, from the simplest: later values mean more turning or a longer way round. */
    public enum Reroute { NONE, TURN_IN_PLACE, SPLINE, POLYLINE, FORCED_SPLINE }

    private static final class Leg {
        final List<Segment> segments;
        final Reroute reroute;
        final double length;
        final double turns;
        // Per planner ball: 1 if this leg's intake passes over it, -1 if not, 0 not yet checked.
        byte[] sweeps;

        Leg(List<Segment> segments, Reroute reroute, double turns) {
            this.segments = segments;
            this.reroute = reroute;
            this.turns = turns;
            double total = 0;
            for (Segment s : segments) total += s.length();
            this.length = total;
        }

        double cost() {
            return length + TURN_COST_IN_PER_RAD * turns;
        }
    }

    private static final Leg STAY = new Leg(Collections.<Segment>emptyList(), Reroute.NONE, 0);

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
        final double turns;
        private final RobotShape robot;

        Shape(List<Segment> intake, Reroute intakeReroute, List<Segment> back, Reroute returnReroute, Pose intakeEnd,
              int essentialStops, double turns, RobotShape robot) {
            this.intake = intake;
            this.intakeReroute = intakeReroute;
            this.back = back;
            this.returnReroute = returnReroute;
            this.intakeEnd = intakeEnd;
            this.essentialStops = essentialStops;
            this.intakeLength = totalLength(intake);
            this.returnLength = totalLength(back);
            this.turns = turns;
            this.robot = robot;
        }

        double length() {
            return intakeLength + returnLength;
        }

        double cost() {
            return length() + TURN_COST_IN_PER_RAD * turns;
        }
    }

    /**
     * One thing for the follower to do: drive {@link #path} to a settled stop, or turn in place to
     * {@link #turnTo}. A plan splits wherever the heading or the direction of travel jumps, because Pedro moves on
     * to the next piece of a path before reaching the end of this one, which would start a turn, or a reversal,
     * before the intake reached its ball.
     */
    public static final class Step {
        /** Null for a turn. */
        public final Path path;
        /** Null for a drive. */
        public final Pose turnTo;
        /** Sampled length of a drive in inches; 0 for a turn. */
        public final double length;
        /** Part of the intake leg rather than the return. */
        public final boolean intake;

        Step(Path path, Pose turnTo, double length, boolean intake) {
            this.path = path;
            this.turnTo = turnTo;
            this.length = length;
            this.intake = intake;
        }
    }

    public static final class Plan {
        /** Null when there are no balls. */
        public final Path intake;
        /** Null when the intake leg already ends at the return position. */
        public final Path back;
        /** The robot's centre where the intake reaches the last ball. */
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

        Plan(Path intake, Path back, Shape shape, Pose start, Pose returnPose) {
            this.intake = intake;
            this.back = back;
            this.intakeEnd = shape.intakeEnd;
            this.essentialStops = shape.essentialStops;
            this.intakeReroute = shape.intakeReroute;
            this.returnReroute = shape.returnReroute;
            this.intakeLength = shape.intakeLength;
            this.returnLength = shape.returnLength;
            this.intakeTrack = track(shape.intake, shape.robot);
            this.steps = steps(start.heading(), shape, returnPose);
        }
    }

    /** The Pedro paths for exactly the geometry the route was scored on. */
    public static Plan build(RouteOptimizer.Route route) {
        Shape shape = route.shape;
        return new Plan(toPath(shape.intake), toPath(shape.back), shape, route.start, route.returnPose);
    }

    private static List<Step> steps(double startHeading, Shape shape, Pose returnPose) {
        List<Step> steps = new ArrayList<>();
        double[] heading = {startHeading};
        addSteps(steps, shape.intake, true, heading);
        addSteps(steps, shape.back, false, heading);
        if (!shape.back.isEmpty() && Math.abs(Angle.normalizeSigned(returnPose.heading() - heading[0])) > SMOOTH_JOIN) {
            steps.add(new Step(null, returnPose, 0, false));
        }
        return steps;
    }

    private static void addSteps(List<Step> steps, List<Segment> segments, boolean intake, double[] heading) {
        List<Segment> run = new ArrayList<>();
        for (Segment s : segments) {
            boolean turn = Math.abs(Angle.normalizeSigned(s.startHeading() - heading[0])) > SMOOTH_JOIN;
            Segment before = run.isEmpty() ? null : run.get(run.size() - 1);
            boolean swerve = before != null
                    && Math.abs(Angle.normalizeSigned(s.startTravel() - before.endTravel())) > SMOOTH_JOIN;
            if (turn || swerve) flush(steps, run, intake);
            if (turn) steps.add(new Step(null, new Pose(s.start().x(), s.start().y(), s.startHeading()), 0, intake));
            run.add(s);
            heading[0] = s.endHeading();
        }
        flush(steps, run, intake);
    }

    private static void flush(List<Step> steps, List<Segment> run, boolean intake) {
        if (run.isEmpty()) return;
        steps.add(new Step(toPath(run), null, totalLength(run), intake));
        run.clear();
    }

    private static Path toPath(List<Segment> segments) {
        if (segments.isEmpty()) return null;
        if (segments.size() == 1) return segments.get(0).toPath();
        Path[] parts = new Path[segments.size()];
        for (int i = 0; i < parts.length; i++) parts[i] = segments.get(i).toPath();
        return Paths.path(parts);
    }

    /** Shapes for many visit orders of the same balls, sharing every leg they have in common. */
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
        // Where the robot's centre can turn in place: the visibility graph routes only through there.
        private final Region turnArea;
        private final double turnClearance;
        private final double[][] sides;
        private final Map<Integer, double[]> stops = new HashMap<>();
        private final Map<Long, Leg> legs = new HashMap<>();
        private final Map<Integer, Leg> returns = new HashMap<>();

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
            double r = robot.turnRadius();
            turnArea = area.maxX - area.minX > 2 * r && area.maxY - area.minY > 2 * r
                    ? new Region(area.minX + r, area.maxX - r, area.minY + r, area.maxY - r) : null;
            turnClearance = r + gap;
            sides = new double[balls.length][];
            for (int i = 0; i < balls.length; i++) sides[i] = aimSides(robot, balls[i].radius);
        }

        /** Null when no drivable shape visits the balls in {@code order}, given as indices into the planner's balls. */
        Shape shape(int[] order) {
            if (order.length == 0) return assemble(order, new int[0]);
            Ball[] ordered = new Ball[order.length];
            for (int i = 0; i < order.length; i++) ordered[i] = balls[order[i]];
            Pose intake = robot.intakeAt(start);
            int[] essential = IntakeCurvePlanner.essentialStops(intake.x(), intake.y(), ordered, robot.intakeWidthIn);
            Shape shape = assemble(order, essential);
            if (shape != null || essential.length == order.length) return shape;
            // The curve between aimed-at balls bends away from the straight line the sweep was judged on.
            int[] all = new int[order.length];
            for (int i = 0; i < all.length; i++) all[i] = i;
            return assemble(order, all);
        }

        private Shape assemble(int[] order, int[] essential) {
            List<Leg> chosen = new ArrayList<>(essential.length);
            int previous = START;
            int current = START;
            for (int index : essential) {
                int next = order[index];
                Leg leg = leg(previous, current, next);
                if (leg == null) return null;
                chosen.add(leg);
                previous = current;
                current = next;
            }
            for (int ball : order) {
                if (!sweptByAny(chosen, ball)) return null;
            }
            Leg back = returnLeg(previous, current);
            if (back == null) return null;
            List<Segment> intake = new ArrayList<>();
            Reroute intakeReroute = Reroute.NONE;
            double turns = back.turns;
            for (Leg leg : chosen) {
                intake.addAll(leg.segments);
                if (leg.reroute.compareTo(intakeReroute) > 0) intakeReroute = leg.reroute;
                turns += leg.turns;
            }
            Pose intakeEnd = current == START ? start : centre(previous, current);
            return new Shape(intake, intakeReroute, back.segments, back.reroute, intakeEnd, essential.length, turns, robot);
        }

        private boolean sweptByAny(List<Leg> chosen, int ball) {
            for (Leg leg : chosen) {
                if (leg.sweeps == null) leg.sweeps = new byte[balls.length];
                if (leg.sweeps[ball] == 0) leg.sweeps[ball] = (byte) (swept(leg.segments, balls[ball]) ? 1 : -1);
                if (leg.sweeps[ball] > 0) return true;
            }
            return false;
        }

        /** Heading then centre (x, y) of the stop at {@code ball} approached from {@code from}, or null if there is none. */
        private double[] stop(int from, int ball) {
            int key = (from + 1) * balls.length + ball;
            if (stops.containsKey(key)) return stops.get(key);
            Ball b = balls[ball];
            double fromX;
            double fromY;
            if (from == START) {
                Pose intake = robot.intakeAt(start);
                fromX = intake.x();
                fromY = intake.y();
            } else {
                fromX = balls[from].x;
                fromY = balls[from].y;
            }
            double[] aimed = aim(ball, Math.atan2(b.y - fromY, b.x - fromX));
            double[] stop = null;
            if (aimed != null) {
                Pose c = robot.centreFor(b.x, b.y, aimed[0], aimed[1]);
                stop = new double[] {aimed[0], c.x(), c.y()};
            }
            stops.put(key, stop);
            return stop;
        }

        private Pose centre(int from, int ball) {
            double[] s = stop(from, ball);
            return new Pose(s[1], s[2], s[0]);
        }

        /**
         * The heading to aim the intake at a ball with, and where across the intake (inches left of its middle):
         * {@code wanted} on the middle if the robot fits there and could line up on the ball straight from somewhere
         * it can turn, else the nearest heading that can, else {@code wanted} or the nearest heading that only fits;
         * at each heading the point nearest the middle that fits. Null if none fits.
         */
        private double[] aim(int ball, double wanted) {
            Ball b = balls[ball];
            if (lineUpFrom(b, wanted, 0) != null) return new double[] {Angle.normalize(wanted), 0};
            Aims grid = headings(ball);
            int best = -1;
            double bestOff = Double.POSITIVE_INFINITY;
            boolean bestLinesUp = false;
            for (int i = 0; i < HEADING_STEPS; i++) {
                if (grid.state[i] == 0) continue;
                double off = Math.abs(Angle.normalizeSigned(2 * Math.PI * i / HEADING_STEPS - wanted));
                boolean linesUp = grid.state[i] == 2;
                if (linesUp && !bestLinesUp || linesUp == bestLinesUp && off < bestOff) {
                    best = i;
                    bestOff = off;
                    bestLinesUp = linesUp;
                }
            }
            if (!bestLinesUp) {
                double side = nearestFit(ball, wanted);
                if (!Double.isNaN(side)) return new double[] {Angle.normalize(wanted), side};
            }
            return best < 0 ? null : new double[] {2 * Math.PI * best / HEADING_STEPS, grid.side[best]};
        }

        // Per ball and grid heading: where across the intake the robot first fits, and 0 if it doesn't fit, 1 if it
        // fits, 2 if it also lines up straight from a turning spot.
        private static final class Aims {
            final double[] side = new double[HEADING_STEPS];
            final byte[] state = new byte[HEADING_STEPS];
        }

        private final Map<Integer, Aims> aimGrids = new HashMap<>();

        private Aims headings(int ball) {
            Aims grid = aimGrids.get(ball);
            if (grid != null) return grid;
            grid = new Aims();
            Ball b = balls[ball];
            for (int i = 0; i < HEADING_STEPS; i++) {
                double h = 2 * Math.PI * i / HEADING_STEPS;
                double side = nearestFit(ball, h);
                grid.side[i] = side;
                grid.state[i] = (byte) (Double.isNaN(side) ? 0 : lineUpFrom(b, h, side) != null ? 2 : 1);
            }
            aimGrids.put(ball, grid);
            return grid;
        }

        /** The point across the intake nearest its middle at which aiming at the ball fits; NaN if none does. */
        private double nearestFit(int ball, double heading) {
            for (double side : sides[ball]) {
                if (fitsAimedAt(balls[ball], heading, side)) return side;
            }
            return Double.NaN;
        }

        private boolean fitsAimedAt(Ball b, double heading, double side) {
            Pose c = robot.centreFor(b.x, b.y, heading, side);
            return robot.fits(c.x(), c.y(), heading, area, obstacles, gap);
        }

        /** Where the robot can turn in place and then drive straight on to the stop aiming at {@code b}; null if nowhere. */
        private Pose lineUpFrom(Ball b, double heading, double side) {
            if (!fitsAimedAt(b, heading, side)) return null;
            // A turn on the stop itself would swing the ball in sideways, so the straight run in has some length.
            return escape(robot.centreFor(b.x, b.y, heading, side), heading, Math.PI, STRAIGHT_APPROACH_IN);
        }

        /** From the stop at {@code current} (approached from {@code previous}) to the stop at {@code next}. */
        private Leg leg(int previous, int current, int next) {
            long key = ((long) (previous + 1) * (balls.length + 1) + (current + 1)) * balls.length + next;
            if (legs.containsKey(key)) return legs.get(key);
            Leg leg = null;
            double[] to = stop(current, next);
            if (to != null) {
                Pose from = current == START ? start : centre(previous, current);
                Pose target = new Pose(to[1], to[2], to[0]);
                Ball ball = balls[next];
                leg = cheapest(hitting(direct(from, target), ball), hitting(turnFirst(from, target), ball));
                if (leg == null) leg = hitting(detour(from, target, false), ball);
            }
            legs.put(key, leg);
            return leg;
        }

        private Leg returnLeg(int previous, int current) {
            int key = (previous + 1) * (balls.length + 1) + (current + 1);
            if (returns.containsKey(key)) return returns.get(key);
            Pose from = current == START ? start : centre(previous, current);
            Leg leg;
            if (from.distance(returnPose) < COINCIDENT_IN) {
                leg = STAY;
            } else {
                Segment turning = Segment.turning(from.heading(), returnPose.heading(), from, returnPose);
                Segment straight = Segment.turning(from.heading(), from.heading(), from, returnPose);
                leg = cheapest(
                        turning.fits(robot, area, obstacles, gap)
                                ? new Leg(Collections.singletonList(turning), Reroute.NONE, 0) : null,
                        straight.fits(robot, area, obstacles, gap) && canTurnAtReturn()
                                ? new Leg(Collections.singletonList(straight), Reroute.TURN_IN_PLACE,
                                        Math.abs(Angle.normalizeSigned(returnPose.heading() - from.heading()))) : null);
                if (leg == null) leg = detour(from, returnPose, true);
            }
            returns.put(key, leg);
            return leg;
        }

        private boolean canTurnAtReturn() {
            return robot.canTurnAt(returnPose.x(), returnPose.y(), area, obstacles, gap);
        }

        /** The shortest of the cubics leaving {@code from} along its heading and arriving at {@code to} along its heading. */
        private Leg direct(Pose from, Pose to) {
            Segment s = gentlest(from, from.heading(), to, to.heading());
            return s == null ? null : new Leg(Collections.singletonList(s), Reroute.NONE, 0);
        }

        /** Turns in place to face {@code to}, then the same cubics from there. */
        private Leg turnFirst(Pose from, Pose to) {
            if (!robot.canTurnAt(from.x(), from.y(), area, obstacles, gap)) return null;
            double facing = Math.atan2(to.y() - from.y(), to.x() - from.x());
            Segment s = gentlest(from, facing, to, to.heading());
            if (s == null) return null;
            return new Leg(Collections.singletonList(s), Reroute.TURN_IN_PLACE,
                    Math.abs(Angle.normalizeSigned(facing - from.heading())));
        }

        // Longer handles round off a big change of heading that short ones would take as a cusp.
        private Segment gentlest(Pose from, double leaving, Pose to, double arriving) {
            Segment best = null;
            for (double reach : HANDLE_REACHES) {
                Segment s = hermite(from, leaving, to, arriving, reach);
                // Ties go to the shorter handle: on a straight leg every reach draws the same line.
                if (s != null && (best == null || s.length() < best.length() - 1e-9) && s.fits(robot, area, obstacles, gap)) {
                    best = s;
                }
            }
            return best;
        }

        /**
         * Drives straight out of {@code from} at its heading (forward, backing up or strafing) to where the robot can
         * turn, through the visibility graph (built for the turning circle, so every hop and corner is clear at any
         * heading), then straight in to {@code to} at its heading: to a ball forward, lined up some way back where
         * there's room; to the return pose any of the ways out.
         */
        private Leg detour(Pose from, Pose to, boolean toReturnPose) {
            if (turnArea == null) return null;
            Leg best = null;
            for (double out : SIDEWAYS) {
                Pose exit = escape(from, from.heading(), out, 0);
                if (exit == null) continue;
                double[][] entries = toReturnPose
                        ? new double[][] {{Math.PI, 0}, {0, 0}, {Math.PI / 2, 0}, {-Math.PI / 2, 0}}
                        : new double[][] {{Math.PI, STRAIGHT_APPROACH_IN}, {Math.PI, 0}};
                for (double[] entry : entries) {
                    Pose enter = escape(to, to.heading(), entry[0], entry[1]);
                    if (enter == null) continue;
                    VisibilityGraphPlanner.Leg graph =
                            VisibilityGraphPlanner.planPath(exit, enter, turnArea, obstacles, turnClearance);
                    if (graph == null) continue;
                    List<Pose> points = new ArrayList<>();
                    points.add(from);
                    for (Pose p : graph.waypoints) addDistinct(points, p);
                    addDistinct(points, to);
                    if (out == 0 && entry[0] == Math.PI) {
                        List<Segment> smooth = chain(points, from.heading(), to.heading());
                        if (smooth != null && allFit(smooth, obstacles)) {
                            best = cheapest(best, new Leg(smooth, Reroute.SPLINE, 0));
                        } else if (smooth != null && forceSplineOnly && allFit(smooth, Collections.<Obstacle>emptyList())) {
                            return new Leg(smooth, Reroute.FORCED_SPLINE, 0);
                        }
                    }
                    best = cheapest(best, polyline(points, from, exit, enter, to, toReturnPose));
                    if (best != null && entry[1] > 0) break;
                }
            }
            return best;
        }

        // Straight hops with a turn in place at each corner. The hop out of the start and the one into the end keep
        // those ends' headings. A graph hop ending at a ball's stop itself turns to the stop's heading on the way,
        // which the graph's clearance allows at any heading; one ending at the return pose keeps its own heading.
        // Runs of graph hops are then rounded into curves where that stays clear.
        private Leg polyline(List<Pose> points, Pose from, Pose exit, Pose enter, Pose to, boolean turnAtEnd) {
            List<Segment> hops = new ArrayList<>();
            int last = points.size() - 1;
            boolean[] graphHop = new boolean[last + 1];
            for (int i = 1; i <= last; i++) {
                Pose a = points.get(i - 1);
                Pose b = points.get(i);
                if (i == 1 && b.distance(exit) == 0 && exit.distance(from) > 0) {
                    hops.add(Segment.turning(from.heading(), from.heading(), a, b));
                } else if (i == last && a.distance(enter) == 0 && enter.distance(to) > 0) {
                    hops.add(Segment.turning(to.heading(), to.heading(), a, b));
                } else if (i == last && !turnAtEnd) {
                    hops.add(Segment.turning(Math.atan2(b.y() - a.y(), b.x() - a.x()), to.heading(), a, b));
                } else {
                    hops.add(Segment.tangent(a, b));
                    graphHop[i] = true;
                }
            }
            if (!allFit(hops, obstacles)) return null;
            hops = roundCorners(points, hops, graphHop);
            double turns = 0;
            double heading = from.heading();
            for (Segment s : hops) {
                turns += Math.abs(Angle.normalizeSigned(s.startHeading() - heading));
                heading = s.endHeading();
            }
            // Into the return pose the last hop keeps its heading, and the plan ends with a turn in place there.
            if (turnAtEnd) turns += Math.abs(Angle.normalizeSigned(to.heading() - heading));
            return new Leg(hops, Reroute.POLYLINE, turns);
        }

        /** Replaces each run of graph hops, longest first, with one curve through their points where {@link #turnSafe} allows. */
        private List<Segment> roundCorners(List<Pose> points, List<Segment> hops, boolean[] graphHop) {
            List<Segment> out = new ArrayList<>();
            int last = points.size() - 1;
            int i = 1;
            while (i <= last) {
                List<Segment> curve = null;
                int end = i;
                if (graphHop[i]) {
                    int runEnd = i;
                    while (runEnd < last && graphHop[runEnd + 1]) runEnd++;
                    for (end = runEnd; end > i && curve == null; end--) {
                        curve = turnSafe(points.subList(i - 1, end + 1));
                    }
                    end++;
                }
                if (curve == null) {
                    out.add(hops.get(i - 1));
                    i++;
                } else {
                    out.addAll(curve);
                    i = end + 1;
                }
            }
            return out;
        }

        // A curve through the points, leaving and arriving along its first and last hops, kept only if the robot could
        // turn on the spot all along it: the follower's heading lags on a curve, and no heading may reach a rail.
        private List<Segment> turnSafe(List<Pose> run) {
            Pose a = run.get(0);
            Pose b = run.get(1);
            Pose y = run.get(run.size() - 2);
            Pose z = run.get(run.size() - 1);
            List<Segment> curve = chain(run, Math.atan2(b.y() - a.y(), b.x() - a.x()),
                    Math.atan2(z.y() - y.y(), z.x() - y.x()));
            if (curve == null || !allFit(curve, obstacles)) return null;
            for (Segment s : curve) {
                double[] xs = s.xs();
                double[] ys = s.ys();
                for (int k = 0; k < xs.length; k++) {
                    if (!robot.canTurnAt(xs[k], ys[k], area, obstacles, gap)) return null;
                }
            }
            return curve;
        }

        /**
         * The first point at least {@code minDistance} from {@code at} in the direction {@code offset} from
         * {@code heading} (0 ahead, pi behind) where the robot can turn, keeping that heading all the way; null if the
         * footprint stops fitting first or there is none within the escape range.
         */
        private Pose escape(Pose at, double heading, double offset, double minDistance) {
            double ux = Math.cos(heading + offset);
            double uy = Math.sin(heading + offset);
            for (double d = 0; d <= MAX_ESCAPE_IN; d += ESCAPE_STEP_IN) {
                double x = at.x() + d * ux;
                double y = at.y() + d * uy;
                if (!robot.fits(x, y, heading, area, obstacles, gap)) return null;
                if (d >= minDistance && robot.canTurnAt(x, y, area, obstacles, gap)) return new Pose(x, y, heading);
            }
            return null;
        }

        private boolean allFit(List<Segment> segments, List<Obstacle> against) {
            for (Segment s : segments) {
                if (!s.fits(robot, area, against, gap)) return false;
            }
            return true;
        }

        private Leg hitting(Leg leg, Ball ball) {
            return leg != null && swept(leg.segments, ball) ? leg : null;
        }

        // The ball crosses the line of the intake from in front of it to behind it, within the intake's width and
        // nearly head-on (within about 37 degrees), so a robot swinging round onto it doesn't count.
        private boolean swept(List<Segment> intake, Ball b) {
            double half = robot.intakeWidthIn / 2;
            double reach = robot.intakeOffsetIn;
            for (Segment s : intake) {
                double[] xs = s.xs();
                double[] ys = s.ys();
                double[] hs = s.headings();
                double aheadBefore = Double.NaN;
                double sideBefore = 0;
                for (int i = 0; i < xs.length; i++) {
                    double c = Math.cos(hs[i]);
                    double sn = Math.sin(hs[i]);
                    double dx = b.x - (xs[i] + reach * c);
                    double dy = b.y - (ys[i] + reach * sn);
                    double ahead = dx * c + dy * sn;
                    double side = -dx * sn + dy * c;
                    if (!Double.isNaN(aheadBefore) && aheadBefore > HIT_TOLERANCE_IN && ahead <= HIT_TOLERANCE_IN
                            && Math.abs(side - sideBefore) <= HEAD_ON * (aheadBefore - ahead)) {
                        double f = (aheadBefore - HIT_TOLERANCE_IN) / (aheadBefore - ahead);
                        if (Math.abs(sideBefore + f * (side - sideBefore)) <= half + HIT_TOLERANCE_IN) return true;
                    }
                    aheadBefore = ahead;
                    sideBefore = side;
                }
            }
            return false;
        }

    }

    /** Where across the intake the planner aims at a ball of {@code radius}, in inches left of its middle: the middle first, then alternately further out. */
    static double[] aimSides(RobotShape robot, double radius) {
        double max = robot.intakeWidthIn / 2 - radius - SIDE_MARGIN_IN;
        int n = max > 0 ? (int) Math.floor(max / SIDE_STEP_IN + 1e-9) : 0;
        double[] sides = new double[1 + 2 * n];
        for (int i = 1; i <= n; i++) {
            sides[2 * i - 1] = i * SIDE_STEP_IN;
            sides[2 * i] = -i * SIDE_STEP_IN;
        }
        return sides;
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

    /** A cubic from {@code a} leaving at {@code headingA} to {@code b} arriving at {@code headingB}, its handles {@code reach} of the way; null if the ends coincide. */
    private static Segment hermite(Pose a, double headingA, Pose b, double headingB, double reach) {
        double chord = a.distance(b);
        if (chord < COINCIDENT_IN) return null;
        reach *= chord;
        return Segment.tangent(new Pose(a.x(), a.y(), 0),
                new Pose(a.x() + reach * Math.cos(headingA), a.y() + reach * Math.sin(headingA), 0),
                new Pose(b.x() - reach * Math.cos(headingB), b.y() - reach * Math.sin(headingB), 0),
                new Pose(b.x(), b.y(), 0));
    }

    /** Cubics through every point, tangent at each to the line through its neighbours, and at the ends to the given headings. */
    private static List<Segment> chain(List<Pose> points, double startHeading, double endHeading) {
        int n = points.size();
        double[] headings = new double[n];
        headings[0] = startHeading;
        headings[n - 1] = endHeading;
        for (int i = 1; i < n - 1; i++) {
            Pose before = points.get(i - 1);
            Pose after = points.get(i + 1);
            headings[i] = Math.atan2(after.y() - before.y(), after.x() - before.x());
        }
        List<Segment> out = new ArrayList<>();
        for (int i = 1; i < n; i++) {
            Segment s = hermite(points.get(i - 1), headings[i - 1], points.get(i), headings[i], 1.0 / 3);
            if (s == null) return null;
            out.add(s);
        }
        return out;
    }

    private static Leg cheapest(Leg a, Leg b) {
        if (a == null) return b;
        if (b == null) return a;
        return b.cost() < a.cost() ? b : a;
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
