package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Turns a visit order into the two drives of a {@link RouteOptimizer.Route}: the intake leg, tangent to a curve
 * through the essential stops, then a return that turns back to the return pose's heading. Each leg tries a
 * smooth curve first, then one spline through the visibility-graph detour waypoints, then straight hops through
 * them; a candidate is used only if every sample of it stays inside the keep-in region and clear of every
 * obstacle.
 */
public final class RoutePathBuilder {
    /** Points closer than this are one point: Pedro rejects a zero-length line, and a tiny hop has no usable heading. */
    public static final double COINCIDENT_IN = 1.0;

    private RoutePathBuilder() {}

    public enum Reroute { NONE, SPLINE, POLYLINE, FORCED_SPLINE }

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

        Shape(List<Segment> intake, Reroute intakeReroute, List<Segment> back, Reroute returnReroute,
              Pose intakeEnd, int essentialStops) {
            this.intake = intake;
            this.intakeReroute = intakeReroute;
            this.back = back;
            this.returnReroute = returnReroute;
            this.intakeEnd = intakeEnd;
            this.essentialStops = essentialStops;
            this.intakeLength = totalLength(intake);
            this.returnLength = totalLength(back);
        }

        double length() {
            return intakeLength + returnLength;
        }
    }

    public static final class Plan {
        /** Null when there are no balls. */
        public final Path intake;
        /** Null when the intake leg already ends at the return position. */
        public final Path back;
        public final Pose intakeEnd;
        public final int essentialStops;
        public final Reroute intakeReroute;
        public final Reroute returnReroute;
        /** Sampled lengths in inches. */
        public final double intakeLength;
        public final double returnLength;

        Plan(Path intake, Path back, Shape shape) {
            this.intake = intake;
            this.back = back;
            this.intakeEnd = shape.intakeEnd;
            this.essentialStops = shape.essentialStops;
            this.intakeReroute = shape.intakeReroute;
            this.returnReroute = shape.returnReroute;
            this.intakeLength = shape.intakeLength;
            this.returnLength = shape.returnLength;
        }
    }

    /** The Pedro paths for exactly the geometry the route was scored on. */
    public static Plan build(RouteOptimizer.Route route) {
        Shape shape = route.shape;
        Path intake = null;
        double intakeEndHeading = route.start.heading();
        if (!shape.intake.isEmpty()) {
            List<Path> parts = new ArrayList<>();
            for (Segment s : shape.intake) parts.add(Paths.path(s.toCurve()).tangent());
            intake = parts.size() == 1 ? parts.get(0) : Paths.path(parts.toArray(new Path[0]));
            intakeEndHeading = shape.intake.get(shape.intake.size() - 1).endHeading();
        }

        Path back = null;
        if (!shape.back.isEmpty()) {
            double returnHeading = route.returnPose.heading();
            List<Path> parts = new ArrayList<>();
            int last = shape.back.size() - 1;
            if (last == 0) {
                parts.add(Paths.path(shape.back.get(0).toCurve()).linear(intakeEndHeading, returnHeading));
            } else {
                for (int i = 0; i < last; i++) parts.add(Paths.path(shape.back.get(i).toCurve()).tangent());
                Segment hop = shape.back.get(last);
                parts.add(Paths.path(hop.toCurve()).linear(hop.endHeading(), returnHeading));
            }
            back = parts.size() == 1 ? parts.get(0) : Paths.path(parts.toArray(new Path[0]));
        }
        return new Plan(intake, back, shape);
    }

    /**
     * Null if no candidate for a leg stays clear. {@code legs} are the visibility-graph legs from the start to each
     * ball in order, then from the last ball (or the start) to the return pose. Neighbouring balls in
     * {@code order}, and the start and the first ball, must be at least {@link #COINCIDENT_IN} apart.
     * {@code forceSplineOnly} keeps a detour spline that clips an obstacle (still never one that leaves the
     * region): testing only.
     */
    static Shape shape(Pose start, Ball[] order, List<VisibilityGraphPlanner.Leg> legs, Pose returnPose,
                       Region keepIn, List<Obstacle> obstacles, double clearance, double intakeWidthIn,
                       boolean forceSplineOnly) {
        int n = order.length;
        List<Segment> intake = Collections.emptyList();
        Reroute intakeReroute = Reroute.NONE;
        Pose intakeEnd = start;
        int essentialStops = 0;
        if (n > 0) {
            List<Pose> stops = IntakeCurvePlanner.essentialStops(start, order, intakeWidthIn);
            essentialStops = stops.size();
            List<Pose> through = new ArrayList<>();
            through.add(start);
            through.addAll(stops);
            Segment direct = IntakeCurvePlanner.through(through);
            if (direct.staysClear(keepIn, obstacles, clearance)) {
                intake = Collections.singletonList(direct);
            } else {
                // A detour stops at every ball: the few inches an intake sweep saves aren't worth the risk here.
                List<Pose> waypoints = new ArrayList<>();
                waypoints.add(start);
                for (int i = 0; i < n; i++) appendAfterFirst(waypoints, legs.get(i).waypoints);
                waypoints = withoutCoincident(waypoints);
                Detour detour = detour(waypoints, keepIn, obstacles, clearance, forceSplineOnly);
                if (detour == null) return null;
                intake = detour.segments;
                intakeReroute = detour.reroute;
            }
            intakeEnd = order[n - 1].toPose();
        }

        List<Segment> back = Collections.emptyList();
        Reroute returnReroute = Reroute.NONE;
        if (intakeEnd.distance(returnPose) >= COINCIDENT_IN) {
            Segment direct = new Segment(intakeEnd, returnPose);
            if (direct.staysClear(keepIn, obstacles, clearance)) {
                back = Collections.singletonList(direct);
            } else {
                List<Pose> waypoints = new ArrayList<>();
                waypoints.add(intakeEnd);
                appendAfterFirst(waypoints, legs.get(n).waypoints);
                waypoints = withoutCoincident(waypoints);
                Detour detour = detour(waypoints, keepIn, obstacles, clearance, false);
                if (detour == null) return null;
                back = detour.segments;
                returnReroute = detour.reroute;
            }
        }
        return new Shape(intake, intakeReroute, back, returnReroute, intakeEnd, essentialStops);
    }

    private static final class Detour {
        final List<Segment> segments;
        final Reroute reroute;

        Detour(List<Segment> segments, Reroute reroute) {
            this.segments = segments;
            this.reroute = reroute;
        }
    }

    private static Detour detour(List<Pose> waypoints, Region keepIn, List<Obstacle> obstacles, double clearance,
                                 boolean forceSplineOnly) {
        Segment spline = IntakeCurvePlanner.through(waypoints);
        if (spline.staysClear(keepIn, obstacles, clearance)) {
            return new Detour(Collections.singletonList(spline), Reroute.SPLINE);
        }
        if (forceSplineOnly && spline.staysInside(keepIn)) {
            return new Detour(Collections.singletonList(spline), Reroute.FORCED_SPLINE);
        }
        List<Segment> hops = new ArrayList<>();
        for (int i = 1; i < waypoints.size(); i++) {
            Segment hop = new Segment(waypoints.get(i - 1), waypoints.get(i));
            if (!hop.staysClear(keepIn, obstacles, clearance)) return null;
            hops.add(hop);
        }
        return new Detour(hops, Reroute.POLYLINE);
    }

    // Each leg starts where the previous one ended, so its first waypoint is already in the list.
    private static void appendAfterFirst(List<Pose> into, List<Pose> leg) {
        into.addAll(leg.subList(1, leg.size()));
    }

    /**
     * Drops each waypoint within {@link #COINCIDENT_IN} of the one kept before it, except the last, which is a
     * target the leg must reach: it replaces the kept points it is too close to instead (never the first).
     */
    private static List<Pose> withoutCoincident(List<Pose> waypoints) {
        List<Pose> kept = new ArrayList<>();
        kept.add(waypoints.get(0));
        int last = waypoints.size() - 1;
        for (int i = 1; i < last; i++) {
            Pose p = waypoints.get(i);
            if (p.distance(kept.get(kept.size() - 1)) >= COINCIDENT_IN) kept.add(p);
        }
        Pose target = waypoints.get(last);
        while (kept.size() > 1 && target.distance(kept.get(kept.size() - 1)) < COINCIDENT_IN) {
            kept.remove(kept.size() - 1);
        }
        kept.add(target);
        if (target.distance(kept.get(0)) < COINCIDENT_IN && kept.size() == 2) {
            throw new IllegalStateException("leg from " + waypoints.get(0) + " to " + waypoints.get(last)
                    + " has coincident ends; the route should have dropped that target");
        }
        return kept;
    }

    private static double totalLength(List<Segment> segments) {
        double total = 0;
        for (Segment s : segments) total += s.length();
        return total;
    }
}
