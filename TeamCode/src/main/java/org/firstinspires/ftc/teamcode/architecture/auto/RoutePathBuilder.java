package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.bezier.BezierCurve;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds the two paths that drive a {@link RouteOptimizer.Route}: the intake leg, tangent to a curve through
 * the essential stops, then a straight return that turns back to the return pose's heading. A leg that would
 * clip an obstacle is bent around it as one spline through the route's visibility-graph waypoints or, if that
 * still clips, driven as straight hops through them.
 */
public final class RoutePathBuilder {
    private static final double SAME_SPOT_IN = 1e-6;

    private RoutePathBuilder() {}

    public enum Reroute { NONE, SPLINE, POLYLINE, FORCED_SPLINE }

    public static final class Plan {
        /** Null when there are no balls. */
        public final Path intake;
        /** Null when the intake leg already ends at the return position. */
        public final Path back;
        public final Pose intakeEnd;
        public final int essentialStops;
        public final Reroute intakeReroute;
        public final Reroute returnReroute;

        Plan(Path intake, Path back, Pose intakeEnd, int essentialStops,
             Reroute intakeReroute, Reroute returnReroute) {
            this.intake = intake;
            this.back = back;
            this.intakeEnd = intakeEnd;
            this.essentialStops = essentialStops;
            this.intakeReroute = intakeReroute;
            this.returnReroute = returnReroute;
        }
    }

    /** {@code forceSplineOnly} drives the intake spline even when it clips an obstacle: testing only. */
    public static Plan build(Pose start, RouteOptimizer.Route route, Pose returnPose, List<Obstacle> obstacles,
                             double clearance, double intakeWidthIn, boolean forceSplineOnly) {
        int n = route.order.length;

        Path intake = null;
        Pose intakeEnd = start;
        double intakeEndHeading = start.heading();
        int essentialStops = 0;
        Reroute intakeReroute = Reroute.NONE;
        if (n > 0) {
            List<Pose> stops = IntakeCurvePlanner.essentialStops(start, route.order, intakeWidthIn);
            essentialStops = stops.size();
            Curve direct = IntakeCurvePlanner.buildThroughCurve(start, stops);
            if (!IntakeCurvePlanner.collides(direct, obstacles, clearance)) {
                intake = Paths.path(direct).tangent();
                intakeEnd = stops.get(stops.size() - 1);
                intakeEndHeading = direct.derivative(1).theta();
            } else {
                // A detour stops at every ball: the few inches an intake sweep saves aren't worth the risk here.
                List<Pose> waypoints = new ArrayList<>();
                waypoints.add(start);
                for (int i = 0; i < n; i++) appendAfterFirst(waypoints, route.legs.get(i).waypoints);
                intakeEnd = waypoints.get(waypoints.size() - 1);

                BezierCurve spline = BezierCurve.through(waypoints.toArray(new Pose[0]));
                boolean clips = IntakeCurvePlanner.collides(spline, obstacles, clearance);
                if (!clips || forceSplineOnly) {
                    intake = Paths.path(spline).tangent();
                    intakeEndHeading = spline.derivative(1).theta();
                    intakeReroute = clips ? Reroute.FORCED_SPLINE : Reroute.SPLINE;
                } else {
                    intake = Paths.path(hops(waypoints).toArray(new Path[0]));
                    intakeEndHeading = hopHeading(waypoints, waypoints.size() - 2);
                    intakeReroute = Reroute.POLYLINE;
                }
            }
        }

        Path back = null;
        Reroute returnReroute = Reroute.NONE;
        boolean atReturn = intakeEnd.distance(returnPose) <= SAME_SPOT_IN;
        if (!atReturn && !Obstacle.anyBlocks(obstacles,
                intakeEnd.x(), intakeEnd.y(), returnPose.x(), returnPose.y(), clearance)) {
            back = Paths.line(intakeEnd, returnPose).linear(intakeEndHeading, returnPose.heading());
        } else if (!atReturn) {
            List<Pose> waypoints = new ArrayList<>();
            waypoints.add(intakeEnd);
            appendAfterFirst(waypoints, route.legs.get(n).waypoints);

            BezierCurve spline = BezierCurve.through(waypoints.toArray(new Pose[0]));
            if (!IntakeCurvePlanner.collides(spline, obstacles, clearance)) {
                back = Paths.path(spline).linear(intakeEndHeading, returnPose.heading());
                returnReroute = Reroute.SPLINE;
            } else {
                List<Path> hops = hops(waypoints);
                int last = hops.size() - 1;
                hops.set(last, Paths.line(waypoints.get(last), waypoints.get(last + 1))
                        .linear(hopHeading(waypoints, last), returnPose.heading()));
                back = Paths.path(hops.toArray(new Path[0]));
                returnReroute = Reroute.POLYLINE;
            }
        }

        return new Plan(intake, back, intakeEnd, essentialStops, intakeReroute, returnReroute);
    }

    // Each leg starts where the previous one ended, so its first waypoint is already in the list.
    private static void appendAfterFirst(List<Pose> into, List<Pose> leg) {
        into.addAll(leg.subList(1, leg.size()));
    }

    private static List<Path> hops(List<Pose> waypoints) {
        List<Path> hops = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            hops.add(Paths.line(waypoints.get(i), waypoints.get(i + 1)).tangent());
        }
        return hops;
    }

    private static double hopHeading(List<Pose> waypoints, int from) {
        Pose a = waypoints.get(from);
        Pose b = waypoints.get(from + 1);
        return Math.atan2(b.y() - a.y(), b.x() - a.x());
    }
}
