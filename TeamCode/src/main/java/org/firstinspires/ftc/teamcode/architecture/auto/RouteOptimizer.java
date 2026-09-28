package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.curves.Curve;

import java.util.ArrayList;
import java.util.List;

/**
 * Picks the order to collect a few balls: tries every order (n! of them, so keep n small) and keeps the
 * shortest drive, scored on the paths {@link RoutePathBuilder} will build rather than on straight hops (a greedy
 * nearest-first pick can lock in a bad first ball). Every leg's visibility-graph detour is planned too, for
 * RoutePathBuilder's reroutes.
 */
public final class RouteOptimizer {
    private RouteOptimizer() {}

    public static final class Route {
        /** Empty when there are no balls; the route is then just the return. */
        public final Ball[] order;
        /** Start to each ball in order, then the last ball (or the start) to the return pose. */
        public final List<VisibilityGraphPlanner.Leg> legs;
        /** Estimated drive length in inches. */
        public final double length;

        Route(Ball[] order, List<VisibilityGraphPlanner.Leg> legs, double length) {
            this.order = order;
            this.legs = legs;
            this.length = length;
        }
    }

    /** Null if no order reaches every ball and the return pose. */
    public static Route findOptimalRoute(Pose start, Ball[] balls, Pose returnPose, List<Obstacle> obstacles,
                                         double clearance, double intakeWidthIn) {
        int n = balls.length;
        // A leg depends only on its endpoints, so each pair is planned once rather than once per order.
        // Index n stands for the start.
        VisibilityGraphPlanner.Leg[][] toBall = new VisibilityGraphPlanner.Leg[n + 1][n];
        VisibilityGraphPlanner.Leg[] toReturn = new VisibilityGraphPlanner.Leg[n + 1];
        for (int from = 0; from <= n; from++) {
            Pose fromPose = from == n ? start : balls[from].toPose();
            for (int to = 0; to < n; to++) {
                if (to != from) {
                    toBall[from][to] = VisibilityGraphPlanner.planPath(fromPose, balls[to].toPose(), obstacles, clearance);
                }
            }
            toReturn[from] = VisibilityGraphPlanner.planPath(fromPose, returnPose, obstacles, clearance);
        }

        Route best = null;
        for (int[] order : permutations(n)) {
            List<VisibilityGraphPlanner.Leg> legs = new ArrayList<>();
            Ball[] ordered = new Ball[n];
            int at = n;
            for (int i = 0; i < n; i++) {
                legs.add(toBall[at][order[i]]);
                ordered[i] = balls[order[i]];
                at = order[i];
            }
            legs.add(toReturn[at]);
            if (legs.contains(null)) continue;

            double length = driveLength(start, ordered, returnPose, obstacles, clearance, intakeWidthIn, legs);
            if (best == null || length < best.length) best = new Route(ordered, legs, length);
        }
        return best;
    }

    private static double driveLength(Pose start, Ball[] order, Pose returnPose, List<Obstacle> obstacles,
                                      double clearance, double intakeWidthIn, List<VisibilityGraphPlanner.Leg> legs) {
        int n = order.length;
        Pose intakeEnd = start;
        double intakeLength = 0;
        if (n > 0) {
            Curve intake = IntakeCurvePlanner.buildThroughCurve(start,
                    IntakeCurvePlanner.essentialStops(start, order, intakeWidthIn));
            if (IntakeCurvePlanner.collides(intake, obstacles, clearance)) {
                for (int i = 0; i < n; i++) intakeLength += legs.get(i).length;
            } else {
                intakeLength = intake.length();
            }
            intakeEnd = order[n - 1].toPose();
        }

        double returnLength = Obstacle.anyBlocks(obstacles,
                intakeEnd.x(), intakeEnd.y(), returnPose.x(), returnPose.y(), clearance)
                ? legs.get(n).length
                : intakeEnd.distance(returnPose);
        return intakeLength + returnLength;
    }

    private static List<int[]> permutations(int n) {
        int[] indices = new int[n];
        for (int i = 0; i < n; i++) indices[i] = i;
        List<int[]> result = new ArrayList<>();
        permute(indices, 0, result);
        return result;
    }

    private static void permute(int[] indices, int k, List<int[]> result) {
        if (k == indices.length) {
            result.add(indices.clone());
            return;
        }
        for (int i = k; i < indices.length; i++) {
            swap(indices, k, i);
            permute(indices, k + 1, result);
            swap(indices, k, i);
        }
    }

    private static void swap(int[] a, int i, int j) {
        int tmp = a[i];
        a[i] = a[j];
        a[j] = tmp;
    }
}
