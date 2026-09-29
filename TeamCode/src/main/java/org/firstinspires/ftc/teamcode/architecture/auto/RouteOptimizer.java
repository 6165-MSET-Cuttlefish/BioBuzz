package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Picks the order to collect a few balls: tries every order (n! of them, so keep n small) and keeps the
 * shortest drive, scored on the sampled length of the geometry {@link RoutePathBuilder} will drive rather than on
 * straight hops (a greedy nearest-first pick can lock in a bad first ball). Everything stays inside the caller's
 * keep-in region. A ball the route can't reach is dropped and reported rather than voiding the plan.
 */
public final class RouteOptimizer {
    public static final int MAX_BALLS = 6;

    private RouteOptimizer() {}

    public static final class Dropped {
        public final Ball ball;
        public final String reason;

        Dropped(Ball ball, String reason) {
            this.ball = ball;
            this.reason = reason;
        }
    }

    public static final class Route {
        public final Pose start;
        public final Pose returnPose;
        /** The balls in visit order; empty when there are none to visit, and the route is then just the return. */
        public final Ball[] order;
        /** Input balls not in {@link #order}, each with why. */
        public final List<Dropped> dropped;
        /** Sampled drive length in inches. */
        public final double length;
        final RoutePathBuilder.Shape shape;

        Route(Pose start, Pose returnPose, Ball[] order, List<Dropped> dropped, RoutePathBuilder.Shape shape) {
            this.start = start;
            this.returnPose = returnPose;
            this.order = order;
            this.dropped = dropped;
            this.shape = shape;
            this.length = shape.length();
        }
    }

    /** Null if the robot's centre may stand at {@code p}, otherwise why it may not. */
    public static String poseProblem(Pose p, Region keepIn, List<Obstacle> obstacles, double clearance) {
        if (!keepIn.contains(p)) return String.format("(%.1f, %.1f) is outside the keep-in region %s", p.x(), p.y(), keepIn);
        Obstacle o = keepOutContaining(p.x(), p.y(), obstacles, clearance);
        if (o != null) {
            return String.format("(%.1f, %.1f) is inside the keep-out of the obstacle at (%.1f, %.1f)",
                    p.x(), p.y(), o.x, o.y);
        }
        return null;
    }

    /**
     * Throws if {@code start} or {@code returnPose} fails {@link #poseProblem}, so check that first for a pose
     * that comes from the robot rather than from configuration.
     */
    public static Route findOptimalRoute(Pose start, Ball[] balls, Pose returnPose, Region keepIn,
                                         List<Obstacle> obstacles, double clearance, double intakeWidthIn,
                                         boolean forceSplineOnly) {
        if (!(clearance >= 0) || !(intakeWidthIn >= 0)) {
            throw new IllegalArgumentException(String.format(
                    "clearance (%.1f) and intake width (%.1f) must be >= 0", clearance, intakeWidthIn));
        }
        if (balls.length > MAX_BALLS) {
            throw new IllegalArgumentException(balls.length + " balls: the order search is n!, so at most " + MAX_BALLS);
        }
        String startProblem = poseProblem(start, keepIn, obstacles, clearance);
        if (startProblem != null) throw new IllegalArgumentException("start " + startProblem);
        String returnProblem = poseProblem(returnPose, keepIn, obstacles, clearance);
        if (returnProblem != null) throw new IllegalArgumentException("return pose " + returnProblem);

        List<Dropped> dropped = new ArrayList<>();
        List<Ball> candidates = new ArrayList<>();
        for (Ball ball : balls) {
            if (!(Double.isFinite(ball.x) && Double.isFinite(ball.y))) {
                throw new IllegalArgumentException("ball position is not finite: " + ball);
            }
            String problem = placementProblem(ball, start, candidates, keepIn, obstacles, clearance);
            if (problem == null) candidates.add(ball);
            else dropped.add(new Dropped(ball, problem));
        }

        // A leg depends only on its endpoints, so each pair is planned once rather than once per order.
        // Index k stands for the start.
        int k = candidates.size();
        VisibilityGraphPlanner.Leg[][] toBall = new VisibilityGraphPlanner.Leg[k + 1][k];
        VisibilityGraphPlanner.Leg[] toReturn = new VisibilityGraphPlanner.Leg[k + 1];
        for (int from = 0; from <= k; from++) {
            Pose fromPose = from == k ? start : candidates.get(from).toPose();
            for (int to = 0; to < k; to++) {
                if (to != from) {
                    toBall[from][to] = VisibilityGraphPlanner.planPath(fromPose, candidates.get(to).toPose(),
                            keepIn, obstacles, clearance);
                }
            }
            toReturn[from] = VisibilityGraphPlanner.planPath(fromPose, returnPose, keepIn, obstacles, clearance);
        }
        if (toReturn[k] == null) {
            throw new IllegalArgumentException(String.format(
                    "no obstacle-free path inside the keep-in region from the start (%.1f, %.1f) to the return pose (%.1f, %.1f)",
                    start.x(), start.y(), returnPose.x(), returnPose.y()));
        }

        List<Integer> reachable = new ArrayList<>();
        for (int i = 0; i < k; i++) {
            if (toBall[k][i] == null) dropped.add(new Dropped(candidates.get(i), "no obstacle-free path to it from the start"));
            else if (toReturn[i] == null) dropped.add(new Dropped(candidates.get(i), "no obstacle-free path from it to the return pose"));
            else reachable.add(i);
        }

        // Most balls first: only when no order through all of them is drivable is one left out.
        for (int size = reachable.size(); size >= 0; size--) {
            Route best = null;
            for (List<Integer> subset : subsets(reachable, size)) {
                for (int[] perm : permutations(subset.size())) {
                    List<VisibilityGraphPlanner.Leg> legs = new ArrayList<>();
                    Ball[] ordered = new Ball[perm.length];
                    int at = k;
                    for (int i = 0; i < perm.length; i++) {
                        int ball = subset.get(perm[i]);
                        legs.add(toBall[at][ball]);
                        ordered[i] = candidates.get(ball);
                        at = ball;
                    }
                    legs.add(toReturn[at]);
                    if (legs.contains(null)) continue;

                    RoutePathBuilder.Shape shape = RoutePathBuilder.shape(start, ordered, legs, returnPose, keepIn,
                            obstacles, clearance, intakeWidthIn, forceSplineOnly);
                    if (shape == null) continue;
                    if (best == null || shape.length() < best.length) {
                        best = new Route(start, returnPose, ordered, dropped, shape);
                    }
                }
            }
            if (best != null) {
                List<Ball> visited = new ArrayList<>();
                Collections.addAll(visited, best.order);
                for (int i : reachable) {
                    Ball ball = candidates.get(i);
                    if (!visited.contains(ball)) dropped.add(new Dropped(ball, "no drivable order that also visits the others"));
                }
                return best;
            }
        }
        throw new IllegalStateException(String.format(
                "no drivable return from the start (%.1f, %.1f) to the return pose (%.1f, %.1f) although a graph path exists",
                start.x(), start.y(), returnPose.x(), returnPose.y()));
    }

    private static String placementProblem(Ball ball, Pose start, List<Ball> kept, Region keepIn,
                                           List<Obstacle> obstacles, double clearance) {
        if (Math.hypot(ball.x - start.x(), ball.y - start.y()) < RoutePathBuilder.COINCIDENT_IN) {
            return "on the start pose";
        }
        if (!keepIn.contains(ball.x, ball.y)) return "outside the keep-in region " + keepIn;
        Obstacle o = keepOutContaining(ball.x, ball.y, obstacles, clearance);
        if (o != null) return String.format("inside the keep-out of the obstacle at (%.1f, %.1f)", o.x, o.y);
        for (Ball other : kept) {
            if (Math.hypot(ball.x - other.x, ball.y - other.y) < RoutePathBuilder.COINCIDENT_IN) {
                return "the same ball as " + other;
            }
        }
        return null;
    }

    private static Obstacle keepOutContaining(double x, double y, List<Obstacle> obstacles, double clearance) {
        for (Obstacle o : obstacles) {
            if (Math.hypot(x - o.x, y - o.y) <= o.radius + clearance) return o;
        }
        return null;
    }

    private static List<List<Integer>> subsets(List<Integer> items, int size) {
        List<List<Integer>> result = new ArrayList<>();
        collect(items, size, 0, new ArrayList<Integer>(), result);
        return result;
    }

    private static void collect(List<Integer> items, int size, int from, List<Integer> chosen,
                                List<List<Integer>> result) {
        if (chosen.size() == size) {
            result.add(new ArrayList<>(chosen));
            return;
        }
        for (int i = from; i < items.size(); i++) {
            chosen.add(items.get(i));
            collect(items, size, i + 1, chosen, result);
            chosen.remove(chosen.size() - 1);
        }
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
