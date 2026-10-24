package org.firstinspires.ftc.teamcode.architecture.planner;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Picks the order to collect a few balls: tries every order (n! of them, so keep n small) and keeps the shortest,
 * scored on the sampled length of the drive {@link RoutePathBuilder} will make for it rather than on straight hops
 * (a greedy nearest-first pick can lock in a bad first ball). The robot's whole footprint stays inside the caller's
 * area and clear of the obstacles, and its centre passes over each ball it must aim at. A ball the route can't reach
 * is dropped and reported rather than voiding the plan.
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
        /** Sampled drive length of the robot's centre, in inches. */
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

    /** Null if the robot may stand at {@code p} (its heading included), otherwise why it may not. */
    public static String poseProblem(Pose p, RobotShape robot, Region area, List<Obstacle> obstacles, double gapIn) {
        String problem = robot.footprintProblem(p.x(), p.y(), p.heading(), area, obstacles, gapIn);
        return problem == null ? null : String.format("(%.1f, %.1f) facing %.0f deg puts the robot %s",
                p.x(), p.y(), Math.toDegrees(p.heading()), problem);
    }

    /**
     * Null if the robot can stand centred on a ball at (x, y) at some heading, with its footprint clear, as the
     * route passes over it; otherwise why it can't.
     */
    public static String ballProblem(double x, double y, RobotShape robot, Region area,
                                     List<Obstacle> obstacles, double gapIn) {
        if (!area.contains(x, y)) return "outside " + area;
        for (int i = 0; i < RoutePathBuilder.HEADING_STEPS; i++) {
            double heading = 2 * Math.PI * i / RoutePathBuilder.HEADING_STEPS;
            if (robot.fits(x, y, heading, area, obstacles, gapIn)) return null;
        }
        return "out of reach: every heading over it puts the robot over a wall, the centre line or an obstacle";
    }

    /**
     * Throws if {@code start} or {@code returnPose} fails {@link #poseProblem}, so check that first for a pose that
     * comes from the robot rather than from configuration. {@code gapIn} is the least room between the footprint and
     * an obstacle; {@code area} is where the footprint must stay. The balls, the obstacle list and the
     * {@link VisibilityGraphPlanner} settings are copied here: later edits don't reach this search.
     */
    public static Search search(Pose start, Ball[] balls, Pose returnPose, Region area, List<Obstacle> obstacles,
                                RobotShape robot, double gapIn, boolean forceSplineOnly) {
        return new Search(start, balls, returnPose, area, obstacles, robot, gapIn, forceSplineOnly);
    }

    /** {@link #search} run to the end in one call. */
    public static Route findOptimalRoute(Pose start, Ball[] balls, Pose returnPose, Region area,
                                         List<Obstacle> obstacles, RobotShape robot, double gapIn,
                                         boolean forceSplineOnly) {
        Search search = search(start, balls, returnPose, area, obstacles, robot, gapIn, forceSplineOnly);
        while (search.route == null) search.advance();
        return search.route;
    }

    /**
     * Each {@link #step()} shapes visit orders until {@link VisibilityGraphPlanner#routeSearchSliceMs} has passed, so
     * it overruns the slice by at most one shape() or the final Pedro path build. A new request is a new Search.
     */
    public static final class Search {
        private final Pose start;
        private final Pose returnPose;
        private final Ball[] reachable;
        private final List<Dropped> dropped = new ArrayList<>();
        private final RoutePathBuilder.Planner planner;
        private final List<Integer> indices = new ArrayList<>();
        private int size;
        private List<int[]> orders;
        private double[] bound;
        private Integer[] byBound;
        private int next;
        private Route best;
        private Route route;
        private RoutePathBuilder.Plan plan;

        private Search(Pose start, Ball[] balls, Pose returnPose, Region area, List<Obstacle> obstacles,
                       RobotShape robot, double gapIn, boolean forceSplineOnly) {
            if (!(gapIn >= 0)) throw new IllegalArgumentException(String.format("obstacle gap %.1f in must be >= 0", gapIn));
            if (balls.length > MAX_BALLS) {
                throw new IllegalArgumentException(balls.length + " balls: the order search is n!, so at most " + MAX_BALLS);
            }
            obstacles = Collections.unmodifiableList(new ArrayList<>(obstacles));
            String startProblem = poseProblem(start, robot, area, obstacles, gapIn);
            if (startProblem != null) throw new IllegalArgumentException("start " + startProblem);
            String returnProblem = poseProblem(returnPose, robot, area, obstacles, gapIn);
            if (returnProblem != null) throw new IllegalArgumentException("return pose " + returnProblem);
            this.start = start;
            this.returnPose = returnPose;

            List<Ball> kept = new ArrayList<>();
            List<Ball> copies = new ArrayList<>();
            for (Ball given : balls) {
                Ball ball = new Ball(given.x, given.y, given.radius);
                if (!(Double.isFinite(ball.x) && Double.isFinite(ball.y))) {
                    throw new IllegalArgumentException("ball position is not finite: " + ball);
                }
                String problem = placementProblem(ball, start, copies, area, obstacles, robot, gapIn);
                if (problem == null) {
                    kept.add(given);
                    copies.add(ball);
                } else {
                    dropped.add(new Dropped(given, problem));
                }
            }

            reachable = kept.toArray(new Ball[0]);
            planner = new RoutePathBuilder.Planner(start, copies.toArray(new Ball[0]), returnPose, area, obstacles,
                    robot, gapIn, forceSplineOnly);
            for (int i = 0; i < reachable.length; i++) indices.add(i);
            size = reachable.length;
        }

        /** Runs this loop's slice; true once {@link #route()} and {@link #plan()} are ready. */
        public boolean step() {
            double sliceMs = VisibilityGraphPlanner.routeSearchSliceMs;
            if (!(sliceMs >= 0)) {
                throw new IllegalArgumentException(String.format("routeSearchSliceMs %.1f must be >= 0", sliceMs));
            }
            long t0 = System.nanoTime();
            while (plan == null) {
                advance();
                if (System.nanoTime() - t0 >= sliceMs * 1e6) break;
            }
            return plan != null;
        }

        public boolean done() {
            return plan != null;
        }

        /** The balls in its order and dropped list are the caller's own, not the copies the search planned. */
        public Route route() {
            if (plan == null) throw new IllegalStateException("the route search hasn't finished: step() it until it returns true");
            return route;
        }

        public RoutePathBuilder.Plan plan() {
            route();
            return plan;
        }

        private void advance() {
            if (route != null) {
                plan = RoutePathBuilder.build(route);
                return;
            }
            if (orders == null) {
                orders = orders(indices, size);
                final double[] bounds = new double[orders.size()];
                byBound = new Integer[orders.size()];
                for (int i = 0; i < bounds.length; i++) {
                    bounds[i] = planner.lowerBound(orders.get(i));
                    byBound[i] = i;
                }
                Arrays.sort(byBound, (p, q) -> Double.compare(bounds[p], bounds[q]));
                bound = bounds;
                next = 0;
                best = null;
                return;
            }
            if (next < byBound.length && (best == null || bound[byBound[next]] < best.length)) {
                int[] order = orders.get(byBound[next++]);
                RoutePathBuilder.Shape shape = planner.shape(order);
                if (shape != null && (best == null || shape.length() < best.length)) {
                    best = new Route(start, returnPose, ordered(order), dropped, shape);
                }
                return;
            }
            if (best != null) {
                List<Ball> visited = Arrays.asList(best.order);
                for (Ball ball : reachable) {
                    if (!visited.contains(ball)) dropped.add(new Dropped(ball, "no drivable order that also visits the others"));
                }
                route = best;
                return;
            }
            if (size == 0) {
                throw new IllegalArgumentException(String.format(
                        "no drivable path for the robot from the start (%.1f, %.1f) to the return pose (%.1f, %.1f)",
                        start.x(), start.y(), returnPose.x(), returnPose.y()));
            }
            // Most balls first: only when no order through all of them is drivable is one left out.
            size--;
            orders = null;
        }

        private Ball[] ordered(int[] order) {
            Ball[] ordered = new Ball[order.length];
            for (int i = 0; i < order.length; i++) ordered[i] = reachable[order[i]];
            return ordered;
        }
    }

    private static String placementProblem(Ball ball, Pose start, List<Ball> kept, Region area,
                                           List<Obstacle> obstacles, RobotShape robot, double gapIn) {
        if (robot.covers(start, ball.x, ball.y)) return "under the robot at the start";
        String problem = ballProblem(ball.x, ball.y, robot, area, obstacles, gapIn);
        if (problem != null) return problem;
        for (Ball other : kept) {
            if (Math.hypot(ball.x - other.x, ball.y - other.y) < RoutePathBuilder.COINCIDENT_IN) {
                return "the same ball as " + other;
            }
        }
        return null;
    }

    private static List<int[]> orders(List<Integer> indices, int size) {
        List<int[]> orders = new ArrayList<>();
        for (List<Integer> subset : subsets(indices, size)) {
            for (int[] perm : permutations(subset.size())) {
                int[] order = new int[perm.length];
                for (int i = 0; i < perm.length; i++) order[i] = subset.get(perm[i]);
                orders.add(order);
            }
        }
        return orders;
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
