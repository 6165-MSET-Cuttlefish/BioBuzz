package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Picks the order to collect a few balls: tries every order (n! of them, so keep n small) and keeps the cheapest,
 * scored on the sampled length of the drive {@link RoutePathBuilder} will make plus its turns in place and its
 * stops, rather than on straight hops (a greedy nearest-first pick can lock in a bad first ball). The robot's whole
 * footprint stays inside the caller's area and clear of the obstacles, and the intake passes over each ball. A ball
 * the route can't reach is dropped and reported rather than voiding the plan.
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
     * Null if some heading puts the intake on a ball of {@code radius} at (x, y), anywhere across it the planner
     * aims, with the robot's footprint clear; otherwise why none does.
     */
    public static String ballProblem(double x, double y, double radius, RobotShape robot, Region area,
                                     List<Obstacle> obstacles, double gapIn) {
        if (!area.contains(x, y)) return "outside " + area;
        double[] sides = RoutePathBuilder.aimSides(robot, radius);
        for (int i = 0; i < RoutePathBuilder.HEADING_STEPS; i++) {
            double heading = 2 * Math.PI * i / RoutePathBuilder.HEADING_STEPS;
            for (double side : sides) {
                Pose c = robot.centreFor(x, y, heading, side);
                if (robot.fits(c.x(), c.y(), heading, area, obstacles, gapIn)) return null;
            }
        }
        return "out of the intake's reach: every heading puts the robot over a wall, the centre line or an obstacle";
    }

    /**
     * Throws if {@code start} or {@code returnPose} fails {@link #poseProblem}, so check that first for a pose
     * that comes from the robot rather than from configuration. {@code gapIn} is the least room between the
     * footprint and an obstacle; {@code area} is where the footprint must stay.
     */
    public static Route findOptimalRoute(Pose start, Ball[] balls, Pose returnPose, Region area,
                                         List<Obstacle> obstacles, RobotShape robot, double gapIn,
                                         boolean forceSplineOnly) {
        if (!(gapIn >= 0)) throw new IllegalArgumentException(String.format("obstacle gap %.1f in must be >= 0", gapIn));
        if (balls.length > MAX_BALLS) {
            throw new IllegalArgumentException(balls.length + " balls: the order search is n!, so at most " + MAX_BALLS);
        }
        String startProblem = poseProblem(start, robot, area, obstacles, gapIn);
        if (startProblem != null) throw new IllegalArgumentException("start " + startProblem);
        String returnProblem = poseProblem(returnPose, robot, area, obstacles, gapIn);
        if (returnProblem != null) throw new IllegalArgumentException("return pose " + returnProblem);

        List<Dropped> dropped = new ArrayList<>();
        List<Ball> candidates = new ArrayList<>();
        for (Ball ball : balls) {
            if (!(Double.isFinite(ball.x) && Double.isFinite(ball.y))) {
                throw new IllegalArgumentException("ball position is not finite: " + ball);
            }
            String problem = placementProblem(ball, start, candidates, area, obstacles, robot, gapIn);
            if (problem == null) candidates.add(ball);
            else dropped.add(new Dropped(ball, problem));
        }

        Ball[] reachable = candidates.toArray(new Ball[0]);
        RoutePathBuilder.Planner planner = new RoutePathBuilder.Planner(start, reachable, returnPose, area, obstacles,
                robot, gapIn, forceSplineOnly);
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < reachable.length; i++) indices.add(i);

        // Most balls first: only when no order through all of them is drivable is one left out.
        for (int size = reachable.length; size >= 0; size--) {
            List<int[]> orders = new ArrayList<>();
            for (List<Integer> subset : subsets(indices, size)) {
                for (int[] perm : permutations(subset.size())) {
                    int[] order = new int[perm.length];
                    for (int i = 0; i < perm.length; i++) order[i] = subset.get(perm[i]);
                    orders.add(order);
                }
            }
            // Likely-good orders first, so the budget cuts the rest short.
            final double[] tour = new double[orders.size()];
            Integer[] byTour = new Integer[orders.size()];
            for (int i = 0; i < tour.length; i++) {
                tour[i] = tourLength(start, reachable, orders.get(i), returnPose);
                byTour[i] = i;
            }
            Arrays.sort(byTour, (p, q) -> Double.compare(tour[p], tour[q]));
            Route best = null;
            double bestCost = Double.POSITIVE_INFINITY;
            for (int index : byTour) {
                int[] order = orders.get(index);
                RoutePathBuilder.Shape shape = planner.shape(order, bestCost);
                if (shape == null || !(shape.cost() < bestCost)) continue;
                Ball[] ordered = new Ball[order.length];
                for (int i = 0; i < order.length; i++) ordered[i] = reachable[order[i]];
                bestCost = shape.cost();
                best = new Route(start, returnPose, ordered, dropped, shape);
            }
            if (best != null) {
                List<Ball> visited = new ArrayList<>();
                Collections.addAll(visited, best.order);
                for (Ball ball : reachable) {
                    if (!visited.contains(ball)) dropped.add(new Dropped(ball, "no drivable order that also visits the others"));
                }
                return best;
            }
        }
        throw new IllegalArgumentException(String.format(
                "no drivable path for the robot from the start (%.1f, %.1f) to the return pose (%.1f, %.1f)",
                start.x(), start.y(), returnPose.x(), returnPose.y()));
    }

    private static String placementProblem(Ball ball, Pose start, List<Ball> kept, Region area,
                                           List<Obstacle> obstacles, RobotShape robot, double gapIn) {
        if (robot.covers(start, ball.x, ball.y)) return "under the robot at the start";
        String problem = ballProblem(ball.x, ball.y, ball.radius, robot, area, obstacles, gapIn);
        if (problem != null) return problem;
        for (Ball other : kept) {
            if (Math.hypot(ball.x - other.x, ball.y - other.y) < RoutePathBuilder.COINCIDENT_IN) {
                return "the same ball as " + other;
            }
        }
        return null;
    }

    private static double tourLength(Pose start, Ball[] balls, int[] order, Pose end) {
        double total = 0;
        double x = start.x();
        double y = start.y();
        for (int index : order) {
            total += Math.sqrt((balls[index].x - x) * (balls[index].x - x) + (balls[index].y - y) * (balls[index].y - y));
            x = balls[index].x;
            y = balls[index].y;
        }
        return total + Math.sqrt((end.x() - x) * (end.x() - x) + (end.y() - y) * (end.y() - y));
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
