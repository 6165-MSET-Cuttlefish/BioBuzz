package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.Ball;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.auto.Region;
import org.firstinspires.ftc.teamcode.architecture.auto.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.auto.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Vision Ball Collection's plan() call on synthetic layouts: RED's own half and the HIVE rails with its default gaps
 * and the placeholder robot geometry, the robot's pose as both start and return. Seeded, so every run plans the same
 * layouts.
 */
final class PlannerWorkload {
    static final double WALL_GAP_IN = 1;
    static final double RAIL_GAP_IN = 1;
    static final double BALL_RADIUS_IN = 1.4;
    static final RobotShape ROBOT = new RobotShape(18, 18, 9, 18);

    static final class Layout {
        final String label;
        final Pose start;
        final Ball[] balls;

        Layout(String label, Pose start, Ball[] balls) {
            this.label = label;
            this.start = start;
            this.balls = balls;
        }

        String describe() {
            StringBuilder sb = new StringBuilder(label).append(" start=").append(start).append(" balls=");
            for (Ball b : balls) sb.append(b);
            return sb.toString();
        }
    }

    static final class Timing {
        final double routeMs;
        final double buildMs;
        final int visited;
        final int dropped;
        final String intakeReroute;
        final String returnReroute;

        Timing(double routeMs, double buildMs, int visited, int dropped, String intakeReroute, String returnReroute) {
            this.routeMs = routeMs;
            this.buildMs = buildMs;
            this.visited = visited;
            this.dropped = dropped;
            this.intakeReroute = intakeReroute;
            this.returnReroute = returnReroute;
        }
    }

    // Static so it lasts as long as the loaded classes: an app start or a Sloth load resets it, as it does the JIT's work.
    private static boolean plannedBefore;

    final Region keepIn;
    final List<Obstacle> rails = BioBuzzField.hiveRails();
    private final Random random = new Random(6165);

    PlannerWorkload() {
        double w = FieldConfig.fieldWidthInches;
        keepIn = new Region(WALL_GAP_IN, w / 2 - WALL_GAP_IN, WALL_GAP_IN, w - WALL_GAP_IN);
    }

    Layout random(int balls) {
        Pose start;
        do {
            start = new Pose(uniform(keepIn.minX, keepIn.maxX), uniform(keepIn.minY, keepIn.maxY), uniform(0, 2 * Math.PI));
        } while (RouteOptimizer.poseProblem(start, ROBOT, keepIn, rails, RAIL_GAP_IN) != null);
        Ball[] b = new Ball[balls];
        for (int i = 0; i < balls; i++) {
            // Within the intake's reach, or the planner drops the ball and the n-ball case times a smaller search.
            do {
                b[i] = new Ball(uniform(keepIn.minX, keepIn.maxX), uniform(keepIn.minY, keepIn.maxY), BALL_RADIUS_IN);
            } while (RouteOptimizer.ballProblem(b[i].x, b[i].y, BALL_RADIUS_IN, ROBOT, keepIn, rails, RAIL_GAP_IN) != null
                    || ROBOT.covers(start, b[i].x, b[i].y));
        }
        return new Layout("random n=" + balls, start, b);
    }

    /** Hand-built worst cases: a line of balls under the HIVE behind the rail, balls on both sides of it, and a tight cluster. */
    List<Layout> crafted() {
        List<Layout> out = new ArrayList<>();
        Pose start = new Pose(20, 72, 0);
        out.add(new Layout("5 balls behind the rail", start, new Ball[] {
                ball(59, 56), ball(60, 64), ball(60, 72), ball(60, 80), ball(59, 88)}));
        out.add(bothSidesOfTheRail(5));
        out.add(new Layout("5-ball cluster", start, new Ball[] {
                ball(25, 100), ball(27, 103), ball(24, 106), ball(29, 99), ball(26, 109)}));
        return out;
    }

    Layout bothSidesOfTheRail(int balls) {
        Ball[] all = {ball(30, 60), ball(59, 62), ball(30, 85), ball(59, 84), ball(25, 110)};
        if (balls < 1 || balls > all.length) throw new IllegalArgumentException("bothSidesOfTheRail takes 1 to 5 balls, not " + balls);
        return new Layout(balls + " balls on both sides of the rail", new Pose(20, 72, 0), Arrays.copyOf(all, balls));
    }

    static boolean plannedBefore() {
        return plannedBefore;
    }

    Timing plan(Layout layout) {
        plannedBefore = true;
        try {
            long t0 = System.nanoTime();
            RouteOptimizer.Route route = RouteOptimizer.findOptimalRoute(layout.start, layout.balls, layout.start, keepIn,
                    rails, ROBOT, RAIL_GAP_IN, false);
            long t1 = System.nanoTime();
            RoutePathBuilder.Plan plan = RoutePathBuilder.build(route);
            long t2 = System.nanoTime();
            return new Timing((t1 - t0) / 1e6, (t2 - t1) / 1e6, route.order.length, route.dropped.size(),
                    String.valueOf(plan.intakeReroute), String.valueOf(plan.returnReroute));
        } catch (RuntimeException e) {
            throw new IllegalStateException("planner threw on " + layout.describe(), e);
        }
    }

    static Map<String, Object> summarize(List<Timing> timings) {
        Samples route = new Samples();
        Samples build = new Samples();
        Samples total = new Samples();
        int fewerVisited = 0;
        Map<String, Integer> reroutes = new LinkedHashMap<>();
        for (Timing t : timings) {
            route.add(t.routeMs);
            build.add(t.buildMs);
            total.add(t.routeMs + t.buildMs);
            if (t.dropped > 0) fewerVisited++;
            String key = t.intakeReroute + "/" + t.returnReroute;
            Integer c = reroutes.get(key);
            reroutes.put(key, c == null ? 1 : c + 1);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("plans", timings.size());
        m.put("findOptimalRouteMs", route.summary());
        m.put("buildMs", build.summary());
        m.put("totalMs", total.summary());
        m.put("plansWithDroppedBalls", fewerVisited);
        m.put("intakeSlashReturnReroutes", reroutes);
        return m;
    }

    private Ball ball(double x, double y) {
        return new Ball(x, y, BALL_RADIUS_IN);
    }

    private double uniform(double lo, double hi) {
        return lo + random.nextDouble() * (hi - lo);
    }
}
