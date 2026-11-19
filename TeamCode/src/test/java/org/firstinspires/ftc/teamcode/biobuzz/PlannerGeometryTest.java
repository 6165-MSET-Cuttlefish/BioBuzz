package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.planner.Ball;
import org.firstinspires.ftc.teamcode.architecture.planner.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.planner.Region;
import org.firstinspires.ftc.teamcode.architecture.planner.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.planner.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.planner.VisibilityGraphPlanner;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/** Drives every planned step through Pedro's paths and checks the footprint with its own maths, not RobotShape's. */
public class PlannerGeometryTest {
    // Fixed rather than RobotGeometry's, so measuring the robot doesn't move these numbers.
    private static final double LENGTH_IN = 16, WIDTH_IN = 15, INTAKE_OFFSET_IN = 8, INTAKE_WIDTH_IN = 15;
    private static final RobotShape ROBOT = new RobotShape(LENGTH_IN, WIDTH_IN, INTAKE_OFFSET_IN, INTAKE_WIDTH_IN);
    private static final double GAP_IN = 1;
    private static final double FRONT = Math.max(LENGTH_IN / 2, INTAKE_OFFSET_IN), BACK = LENGTH_IN / 2;
    private static final double HALF_WIDTH = Math.max(WIDTH_IN, INTAKE_WIDTH_IN) / 2;
    // The planner checks the footprint every 1 in and 2 deg, so between checks a corner can come up to half of one step's move closer.
    private static final double BETWEEN_SAMPLES_IN = (1 + Math.hypot(FRONT, HALF_WIDTH) * Math.toRadians(2)) / 2;
    // The planner measures a ball to chords of the intake's track, which cut inside the curve.
    private static final double INTAKE_SLACK_IN = 0.02;

    private AllianceColor alliance;
    private double sliceMs;
    private Region area;
    private List<Obstacle> rails;

    @Before
    public void field() {
        alliance = Context.allianceColor;
        sliceMs = VisibilityGraphPlanner.routeSearchSliceMs;
        Context.allianceColor = AllianceColor.RED;
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
        VisibilityGraphPlanner.routeSearchSliceMs = Double.POSITIVE_INFINITY;
        area = BioBuzzField.ownHalf(GAP_IN);
        rails = BioBuzzField.hiveRails();
    }

    @After
    public void restore() {
        Context.allianceColor = alliance;
        VisibilityGraphPlanner.routeSearchSliceMs = sliceMs;
        FieldConfig.clear();
    }

    @Test
    public void waypointDrivesKeepClear() {
        Random random = new Random(1);
        for (int i = 0; i < 600; i++) {
            Pose start = randomPose(random);
            Pose target = i % 2 == 0 ? randomPose(random) : shotPose(random);
            RouteOptimizer.Search search = plan(start, new Ball[0], target);
            if (search.noRoute() != null) continue;
            assertNull(start + " -> " + target, problem(search.plan(), start, target, new Ball[0]));
        }
    }

    @Test
    public void collectionsKeepClearAndPassOverEveryBall() {
        Random random = new Random(2);
        for (int balls = 1; balls <= BallCollection.BALLS; balls++) {
            for (int i = 0; i < 25; i++) {
                Pose start = randomPose(random);
                Ball[] layout = randomBalls(random, start, balls);
                RouteOptimizer.Search search = plan(start, layout, start);
                if (search.noRoute() != null) continue;
                RouteOptimizer.Route route = search.route();
                assertNull(start + " " + describe(layout), problem(search.plan(), start, start, route.order));
            }
        }
    }

    // 84% plan here, 39% without the return leg's turning-clearance detour: 75% catches losing it.
    @Test
    public void mostWaypointDrivesNeedingADetourPlan() {
        Random random = new Random(3);
        int needDetour = 0;
        int planned = 0;
        for (int i = 0; i < 1000; i++) {
            RouteOptimizer.Search search = plan(randomPose(random), new Ball[0], randomPose(random));
            if (search.noRoute() != null) {
                needDetour++;
            } else if (search.plan().returnReroute != RoutePathBuilder.Reroute.NONE) {
                needDetour++;
                planned++;
            }
        }
        assertTrue(planned + " of " + needDetour, planned >= 0.75 * needDetour);
    }

    private RouteOptimizer.Search plan(Pose start, Ball[] balls, Pose target) {
        RouteOptimizer.Search search = RouteOptimizer.search(start, balls, target, area, rails, ROBOT, GAP_IN, false);
        assertTrue("an infinite slice finishes in one step", search.step());
        return search;
    }

    private Pose randomPose(Random random) {
        while (true) {
            Pose p = new Pose(uniform(random, area.minX, area.maxX), uniform(random, area.minY, area.maxY),
                    uniform(random, 0, 2 * Math.PI));
            if (RouteOptimizer.poseProblem(p, ROBOT, area, rails, GAP_IN) == null) return p;
        }
    }

    // North or south of the HIVE with the robot's back toward it, where BioBuzz Tele's shots come from.
    private Pose shotPose(Random random) {
        while (true) {
            boolean north = random.nextBoolean();
            double y = north ? uniform(random, 92, 125) : uniform(random, 16, 50);
            double heading = (north ? Math.PI / 2 : -Math.PI / 2) + Math.toRadians(uniform(random, -20, 20));
            Pose p = new Pose(uniform(random, 35, area.maxX), y, Angle.normalize(heading));
            if (RouteOptimizer.poseProblem(p, ROBOT, area, rails, GAP_IN) == null) return p;
        }
    }

    private Ball[] randomBalls(Random random, Pose start, int count) {
        Ball[] balls = new Ball[count];
        for (int i = 0; i < count; i++) {
            do {
                balls[i] = new Ball(uniform(random, area.minX, area.maxX), uniform(random, area.minY, area.maxY), 1.4);
            } while (RouteOptimizer.ballProblem(balls[i].x, balls[i].y, ROBOT, area, rails, GAP_IN) != null
                    || ROBOT.covers(start, balls[i].x, balls[i].y));
        }
        return balls;
    }

    /** Null if every step starts where the last ended, the footprint stays clear throughout, and each ball passes under the intake. */
    private String problem(RoutePathBuilder.Plan plan, Pose start, Pose end, Ball[] order) {
        double x = start.x(), y = start.y(), h = start.heading();
        List<double[]> intakeTrack = new ArrayList<>();
        for (int k = 0; k < plan.steps.size(); k++) {
            RoutePathBuilder.Step step = plan.steps.get(k);
            if (step.path == null) {
                if (Math.hypot(step.turnTo.x() - x, step.turnTo.y() - y) > 1e-6) return "step " + k + " turns away from where the robot is";
                double sweep = Angle.normalizeSigned(step.turnTo.heading() - h);
                int n = Math.max(1, (int) Math.ceil(Math.abs(sweep) / Math.toRadians(0.5)));
                for (int i = 0; i <= n; i++) {
                    String p = footprintProblem(x, y, h + sweep * i / n);
                    if (p != null) return "step " + k + ", a turn at " + at(x, y, h + sweep * i / n) + ": " + p;
                }
                h = step.turnTo.heading();
                continue;
            }
            Pose first = step.path.get(0);
            if (Math.hypot(first.x() - x, first.y() - y) > 1e-6) return "step " + k + " starts away from where the robot is";
            if (Math.abs(Angle.normalizeSigned(first.heading() - h)) > Math.toRadians(1.5)) {
                return "step " + k + " starts facing " + Math.toDegrees(first.heading()) + " deg, not " + Math.toDegrees(h);
            }
            int n = Math.max(400, (int) Math.ceil(step.length * 8));
            for (int i = 0; i <= n; i++) {
                Pose p = step.path.get((double) i / n);
                String problem = footprintProblem(p.x(), p.y(), p.heading());
                if (problem != null) return "step " + k + ", a drive at " + at(p.x(), p.y(), p.heading()) + ": " + problem;
                if (step.intake) {
                    intakeTrack.add(new double[]{p.x() + INTAKE_OFFSET_IN * Math.cos(p.heading()),
                            p.y() + INTAKE_OFFSET_IN * Math.sin(p.heading())});
                }
            }
            Pose last = step.path.get(1);
            x = last.x();
            y = last.y();
            h = last.heading();
        }
        // The planner treats points closer than COINCIDENT_IN as one, and then only turns.
        if (Math.hypot(end.x() - x, end.y() - y) > RoutePathBuilder.COINCIDENT_IN) return "ends at " + at(x, y, h);
        if (Math.abs(Angle.normalizeSigned(end.heading() - h)) > Math.toRadians(1.5)) return "ends at " + at(x, y, h);
        for (Ball b : order) {
            if (distanceToTrack(intakeTrack, b.x, b.y) > INTAKE_WIDTH_IN / 2 + INTAKE_SLACK_IN) return b + " misses the intake";
        }
        return null;
    }

    private String footprintProblem(double x, double y, double h) {
        double c = Math.cos(h), s = Math.sin(h);
        double[][] local = {{FRONT, HALF_WIDTH}, {-BACK, HALF_WIDTH}, {-BACK, -HALF_WIDTH}, {FRONT, -HALF_WIDTH}};
        double[] px = new double[4], py = new double[4];
        for (int i = 0; i < 4; i++) {
            px[i] = x + local[i][0] * c - local[i][1] * s;
            py[i] = y + local[i][0] * s + local[i][1] * c;
            double over = Math.max(Math.max(area.minX - px[i], px[i] - area.maxX), Math.max(area.minY - py[i], py[i] - area.maxY));
            if (over > BETWEEN_SAMPLES_IN) return String.format("a corner %.2f in outside %s", over, area);
        }
        for (Obstacle rail : rails) {
            Region b = rail.bounds;
            double d = polygonDistance(px, py, new double[]{b.minX, b.maxX, b.maxX, b.minX}, new double[]{b.minY, b.minY, b.maxY, b.maxY});
            if (d < GAP_IN - BETWEEN_SAMPLES_IN) return String.format("%.2f in from %s", d, rail);
        }
        return null;
    }

    private static double distanceToTrack(List<double[]> track, double x, double y) {
        double best = Double.POSITIVE_INFINITY;
        for (int i = 1; i < track.size(); i++) {
            double[] a = track.get(i - 1), b = track.get(i);
            best = Math.min(best, pointToSegment(x, y, a[0], a[1], b[0], b[1]));
        }
        return best;
    }

    private static double polygonDistance(double[] ax, double[] ay, double[] bx, double[] by) {
        if (inside(ax[0], ay[0], bx, by) || inside(bx[0], by[0], ax, ay)) return 0;
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                best = Math.min(best, segmentToSegment(ax[i], ay[i], ax[(i + 1) % 4], ay[(i + 1) % 4],
                        bx[j], by[j], bx[(j + 1) % 4], by[(j + 1) % 4]));
            }
        }
        return best;
    }

    private static boolean inside(double x, double y, double[] qx, double[] qy) {
        boolean left = false, right = false;
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            double cross = (qx[j] - qx[i]) * (y - qy[i]) - (qy[j] - qy[i]) * (x - qx[i]);
            if (cross > 0) left = true;
            if (cross < 0) right = true;
        }
        return !(left && right);
    }

    private static double segmentToSegment(double x1, double y1, double x2, double y2,
                                           double x3, double y3, double x4, double y4) {
        double d1 = cross(x3, y3, x4, y4, x1, y1), d2 = cross(x3, y3, x4, y4, x2, y2);
        double d3 = cross(x1, y1, x2, y2, x3, y3), d4 = cross(x1, y1, x2, y2, x4, y4);
        if (d1 * d2 < 0 && d3 * d4 < 0) return 0;
        return Math.min(Math.min(pointToSegment(x1, y1, x3, y3, x4, y4), pointToSegment(x2, y2, x3, y3, x4, y4)),
                Math.min(pointToSegment(x3, y3, x1, y1, x2, y2), pointToSegment(x4, y4, x1, y1, x2, y2)));
    }

    private static double cross(double ax, double ay, double bx, double by, double px, double py) {
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    private static double pointToSegment(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay;
        double lengthSq = dx * dx + dy * dy;
        double t = lengthSq == 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lengthSq));
        return Math.hypot(px - ax - t * dx, py - ay - t * dy);
    }

    private static String at(double x, double y, double h) {
        return String.format("(%.2f, %.2f) facing %.1f deg", x, y, Math.toDegrees(h));
    }

    private static String describe(Ball[] balls) {
        StringBuilder sb = new StringBuilder();
        for (Ball b : balls) sb.append(b);
        return sb.toString();
    }

    private static double uniform(Random random, double lo, double hi) {
        return lo + random.nextDouble() * (hi - lo);
    }
}
