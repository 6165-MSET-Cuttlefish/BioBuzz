package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.architecture.auto.Ball;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.auto.Region;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.auto.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteRun;
import org.firstinspires.ftc.teamcode.architecture.auto.VisibilityGraphPlanner;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;

/**
 * Drives through the first {@code ballCount} balls placed on the dashboard and back to the start, staying on this
 * alliance's half and around the HIVE's ground rails. Balls and the start are authored for RED and mapped to BLUE. While idle it
 * re-plans whenever the inputs change; setting {@code run} to 1 drives the plan shown, and setting it to 0 aborts.
 * After a run it re-plans from wherever the robot stopped, so run 0 then 1 goes again from there.
 */
@TeleOp(name = "Ball Collection", group = "Test")
public class BallCollectionTest extends EnhancedOpMode {

    @Config("Ball Collection")
    public static class Tuning {
        public static int ballCount = 3;
        public static Ball ball1 = new Ball(30, 50, 1.4);
        public static Ball ball2 = new Ball(30, 92, 1.4);
        public static Ball ball3 = new Ball(28, 112, 1.4);
        public static Ball ball4 = new Ball(50, 28, 1.4);

        public static double startX = 20;
        public static double startY = 72;
        public static double startHeadingDeg = 0;

        /** Robot centre to the walls and the centre line. */
        public static double wallMarginIn = 9;
        /** Robot centre to the edge of a HIVE rail. */
        public static double clearanceIn = 9;
        public static double intakeWidthIn = 18;
        public static double timeoutMinAvgSpeedIps = 10;
        public static double timeoutMinSec = 4;
        public static double timeoutMaxSec = 15;
        public static boolean forceSplineOnly = false;
        public static int run = 0;
    }

    private static final int MAX_BALLS = 4;
    private static final int DRAW_SAMPLES = 20;
    private static final double MOVED_IN = 1;

    private List<Double> plannedConfig;
    private Pose startPose;
    private Region keepIn;
    private Ball[] balls = new Ball[0];
    private List<Obstacle> obstacles = BioBuzzField.hiveRails();
    private RouteOptimizer.Route route;
    private RoutePathBuilder.Plan plan;
    private String noPlanReason = "";
    private double[][] intakeDrawing;
    private double[][] returnDrawing;

    private RouteRun routeRun;
    private boolean replannedAfterRun;
    private boolean prevRun;
    private String refused = "";

    @Override
    protected Robot createRobot() {
        return new BallCollectionRobot(this);
    }

    @Override
    protected void initialize() {
        planFromConfiguredStart();
    }

    @Override
    protected void initializeLoop() {
        if (configChanged()) planFromConfiguredStart();
    }

    @Override
    protected void onStart() {
        // Re-seed after the Pinpoint's init recalibration, and don't treat a run=1 left over from INIT as a trigger.
        robot.follower.setPose(startPose);
        prevRun = Tuning.run == 1;
    }

    @Override
    protected void gameLoop() {
        boolean runHigh = Tuning.run == 1;
        if (routeRun != null && routeRun.isRunning()) {
            if (!runHigh) routeRun.abort("run set to 0");
            prevRun = runHigh;
            return;
        }

        if (routeRun == null) {
            if (configChanged()) planFromConfiguredStart();
        } else if (!replannedAfterRun || configChanged()
                || robot.follower.pose().distance(startPose) > MOVED_IN) {
            replannedAfterRun = true;
            planFrom(robot.follower.pose());
        }

        if (runHigh && !prevRun) startRoute();
        prevRun = runHigh;
    }

    private void startRoute() {
        if (plan == null) {
            refused = "no plan: " + noPlanReason;
            return;
        }
        double off = robot.follower.pose().distance(startPose);
        if (off > MOVED_IN) {
            refused = String.format("the robot is %.1f in from the plan's start", off);
            return;
        }
        refused = "";
        routeRun = new RouteRun(robot.follower, plan,
                Tuning.timeoutMinAvgSpeedIps, Tuning.timeoutMinSec, Tuning.timeoutMaxSec);
        replannedAfterRun = false;
        routeRun.start();
    }

    // The configured start is configuration, so a bad one throws rather than showing as "no plan".
    private void planFromConfiguredStart() {
        Pose configured = FieldPose.forAlliance(Tuning.startX, Tuning.startY, Math.toRadians(Tuning.startHeadingDeg));
        String problem = RouteOptimizer.poseProblem(configured, BallCollectionRobot.ownHalf(Tuning.wallMarginIn),
                BioBuzzField.hiveRails(), Tuning.clearanceIn);
        if (problem != null) throw new IllegalStateException("Ball Collection start pose " + problem);
        robot.follower.setPose(configured);
        planFrom(configured);
    }

    private void planFrom(Pose start) {
        plannedConfig = config();
        startPose = start;
        keepIn = BallCollectionRobot.ownHalf(Tuning.wallMarginIn);
        // Copies, so a dashboard edit mid-route can't move what the running plan was built around.
        Ball[] slots = {Tuning.ball1, Tuning.ball2, Tuning.ball3, Tuning.ball4};
        balls = new Ball[ballCount()];
        for (int i = 0; i < balls.length; i++) {
            Pose p = FieldPose.forAlliance(slots[i].x, slots[i].y, 0);
            balls[i] = new Ball(p.x(), p.y(), slots[i].radius);
        }
        obstacles = BioBuzzField.hiveRails();

        route = null;
        plan = null;
        intakeDrawing = null;
        returnDrawing = null;
        String problem = RouteOptimizer.poseProblem(start, keepIn, obstacles, Tuning.clearanceIn);
        if (problem != null) {
            noPlanReason = "the robot at " + problem;
            return;
        }
        route = RouteOptimizer.findOptimalRoute(start, balls, start, keepIn, obstacles,
                Tuning.clearanceIn, Tuning.intakeWidthIn, Tuning.forceSplineOnly);
        plan = RoutePathBuilder.build(route);
        intakeDrawing = plan.intake == null ? null : fieldPolyline(plan.intake);
        returnDrawing = plan.back == null ? null : fieldPolyline(plan.back);
    }

    private static int ballCount() {
        return Math.max(0, Math.min(MAX_BALLS, Tuning.ballCount));
    }

    private boolean configChanged() {
        return !config().equals(plannedConfig);
    }

    private static List<Double> config() {
        List<Double> c = new ArrayList<>();
        c.add((double) ballCount());
        for (Ball b : new Ball[]{Tuning.ball1, Tuning.ball2, Tuning.ball3, Tuning.ball4}) {
            c.add(b.x);
            c.add(b.y);
        }
        c.addAll(Arrays.asList(Tuning.startX, Tuning.startY, Tuning.startHeadingDeg,
                Tuning.wallMarginIn, Tuning.clearanceIn, Tuning.intakeWidthIn, Tuning.forceSplineOnly ? 1.0 : 0.0,
                (double) VisibilityGraphPlanner.pointsPerCorner, VisibilityGraphPlanner.boundaryMarginIn,
                FieldConfig.fieldWidthInches, (double) Context.allianceColor.ordinal()));
        return c;
    }

    static double[][] fieldPolyline(Path path) {
        List<PathSegment> segments = path.getSegments();
        double[] xs = new double[segments.size() * DRAW_SAMPLES + 1];
        double[] ys = new double[xs.length];
        int i = 0;
        for (PathSegment segment : segments) {
            for (int s = i == 0 ? 0 : 1; s <= DRAW_SAMPLES; s++) {
                Vector2D p = segment.curve.get((double) s / DRAW_SAMPLES);
                double[] field = FieldVisualization.toField(p.x(), p.y());
                xs[i] = field[0];
                ys[i] = field[1];
                i++;
            }
        }
        return new double[][]{xs, ys};
    }

    static String runState(RouteRun run) {
        if (run.isRunning()) return "DRIVING " + run.phase();
        if (run.finished()) return "DONE (run 0 then 1 to go again)";
        return "ABORTED: " + run.abortReason() + " (run 0 then 1 to go again)";
    }

    @Override
    protected void telemetry() {
        String state = routeRun != null ? runState(routeRun)
                : plan == null ? "IDLE (no plan)" : "IDLE (set run=1 to drive)";
        telemetry.addData("State", state);
        if (!refused.isEmpty()) telemetry.addData("Run refused", refused);
        if (keepIn != null) telemetry.addData("Keep-in", "%s %s", Context.allianceColor, keepIn);
        if (route == null) {
            telemetry.addData("Plan", "none: " + noPlanReason);
            return;
        }

        StringBuilder order = new StringBuilder();
        for (Ball ball : route.order) {
            if (order.length() > 0) order.append(" -> ");
            order.append("B").append(Arrays.asList(balls).indexOf(ball) + 1).append(' ').append(ball);
        }
        telemetry.addData("Order", order.length() == 0 ? "no balls" : order.toString());
        for (RouteOptimizer.Dropped d : route.dropped) {
            telemetry.addData("Dropped B" + (Arrays.asList(balls).indexOf(d.ball) + 1), "%s: %s", d.ball, d.reason);
        }
        telemetry.addData("Length", "%.0f in", route.length);
        if (route.order.length > 0) {
            telemetry.addData("Intake stops", "%d of %d (the intake sweeps up the rest)",
                    plan.essentialStops, route.order.length);
        }
        telemetry.addData("Intake path", plan.intake == null ? "none" : describe(plan.intakeReroute));
        telemetry.addData("Return path", plan.back == null ? "none (already there)" : describe(plan.returnReroute));
    }

    static String describe(RoutePathBuilder.Reroute reroute) {
        switch (reroute) {
            case SPLINE: return "spline around a HIVE rail or wall";
            case POLYLINE: return "straight hops (too tight for a spline)";
            case FORCED_SPLINE: return "spline THROUGH a HIVE rail (forceSplineOnly)";
            default: return "direct";
        }
    }

    static void drawKeepIn(Canvas overlay, Region keepIn) {
        double[] a = FieldVisualization.toField(keepIn.minX, keepIn.minY);
        double[] b = FieldVisualization.toField(keepIn.maxX, keepIn.minY);
        double[] c = FieldVisualization.toField(keepIn.maxX, keepIn.maxY);
        double[] d = FieldVisualization.toField(keepIn.minX, keepIn.maxY);
        overlay.setStroke("#00C853");
        overlay.strokePolyline(new double[]{a[0], b[0], c[0], d[0], a[0]}, new double[]{a[1], b[1], c[1], d[1], a[1]});
    }

    static void drawObstacles(Canvas overlay, List<Obstacle> obstacles) {
        overlay.setStroke("#CC0000");
        overlay.setFill("#FF000033");
        for (Obstacle o : obstacles) {
            double[] a = FieldVisualization.toField(o.minX, o.minY);
            double[] b = FieldVisualization.toField(o.maxX, o.minY);
            double[] c = FieldVisualization.toField(o.maxX, o.maxY);
            double[] d = FieldVisualization.toField(o.minX, o.maxY);
            double[] xs = {a[0], b[0], c[0], d[0]};
            double[] ys = {a[1], b[1], c[1], d[1]};
            overlay.fillPolygon(xs, ys);
            overlay.strokePolygon(xs, ys);
        }
    }

    static void drawPlan(Canvas overlay, RoutePathBuilder.Plan plan, double[][] intakeDrawing, double[][] returnDrawing) {
        if (intakeDrawing != null) {
            overlay.setStroke("#2962FF");
            overlay.strokePolyline(intakeDrawing[0], intakeDrawing[1]);
            double[] end = FieldVisualization.toField(plan.intakeEnd.x(), plan.intakeEnd.y());
            overlay.setStroke("#0D47A1");
            overlay.strokeCircle(end[0], end[1], 2.5);
        }
        if (returnDrawing != null) {
            overlay.setStroke("#82B1FF");
            overlay.strokePolyline(returnDrawing[0], returnDrawing[1]);
        }
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (keepIn != null) drawKeepIn(overlay, keepIn);
        drawObstacles(overlay, obstacles);

        overlay.setFill("#FFA500");
        for (Ball b : balls) {
            double[] p = FieldVisualization.toField(b.x, b.y);
            overlay.fillCircle(p[0], p[1], b.radius);
        }
        if (plan != null) drawPlan(overlay, plan, intakeDrawing, returnDrawing);
    }
}
