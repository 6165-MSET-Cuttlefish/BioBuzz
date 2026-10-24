package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.planner.Ball;
import org.firstinspires.ftc.teamcode.architecture.planner.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.planner.Region;
import org.firstinspires.ftc.teamcode.architecture.planner.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.planner.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteRun;
import org.firstinspires.ftc.teamcode.biobuzz.BallCollection;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.biobuzz.RobotGeometry;

@TeleOp(name = "Ball Collection", group = "Test")
public class BallCollectionTest extends EnhancedOpMode {

    @Config("Ball Collection")
    public static class Tuning {
        public static Ball ball1 = new Ball(24, 0, 1.4);
        public static Ball ball2 = new Ball(36, 18, 1.4);
        public static Ball ball3 = new Ball(48, -18, 1.4);
        public static Ball ball4 = new Ball(60, 6, 1.4);

        public static boolean onField = false;
        public static double startXIn = 0;
        public static double startYIn = 0;
        public static double startHeadingDeg = 0;

        /** Least room between the robot's footprint and the walls or the centre line. */
        public static double wallGapIn = 1;
        /** Least room between the robot's footprint and a HIVE rail. */
        public static double railGapIn = 1;
        public static double timeoutMinAvgSpeedIps = 10;
        public static double timeoutMinSec = 4;
        public static double timeoutMaxSec = 30;
        public static boolean forceSplineOnly = false;
        public static int run = 0;
    }

    private static final double MOVED_IN = 1;

    private List<Double> plannedConfig;
    private Pose requestedStart;
    private Region requestedArea;
    private RobotShape requestedShape;
    private Ball[] requestedBalls;
    private List<Obstacle> requestedObstacles;
    private RouteOptimizer.Search search;
    private long searchStartNs;
    private Pose startPose;
    private Region area;
    private RobotShape shape;
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
        stepSearch();
    }

    @Override
    protected void onStart() {
        // Don't treat a run=1 left over from INIT as a trigger.
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
            // From the tracked pose: re-seeding the configured start after START would hide any move since INIT.
            if (configChanged()) planFrom(robot.follower.pose());
        } else if (!replannedAfterRun || configChanged()
                || robot.follower.pose().distance(requestedStart) > MOVED_IN) {
            replannedAfterRun = true;
            planFrom(robot.follower.pose());
        }
        stepSearch();

        if (runHigh && !prevRun) startRoute();
        prevRun = runHigh;
    }

    private void startRoute() {
        // Run drives a plan for the current config and pose, never the older one still on screen.
        if (search != null) {
            refused = "still planning: set run to 0, then 1 once the plan is shown";
            return;
        }
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
        Pose configured = BallCollection.configuredPose(Tuning.onField, Tuning.startXIn, Tuning.startYIn, Tuning.startHeadingDeg);
        String problem = RouteOptimizer.poseProblem(configured, RobotGeometry.shape(),
                BallCollection.keepIn(Tuning.onField, Tuning.wallGapIn), BallCollection.obstacles(Tuning.onField),
                Tuning.railGapIn);
        if (problem != null) throw new IllegalStateException("Ball Collection start pose " + problem);
        robot.follower.setPose(configured);
        planFrom(configured);
    }

    private void planFrom(Pose start) {
        plannedConfig = config();
        requestedStart = start;
        requestedArea = BallCollection.keepIn(Tuning.onField, Tuning.wallGapIn);
        requestedShape = RobotGeometry.shape();
        // Copies, so a dashboard edit mid-route can't move what the running plan was built around.
        Ball[] slots = {Tuning.ball1, Tuning.ball2, Tuning.ball3, Tuning.ball4};
        requestedBalls = new Ball[slots.length];
        for (int i = 0; i < slots.length; i++) {
            Pose p = BallCollection.configuredPose(Tuning.onField, slots[i].x, slots[i].y, 0);
            requestedBalls[i] = new Ball(p.x(), p.y(), slots[i].radius);
        }
        requestedObstacles = BallCollection.obstacles(Tuning.onField);

        search = null;
        String problem = RouteOptimizer.poseProblem(start, requestedShape, requestedArea, requestedObstacles,
                Tuning.railGapIn);
        if (problem != null) {
            show(null, null);
            noPlanReason = "the robot at " + problem;
            return;
        }
        search = RouteOptimizer.search(start, requestedBalls, start, requestedArea, requestedObstacles, requestedShape,
                Tuning.railGapIn, Tuning.forceSplineOnly);
        searchStartNs = System.nanoTime();
    }

    private void stepSearch() {
        if (search != null && search.step()) show(search.route(), search.plan());
    }

    private void show(RouteOptimizer.Route route, RoutePathBuilder.Plan plan) {
        search = null;
        startPose = requestedStart;
        area = requestedArea;
        shape = requestedShape;
        balls = requestedBalls;
        obstacles = requestedObstacles;
        this.route = route;
        this.plan = plan;
        intakeDrawing = plan == null || plan.intake == null ? null : FieldVisualization.pathPolyline(plan.intake);
        returnDrawing = plan == null || plan.back == null ? null : FieldVisualization.pathPolyline(plan.back);
    }

    private boolean configChanged() {
        return !config().equals(plannedConfig);
    }

    private static List<Double> config() {
        List<Double> c = new ArrayList<>();
        c.add(Tuning.onField ? 1.0 : 0.0);
        for (Ball b : new Ball[]{Tuning.ball1, Tuning.ball2, Tuning.ball3, Tuning.ball4}) {
            c.add(b.x);
            c.add(b.y);
        }
        c.addAll(Arrays.asList(Tuning.startXIn, Tuning.startYIn, Tuning.startHeadingDeg,
                Tuning.wallGapIn, Tuning.railGapIn, Tuning.forceSplineOnly ? 1.0 : 0.0));
        c.addAll(CollectionDisplay.plannerInputs());
        return c;
    }

    @Override
    protected void telemetry() {
        String state = search != null ? "PLANNING" : routeRun != null ? CollectionDisplay.runState(routeRun)
                : plan == null ? "IDLE (no plan)" : "IDLE (set run=1 to drive)";
        telemetry.addData("State", state);
        if (!refused.isEmpty()) telemetry.addData("Run refused", refused);
        if (area != null) telemetry.addData("Keep-in", "%s %s", Context.allianceColor, area);
        if (search != null) telemetry.addData("Planning", "%.0f ms so far", (System.nanoTime() - searchStartNs) / 1e6);
        if (route == null) {
            telemetry.addData("Plan", "none: " + (search != null ? "still planning" : noPlanReason));
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
        telemetry.addData("Intake path", plan.intake == null ? "none" : CollectionDisplay.describe(plan.intakeReroute));
        telemetry.addData("Return path", plan.back == null ? "none (already there)"
                : CollectionDisplay.describe(plan.returnReroute));
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (area != null) CollectionDisplay.drawKeepIn(overlay, area);
        CollectionDisplay.drawObstacles(overlay, obstacles);

        overlay.setFill("#FFA500");
        for (Ball b : balls) {
            double[] p = FieldVisualization.toField(b.x, b.y);
            overlay.fillCircle(p[0], p[1], b.radius);
        }
        if (plan != null) CollectionDisplay.drawPlan(overlay, plan, intakeDrawing, returnDrawing, shape);
    }
}
