package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.architecture.auto.Ball;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.auto.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.auto.VisibilityGraphPlanner;
import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

/**
 * Drives through the first {@code ballCount} balls placed on the dashboard and back to the start. While idle it
 * re-plans whenever the balls, obstacles or start pose change; setting {@code run} to 1 drives the plan, and
 * dropping it to 0 and back re-plans from wherever the robot stopped and goes again.
 */
@TeleOp(name = "Ball Collection", group = "Test")
public class BallCollectionTest extends EnhancedOpMode {

    @Config("Ball Collection")
    public static class Tuning {
        public static int ballCount = 3;
        public static Ball ball1 = new Ball(24, 24, 1.4);
        public static Ball ball2 = new Ball(100, 30, 1.4);
        public static Ball ball3 = new Ball(72, 120, 1.4);
        public static Ball ball4 = new Ball(40, 100, 1.4);

        public static Obstacle obstacle1 = new Obstacle(0, 0, 6);
        public static Obstacle obstacle2 = new Obstacle(0, 0, 6);
        public static Obstacle obstacle3 = new Obstacle(0, 0, 6);
        public static Obstacle obstacle4 = new Obstacle(0, 0, 6);
        public static Obstacle obstacle5 = new Obstacle(0, 0, 6);

        public static double startX = 72;
        public static double startY = 72;
        public static double startHeadingDeg = 0;

        public static double clearanceIn = 9;
        public static double intakeWidthIn = 18;
        public static double timeoutMinAvgSpeedIps = 10;
        public static double timeoutMinSec = 4;
        public static boolean forceSplineOnly = false;
        public static int run = 0;
    }

    private static final int MAX_BALLS = 4;
    private static final int DRAW_SAMPLES = 20;

    private double[] plannedConfig;
    private Pose startPose;
    private Ball[] balls;
    private List<Obstacle> obstacles;
    private RouteOptimizer.Route route;
    private RoutePathBuilder.Plan plan;
    private double[][] intakeDrawing;
    private double[][] returnDrawing;

    private Command routeCommand;
    private boolean prevRun;
    private String phase = "";
    private String timedOut = "";

    @Override
    protected Robot createRobot() {
        return new BallCollectionRobot(this);
    }

    @Override
    protected void initialize() {
        planFromConfig();
    }

    @Override
    protected void initializeLoop() {
        if (configChanged()) planFromConfig();
    }

    @Override
    protected void onStart() {
        // Re-seed after the Pinpoint's init recalibration, and don't treat a run=1 left over from INIT as a trigger.
        robot.follower.setPose(startPose);
        prevRun = Tuning.run == 1;
    }

    @Override
    protected void gameLoop() {
        if (routeCommand == null && configChanged()) planFromConfig();

        boolean runHigh = Tuning.run == 1;
        if (runHigh && !prevRun) startRoute();
        prevRun = runHigh;
    }

    private void startRoute() {
        if (routeCommand == null) {
            robot.follower.setPose(startPose);
        } else {
            if (Scheduler.isRunning(routeCommand)) return;
            startPose = robot.follower.pose();
            plan();
        }
        if (plan == null) return;

        timedOut = "";
        List<Command> steps = new ArrayList<>();
        if (plan.intake != null) steps.add(leg("intake", plan.intake));
        if (plan.back != null) steps.add(leg("return", plan.back));
        steps.add(PathCommands.stop(robot.follower));
        steps.add(instant(() -> phase = "done"));
        routeCommand = sequential(steps.toArray(new Command[0]));
        Scheduler.schedule(routeCommand);
    }

    private Command leg(String name, Path path) {
        double timeoutMs = 1000 * Math.max(Tuning.timeoutMinSec, length(path) / Tuning.timeoutMinAvgSpeedIps);
        return sequential(
                instant(() -> phase = name),
                race(
                        PathCommands.follow(robot.follower, path),
                        waitMs(timeoutMs).setEnd(end -> {
                            if (end == EndCondition.NATURALLY) timedOut += name + " ";
                        })));
    }

    private void planFromConfig() {
        plannedConfig = config();
        startPose = new Pose(Tuning.startX, Tuning.startY, Math.toRadians(Tuning.startHeadingDeg));
        robot.follower.setPose(startPose);
        plan();
    }

    private void plan() {
        // Copies, so a dashboard edit mid-route can't move what the running plan was built around.
        Ball[] slots = {Tuning.ball1, Tuning.ball2, Tuning.ball3, Tuning.ball4};
        balls = new Ball[ballCount()];
        for (int i = 0; i < balls.length; i++) balls[i] = new Ball(slots[i].x, slots[i].y, slots[i].radius);
        obstacles = new ArrayList<>();
        for (Obstacle o : obstacleSlots()) obstacles.add(new Obstacle(o.x, o.y, o.radius));

        route = RouteOptimizer.findOptimalRoute(startPose, balls, startPose, obstacles,
                Tuning.clearanceIn, Tuning.intakeWidthIn);
        plan = route == null ? null : RoutePathBuilder.build(startPose, route, startPose, obstacles,
                Tuning.clearanceIn, Tuning.intakeWidthIn, Tuning.forceSplineOnly);
        intakeDrawing = plan == null || plan.intake == null ? null : fieldPolyline(plan.intake);
        returnDrawing = plan == null || plan.back == null ? null : fieldPolyline(plan.back);
    }

    private static int ballCount() {
        return Math.max(0, Math.min(MAX_BALLS, Tuning.ballCount));
    }

    private static Obstacle[] obstacleSlots() {
        return new Obstacle[]{Tuning.obstacle1, Tuning.obstacle2, Tuning.obstacle3, Tuning.obstacle4, Tuning.obstacle5};
    }

    private boolean configChanged() {
        return !Arrays.equals(config(), plannedConfig);
    }

    private static double[] config() {
        Ball[] b = {Tuning.ball1, Tuning.ball2, Tuning.ball3, Tuning.ball4};
        Obstacle[] o = obstacleSlots();
        return new double[]{
                ballCount(),
                b[0].x, b[0].y, b[1].x, b[1].y, b[2].x, b[2].y, b[3].x, b[3].y,
                o[0].x, o[0].y, o[0].radius, o[1].x, o[1].y, o[1].radius, o[2].x, o[2].y, o[2].radius,
                o[3].x, o[3].y, o[3].radius, o[4].x, o[4].y, o[4].radius,
                Tuning.startX, Tuning.startY, Tuning.startHeadingDeg,
                Tuning.clearanceIn, Tuning.intakeWidthIn, Tuning.forceSplineOnly ? 1 : 0,
                VisibilityGraphPlanner.pointsPerObstacle, VisibilityGraphPlanner.boundaryMarginIn};
    }

    private static double length(Path path) {
        double total = 0;
        for (PathSegment segment : path.getSegments()) total += segment.curve.length();
        return total;
    }

    private static double[][] fieldPolyline(Path path) {
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

    @Override
    protected void telemetry() {
        String state = routeCommand == null ? (plan == null ? "IDLE (no route)" : "IDLE (set run=1 to drive)")
                : Scheduler.isRunning(routeCommand) ? "DRIVING " + phase
                : "DONE (run 0 then 1 to go again)";
        telemetry.addData("State", state);
        if (route == null) {
            telemetry.addData("Plan", "no order reaches every ball; move the balls or obstacles");
            return;
        }

        StringBuilder order = new StringBuilder();
        for (Ball ball : route.order) {
            if (order.length() > 0) order.append(" -> ");
            order.append("B").append(Arrays.asList(balls).indexOf(ball) + 1).append(' ').append(ball);
        }
        telemetry.addData("Order", order.length() == 0 ? "no balls" : order.toString());
        telemetry.addData("Est. length", "%.0f in", route.length);
        if (balls.length > 0) {
            telemetry.addData("Intake stops", "%d of %d (the intake sweeps up the rest)",
                    plan.essentialStops, balls.length);
        }
        telemetry.addData("Intake path", plan.intake == null ? "none" : describe(plan.intakeReroute));
        telemetry.addData("Return path", plan.back == null ? "none (already there)" : describe(plan.returnReroute));
        if (!timedOut.isEmpty()) telemetry.addData("Timed out", timedOut);
    }

    private static String describe(RoutePathBuilder.Reroute reroute) {
        switch (reroute) {
            case SPLINE: return "spline around an obstacle";
            case POLYLINE: return "straight hops (too tight for a spline)";
            case FORCED_SPLINE: return "spline THROUGH an obstacle (forceSplineOnly)";
            default: return "direct";
        }
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (balls == null) return;

        overlay.setStroke("#CC0000");
        overlay.setFill("#FF000033");
        for (Obstacle o : obstacles) {
            double[] p = FieldVisualization.toField(o.x, o.y);
            overlay.fillCircle(p[0], p[1], o.radius);
            overlay.strokeCircle(p[0], p[1], o.radius);
        }

        overlay.setFill("#FFA500");
        for (Ball b : balls) {
            double[] p = FieldVisualization.toField(b.x, b.y);
            overlay.fillCircle(p[0], p[1], b.radius);
        }

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
}
