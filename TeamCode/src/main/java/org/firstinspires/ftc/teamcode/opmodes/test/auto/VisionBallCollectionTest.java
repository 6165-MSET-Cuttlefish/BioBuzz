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
import java.util.Collections;
import java.util.Comparator;
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
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.BallType;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;

/**
 * Ball Collection with the balls coming from the Limelight instead of the dashboard. While idle it re-plans
 * whenever the nearest {@code maxBalls} detected balls appear, vanish or move; setting {@code run} to 1 freezes
 * that plan and drives it, returning to where the robot was when it started. Run 0 and back goes again.
 */
@TeleOp(name = "Vision Ball Collection", group = "Test")
public class VisionBallCollectionTest extends EnhancedOpMode {

    @Config("Vision Ball Collection")
    public static class Tuning {
        public static int maxBalls = 4;
        public static boolean collectPollen = true;
        public static boolean collectRedNectar = true;
        public static boolean collectBlueNectar = true;
        /** Also plan through balls out of view but still remembered by FieldBallTracker. */
        public static boolean includeLastSeen = false;

        public static double replanMoveIn = 2;
        public static double replanIntervalMs = 250;

        public static Obstacle obstacle1 = new Obstacle(0, 0, 6);
        public static Obstacle obstacle2 = new Obstacle(0, 0, 6);
        public static Obstacle obstacle3 = new Obstacle(0, 0, 6);

        public static double startX = 72;
        public static double startY = 72;
        public static double startHeadingDeg = 0;

        public static double clearanceIn = 9;
        public static double intakeWidthIn = 18;
        public static double timeoutMinAvgSpeedIps = 10;
        public static double timeoutMinSec = 4;
        public static int run = 0;
    }

    // n! visit orders.
    private static final int MAX_BALLS = 5;
    private static final int DRAW_SAMPLES = 20;

    static class VisionRobot extends BallCollectionRobot {
        Camera camera;

        VisionRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected void initializeGameModules() {
            camera = new Camera(opMode.hardwareMap).withFollower(follower);
        }
    }

    private VisionRobot visionRobot;

    private List<FieldBall> plannedFrom = Collections.emptyList();
    private double[] plannedSettings;
    private double lastPlanSeconds = Double.NEGATIVE_INFINITY;
    private Pose startPose;
    private Ball[] balls = new Ball[0];
    private List<Obstacle> obstacles = new ArrayList<>();
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
        visionRobot = new VisionRobot(this);
        return visionRobot;
    }

    @Override
    protected void initialize() {
        startPose = configuredStart();
        robot.follower.setPose(startPose);
    }

    @Override
    protected void initializeLoop() {
        replanIfChanged();
    }

    @Override
    protected void onStart() {
        // Re-seed after the Pinpoint's init recalibration, and don't treat a run=1 left over from INIT as a trigger.
        startPose = configuredStart();
        robot.follower.setPose(startPose);
        prevRun = Tuning.run == 1;
    }

    private static Pose configuredStart() {
        return new Pose(Tuning.startX, Tuning.startY, Math.toRadians(Tuning.startHeadingDeg));
    }

    @Override
    protected void gameLoop() {
        boolean driving = routeCommand != null && Scheduler.isRunning(routeCommand);
        if (!driving) replanIfChanged();

        boolean runHigh = Tuning.run == 1;
        if (runHigh && !prevRun && !driving) startRoute();
        prevRun = runHigh;
    }

    private void startRoute() {
        startPose = robot.follower.pose();
        plan(selectBalls());
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

    private void replanIfChanged() {
        double now = System.nanoTime() * 1e-9;
        if ((now - lastPlanSeconds) * 1000 < Tuning.replanIntervalMs) return;
        List<FieldBall> selected = selectBalls();
        if (!ballsChanged(selected) && Arrays.equals(settings(), plannedSettings)) return;
        startPose = robot.follower.pose();
        plan(selected);
    }

    /** The nearest {@code maxBalls} wanted balls, in id order so a reshuffle of distances isn't a change. */
    private List<FieldBall> selectBalls() {
        final Pose robotPose = robot.follower.pose();
        List<FieldBall> wanted = new ArrayList<>();
        for (FieldBall ball : visionRobot.camera.getFieldBalls()) {
            if ((ball.visible() || Tuning.includeLastSeen) && wants(ball.type)) wanted.add(ball);
        }
        Collections.sort(wanted, new Comparator<FieldBall>() {
            @Override public int compare(FieldBall a, FieldBall b) {
                return Double.compare(a.distanceTo(robotPose.x(), robotPose.y()),
                        b.distanceTo(robotPose.x(), robotPose.y()));
            }
        });
        List<FieldBall> selected = new ArrayList<>(wanted.subList(0, Math.min(wanted.size(), maxBalls())));
        Collections.sort(selected, new Comparator<FieldBall>() {
            @Override public int compare(FieldBall a, FieldBall b) {
                return Integer.compare(a.id, b.id);
            }
        });
        return selected;
    }

    private static boolean wants(BallType type) {
        switch (type) {
            case POLLEN: return Tuning.collectPollen;
            case NECTAR_RED: return Tuning.collectRedNectar;
            default: return Tuning.collectBlueNectar;
        }
    }

    private boolean ballsChanged(List<FieldBall> selected) {
        if (selected.size() != plannedFrom.size()) return true;
        for (int i = 0; i < selected.size(); i++) {
            FieldBall now = selected.get(i);
            FieldBall then = plannedFrom.get(i);
            if (now.id != then.id || now.distanceTo(then.x, then.y) > Tuning.replanMoveIn) return true;
        }
        return false;
    }

    private void plan(List<FieldBall> selected) {
        plannedFrom = selected;
        plannedSettings = settings();
        lastPlanSeconds = System.nanoTime() * 1e-9;

        balls = new Ball[selected.size()];
        for (int i = 0; i < balls.length; i++) {
            FieldBall b = selected.get(i);
            balls[i] = new Ball(b.x, b.y, b.type.diameterIn / 2);
        }
        // Copies, so a dashboard edit mid-route can't move what the running plan was built around.
        obstacles = new ArrayList<>();
        for (Obstacle o : obstacleSlots()) obstacles.add(new Obstacle(o.x, o.y, o.radius));

        route = RouteOptimizer.findOptimalRoute(startPose, balls, startPose, obstacles,
                Tuning.clearanceIn, Tuning.intakeWidthIn);
        plan = route == null ? null : RoutePathBuilder.build(startPose, route, startPose, obstacles,
                Tuning.clearanceIn, Tuning.intakeWidthIn, false);
        intakeDrawing = plan == null || plan.intake == null ? null : fieldPolyline(plan.intake);
        returnDrawing = plan == null || plan.back == null ? null : fieldPolyline(plan.back);
    }

    private static int maxBalls() {
        return Math.max(0, Math.min(MAX_BALLS, Tuning.maxBalls));
    }

    private static Obstacle[] obstacleSlots() {
        return new Obstacle[]{Tuning.obstacle1, Tuning.obstacle2, Tuning.obstacle3};
    }

    private static double[] settings() {
        Obstacle[] o = obstacleSlots();
        return new double[]{
                maxBalls(), Tuning.collectPollen ? 1 : 0, Tuning.collectRedNectar ? 1 : 0,
                Tuning.collectBlueNectar ? 1 : 0, Tuning.includeLastSeen ? 1 : 0,
                o[0].x, o[0].y, o[0].radius, o[1].x, o[1].y, o[1].radius, o[2].x, o[2].y, o[2].radius,
                Tuning.clearanceIn, Tuning.intakeWidthIn,
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
        boolean driving = routeCommand != null && Scheduler.isRunning(routeCommand);
        String state = driving ? "DRIVING " + phase
                : routeCommand != null ? "DONE (run 0 then 1 to go again)"
                : plan == null ? "IDLE (no route)" : "IDLE (set run=1 to drive)";
        telemetry.addData("State", state);
        if (visionRobot.camera.isFrameStale()) {
            telemetry.addData("Vision", "STALE (%.2fs): check the Limelight and its pipeline",
                    visionRobot.camera.getFrameAgeSeconds());
        }
        telemetry.addData("Balls seen", "%d, planning %d", visionRobot.camera.getFieldBalls().size(), balls.length);
        if (route == null) {
            telemetry.addData("Plan", "no order reaches every ball; move the balls or obstacles");
            return;
        }

        StringBuilder order = new StringBuilder();
        for (Ball ball : route.order) {
            FieldBall source = plannedFrom.get(Arrays.asList(balls).indexOf(ball));
            if (order.length() > 0) order.append(" -> ");
            order.append('#').append(source.id).append(' ').append(source.type.label).append(' ').append(ball);
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
            default: return "direct";
        }
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        overlay.setStroke("#CC0000");
        overlay.setFill("#FF000033");
        for (Obstacle o : obstacles) {
            double[] p = FieldVisualization.toField(o.x, o.y);
            overlay.fillCircle(p[0], p[1], o.radius);
            overlay.strokeCircle(p[0], p[1], o.radius);
        }

        // Every tracked ball, outlined in its colour; the planned ones are filled below.
        for (FieldBall b : visionRobot.camera.getFieldBalls()) {
            double[] p = FieldVisualization.toField(b.x, b.y);
            overlay.setStroke(b.visible() ? color(b.type) : "#9E9E9E");
            overlay.strokeCircle(p[0], p[1], b.type.diameterIn / 2);
        }
        for (int i = 0; i < balls.length; i++) {
            double[] p = FieldVisualization.toField(balls[i].x, balls[i].y);
            overlay.setFill(color(plannedFrom.get(i).type));
            overlay.fillCircle(p[0], p[1], balls[i].radius);
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

    private static String color(BallType type) {
        switch (type) {
            case POLLEN: return "#FFD600";
            case NECTAR_RED: return "#E53935";
            default: return "#1E88E5";
        }
    }
}
