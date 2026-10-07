package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.firstinspires.ftc.teamcode.architecture.auto.Ball;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.auto.Region;
import org.firstinspires.ftc.teamcode.architecture.auto.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.auto.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteRun;
import org.firstinspires.ftc.teamcode.architecture.auto.VisibilityGraphPlanner;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BallCollection;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.biobuzz.RobotGeometry;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.BallType;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;

/**
 * Ball Collection with the balls coming from the Limelight instead of the dashboard, from (84, 72, 270 deg)
 * wherever the robot stands unless {@code onField} is on. While idle it re-plans from the robot's pose whenever the nearest
 * {@code maxBalls} usable balls appear, vanish or move, or the robot moves; setting {@code run} to 1 drives exactly the
 * plan shown and returns to where the robot started it, and setting it to 0 aborts. Nothing plans or starts on stale
 * vision or a Limelight fault.
 */
@TeleOp(name = "Vision Ball Collection", group = "Test")
public class VisionBallCollectionTest extends EnhancedOpMode {

    @Config("Vision Ball Collection")
    public static class Tuning {
        public static int maxBalls = 4;
        public static boolean collectPollen = true;
        public static boolean collectRedNectar = true;
        public static boolean collectBlueNectar = true;
        /** Also plan through balls out of view but still remembered, except any lying on a path already driven. */
        public static boolean includeLastSeen = false;
        public static double maxRangeIn = 60;

        public static double replanMoveIn = 2;
        public static double replanTurnDeg = 10;
        public static double replanIntervalMs = 250;

        public static boolean onField = false;
        public static double startX = 84;
        public static double startY = 72;
        public static double startHeadingDeg = 90;

        /** Least room between the robot's footprint and the walls or the centre line. */
        public static double wallGapIn = 1;
        /** Least room between the robot's footprint and a HIVE rail or obstacle. */
        public static double railGapIn = 1;
        /** In the start pose's frame (RED field coordinates with onField on), on top of the HIVE rails. */
        public static Box obstacle1 = new Box(true, 40.75, 82.54, 6, 6);
        public static Box obstacle2 = new Box(false, 40.75, 100, 6, 6);
        public static Box obstacle3 = new Box(false, 40.75, 65, 6, 6);
        public static double timeoutMinAvgSpeedIps = 10;
        public static double timeoutMinSec = 4;
        public static double timeoutMaxSec = 30;
        public static int run = 0;
    }

    /** A rectangular obstacle by its centre and size, editable on the dashboard. */
    public static class Box {
        public boolean enabled;
        public double x;
        public double y;
        public double sizeXIn;
        public double sizeYIn;

        public Box(boolean enabled, double x, double y, double sizeXIn, double sizeYIn) {
            this.enabled = enabled;
            this.x = x;
            this.y = y;
            this.sizeXIn = sizeXIn;
            this.sizeYIn = sizeYIn;
        }
    }

    // n! visit orders.
    private static final int MAX_BALLS = 5;
    private static final double TRAIL_SPACING_IN = 1;
    private static final double TRAIL_TURN_DEG = 5;

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
    private List<Double> plannedSettings;
    private double lastPlanSeconds = Double.NEGATIVE_INFINITY;
    private Pose startPose;
    private Region keepIn;
    private RobotShape shape;
    private Ball[] balls = new Ball[0];
    private List<Obstacle> obstacles = BioBuzzField.hiveRails();
    private RouteOptimizer.Route route;
    private RoutePathBuilder.Plan plan;
    private String noPlanReason = "waiting for the first plan";
    private double[][] intakeDrawing;
    private double[][] returnDrawing;
    private int skippedOutside;
    private int skippedUnreachable;
    private int skippedRange;
    private int skippedDriven;

    private final List<Pose> trail = new ArrayList<>();
    private RouteRun routeRun;
    private boolean prevRun;
    private String refused = "";

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
        // The INIT plan was built from the pose before this re-seed: drop it and re-plan on the first loop.
        clearPlan("re-planning from the start pose");
        plannedSettings = null;
        lastPlanSeconds = Double.NEGATIVE_INFINITY;
        prevRun = Tuning.run == 1;
    }

    // The configured start is configuration, so a bad one throws rather than showing as "no plan".
    private static Pose configuredStart() {
        Pose start = BallCollection.configuredPose(Tuning.onField, Tuning.startX, Tuning.startY, Tuning.startHeadingDeg);
        String problem = RouteOptimizer.poseProblem(start, RobotGeometry.shape(),
                BallCollection.keepIn(Tuning.onField, Tuning.wallGapIn), obstacles(), Tuning.railGapIn);
        if (problem != null) throw new IllegalStateException("Vision Ball Collection start pose " + problem);
        return start;
    }

    @Override
    protected void gameLoop() {
        boolean runHigh = Tuning.run == 1;
        if (routeRun != null && routeRun.isRunning()) {
            recordTrail(robot.follower.pose());
            if (!runHigh) routeRun.abort("run set to 0");
            prevRun = runHigh;
            return;
        }

        // No re-plan on the run edge: run drives the plan that was on screen.
        if (runHigh && !prevRun) startRoute();
        else replanIfChanged();
        prevRun = runHigh;
    }

    private void startRoute() {
        Camera camera = visionRobot.camera;
        if (camera.isFrameStale()) {
            refused = "vision: " + BallCollectionAuto.visionState(camera);
            return;
        }
        if (plan == null) {
            refused = "no plan: " + noPlanReason;
            return;
        }
        Pose pose = robot.follower.pose();
        if (movedSincePlan(pose)) {
            refused = String.format("the robot moved %.1f in / turned %.0f deg since the plan; wait for the re-plan",
                    pose.distance(startPose), Math.toDegrees(turn(pose, startPose)));
            return;
        }
        refused = "";
        routeRun = new RouteRun(robot.follower, plan,
                Tuning.timeoutMinAvgSpeedIps, Tuning.timeoutMinSec, Tuning.timeoutMaxSec);
        routeRun.start();
    }

    private boolean movedSincePlan(Pose pose) {
        return pose.distance(startPose) > Tuning.replanMoveIn
                || Math.toDegrees(turn(pose, startPose)) > Tuning.replanTurnDeg;
    }

    private static double turn(Pose a, Pose b) {
        double d = a.heading() - b.heading();
        return Math.abs(Math.atan2(Math.sin(d), Math.cos(d)));
    }

    private void recordTrail(Pose pose) {
        if (!trail.isEmpty()) {
            Pose last = trail.get(trail.size() - 1);
            if (pose.distance(last) < TRAIL_SPACING_IN && Math.toDegrees(turn(pose, last)) < TRAIL_TURN_DEG) return;
        }
        trail.add(pose);
    }

    private void replanIfChanged() {
        double now = System.nanoTime() * 1e-9;
        if ((now - lastPlanSeconds) * 1000 < Tuning.replanIntervalMs) return;
        Camera camera = visionRobot.camera;
        if (camera.isFrameStale()) {
            clearPlan("vision: " + BallCollectionAuto.visionState(camera));
            balls = new Ball[0];
            plannedFrom = Collections.emptyList();
            plannedSettings = null;
            lastPlanSeconds = now;
            return;
        }
        Pose pose = robot.follower.pose();
        Region region = BallCollection.keepIn(Tuning.onField, Tuning.wallGapIn);
        List<FieldBall> selected = selectBalls(pose, region);
        if (plannedSettings != null && !ballsChanged(selected) && settings().equals(plannedSettings)
                && !movedSincePlan(pose)) {
            return;
        }
        plan(selected, pose, region, now);
    }

    /** The nearest {@code maxBalls} usable balls, in id order so a reshuffle of distances isn't a change. */
    private List<FieldBall> selectBalls(final Pose robotPose, Region region) {
        skippedOutside = 0;
        skippedUnreachable = 0;
        skippedRange = 0;
        skippedDriven = 0;
        List<FieldBall> wanted = new ArrayList<>();
        for (FieldBall ball : visionRobot.camera.getFieldBalls()) {
            if (!wants(ball.type)) continue;
            if (!ball.visible()) {
                if (!Tuning.includeLastSeen) continue;
                if (onTrail(ball)) {
                    skippedDriven++;
                    continue;
                }
            }
            if (!region.contains(ball.x, ball.y)) {
                skippedOutside++;
            } else if (ball.distanceTo(robotPose.x(), robotPose.y()) > Tuning.maxRangeIn) {
                skippedRange++;
            } else if (RouteOptimizer.ballProblem(ball.x, ball.y, ball.type.diameterIn / 2, RobotGeometry.shape(),
                    region, obstacles(), Tuning.railGapIn) != null) {
                // The planner would drop it anyway, but only after it had taken one of the nearest maxBalls places.
                skippedUnreachable++;
            } else {
                wanted.add(ball);
            }
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

    // An unseen ball the robot has driven over has been collected or pushed, so its memory is stale.
    private boolean onTrail(FieldBall ball) {
        RobotShape robotNow = RobotGeometry.shape();
        for (Pose p : trail) {
            if (robotNow.covers(p, ball.x, ball.y)) return true;
        }
        return false;
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

    private void clearPlan(String reason) {
        route = null;
        plan = null;
        intakeDrawing = null;
        returnDrawing = null;
        noPlanReason = reason;
    }

    private void plan(List<FieldBall> selected, Pose pose, Region region, double now) {
        plannedFrom = selected;
        plannedSettings = settings();
        lastPlanSeconds = now;
        startPose = pose;
        keepIn = region;
        shape = RobotGeometry.shape();

        balls = new Ball[selected.size()];
        for (int i = 0; i < balls.length; i++) {
            FieldBall b = selected.get(i);
            balls[i] = new Ball(b.x, b.y, b.type.diameterIn / 2);
        }
        obstacles = obstacles();

        clearPlan("");
        String problem = RouteOptimizer.poseProblem(pose, shape, region, obstacles, Tuning.railGapIn);
        if (problem != null) {
            noPlanReason = "the robot at " + problem;
            return;
        }
        route = RouteOptimizer.findOptimalRoute(pose, balls, pose, region, obstacles, shape, Tuning.railGapIn, false);
        plan = RoutePathBuilder.build(route);
        intakeDrawing = plan.intake == null ? null : BallCollectionTest.fieldPolyline(plan.intake);
        returnDrawing = plan.back == null ? null : BallCollectionTest.fieldPolyline(plan.back);
    }

    private static List<Obstacle> obstacles() {
        List<Obstacle> out = new ArrayList<>(BallCollection.obstacles(Tuning.onField));
        for (Box box : new Box[]{Tuning.obstacle1, Tuning.obstacle2, Tuning.obstacle3}) {
            if (!box.enabled) continue;
            Pose a = BallCollection.configuredPose(Tuning.onField, box.x - box.sizeXIn / 2, box.y - box.sizeYIn / 2, 0);
            Pose b = BallCollection.configuredPose(Tuning.onField, box.x + box.sizeXIn / 2, box.y + box.sizeYIn / 2, 0);
            out.add(new Obstacle(Math.min(a.x(), b.x()), Math.max(a.x(), b.x()),
                    Math.min(a.y(), b.y()), Math.max(a.y(), b.y())));
        }
        return out;
    }

    private static int maxBalls() {
        return Math.max(0, Math.min(MAX_BALLS, Tuning.maxBalls));
    }

    private static List<Double> settings() {
        List<Double> c = new ArrayList<>(Arrays.asList(
                (double) maxBalls(), Tuning.collectPollen ? 1.0 : 0.0, Tuning.collectRedNectar ? 1.0 : 0.0,
                Tuning.collectBlueNectar ? 1.0 : 0.0, Tuning.includeLastSeen ? 1.0 : 0.0, Tuning.maxRangeIn,
                Tuning.onField ? 1.0 : 0.0));
        c.addAll(Arrays.asList(Tuning.wallGapIn, Tuning.railGapIn, RobotGeometry.lengthIn, RobotGeometry.widthIn,
                RobotGeometry.intakeOffsetIn, RobotGeometry.intakeWidthIn,
                (double) VisibilityGraphPlanner.pointsPerCorner, VisibilityGraphPlanner.boundaryMarginIn,
                FieldConfig.fieldWidthInches, (double) Context.allianceColor.ordinal()));
        for (Box box : new Box[]{Tuning.obstacle1, Tuning.obstacle2, Tuning.obstacle3}) {
            c.addAll(Arrays.asList(box.enabled ? 1.0 : 0.0, box.x, box.y, box.sizeXIn, box.sizeYIn));
        }
        return c;
    }

    @Override
    protected void telemetry() {
        String state = routeRun != null ? BallCollectionTest.runState(routeRun)
                : plan == null ? "IDLE (no plan)" : "IDLE (set run=1 to drive)";
        telemetry.addData("State", state);
        if (!refused.isEmpty()) telemetry.addData("Run refused", refused);
        Camera camera = visionRobot.camera;
        if (camera.isFrameStale()) telemetry.addData("Vision", BallCollectionAuto.visionState(camera));
        if (keepIn != null) telemetry.addData("Keep-in", "%s %s", Context.allianceColor, keepIn);
        telemetry.addData("Balls", "%d tracked; skipped %d off our half, %d out of the intake's reach, %d beyond %.0f in, %d on the driven path; planning %d",
                camera.getFieldBalls().size(), skippedOutside, skippedUnreachable, skippedRange, Tuning.maxRangeIn, skippedDriven,
                balls.length);
        if (route == null) {
            telemetry.addData("Plan", "none: " + noPlanReason);
            return;
        }

        StringBuilder order = new StringBuilder();
        for (Ball ball : route.order) {
            if (order.length() > 0) order.append(" -> ");
            order.append(label(ball)).append(' ').append(ball);
        }
        telemetry.addData("Order", order.length() == 0 ? "no balls" : order.toString());
        for (RouteOptimizer.Dropped d : route.dropped) {
            telemetry.addData("Dropped " + label(d.ball), "%s: %s", d.ball, d.reason);
        }
        telemetry.addData("Length", "%.0f in", route.length);
        if (route.order.length > 0) {
            telemetry.addData("Intake stops", "%d of %d (the intake sweeps up the rest)",
                    plan.essentialStops, route.order.length);
        }
        telemetry.addData("Intake path", plan.intake == null ? "none" : BallCollectionTest.describe(plan.intakeReroute));
        telemetry.addData("Return path", plan.back == null ? "none (already there)"
                : BallCollectionTest.describe(plan.returnReroute));
    }

    private String label(Ball ball) {
        FieldBall source = plannedFrom.get(Arrays.asList(balls).indexOf(ball));
        return "#" + source.id + " " + source.type.label;
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (keepIn != null) BallCollectionTest.drawKeepIn(overlay, keepIn);
        BallCollectionTest.drawObstacles(overlay, obstacles);

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
        if (plan != null) BallCollectionTest.drawPlan(overlay, plan, intakeDrawing, returnDrawing, shape);
    }

    private static String color(BallType type) {
        switch (type) {
            case POLLEN: return "#FFD600";
            case NECTAR_RED: return "#E53935";
            default: return "#1E88E5";
        }
    }
}
