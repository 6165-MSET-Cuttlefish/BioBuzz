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
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.BallType;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;

/**
 * Ball Collection with the balls coming from the Limelight instead of the dashboard, on this alliance's half
 * only. While idle it re-plans from the robot's pose whenever the nearest {@code maxBalls} usable balls appear,
 * vanish or move, or the robot moves; setting {@code run} to 1 drives exactly the plan shown and returns to where
 * the robot started it, and setting it to 0 aborts. Nothing plans or starts on stale vision or a Limelight fault.
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

        public static double startX = 20;
        public static double startY = 72;
        public static double startHeadingDeg = 0;

        /** Robot centre to the walls and the centre line. */
        public static double wallMarginIn = 9;
        public static double clearanceIn = 9;
        public static double intakeWidthIn = 18;
        public static double timeoutMinAvgSpeedIps = 10;
        public static double timeoutMinSec = 4;
        public static double timeoutMaxSec = 15;
        public static int run = 0;
    }

    // n! visit orders.
    private static final int MAX_BALLS = 5;
    private static final double TRAIL_SPACING_IN = 1;

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
    private Ball[] balls = new Ball[0];
    private List<Obstacle> obstacles = BioBuzzField.hiveRails();
    private RouteOptimizer.Route route;
    private RoutePathBuilder.Plan plan;
    private String noPlanReason = "waiting for the first plan";
    private double[][] intakeDrawing;
    private double[][] returnDrawing;
    private int skippedOutside;
    private int skippedRail;
    private int skippedRange;
    private int skippedDriven;

    private final List<double[]> trail = new ArrayList<>();
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
        Pose start = FieldPose.forAlliance(Tuning.startX, Tuning.startY, Math.toRadians(Tuning.startHeadingDeg));
        String problem = RouteOptimizer.poseProblem(start, BallCollectionRobot.ownHalf(Tuning.wallMarginIn),
                BioBuzzField.hiveRails(), Tuning.clearanceIn);
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
            double[] last = trail.get(trail.size() - 1);
            if (Math.hypot(pose.x() - last[0], pose.y() - last[1]) < TRAIL_SPACING_IN) return;
        }
        trail.add(new double[]{pose.x(), pose.y()});
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
        Region region = BallCollectionRobot.ownHalf(Tuning.wallMarginIn);
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
        skippedRail = 0;
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
            } else if (nearRail(ball)) {
                skippedRail++;
            } else if (ball.distanceTo(robotPose.x(), robotPose.y()) > Tuning.maxRangeIn) {
                skippedRange++;
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

    // An unseen ball on ground the robot has driven over has been collected or pushed, so its memory is stale.
    private boolean onTrail(FieldBall ball) {
        double reach = Tuning.intakeWidthIn / 2;
        for (double[] p : trail) {
            if (ball.distanceTo(p[0], p[1]) <= reach) return true;
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

        balls = new Ball[selected.size()];
        for (int i = 0; i < balls.length; i++) {
            FieldBall b = selected.get(i);
            balls[i] = new Ball(b.x, b.y, b.type.diameterIn / 2);
        }
        obstacles = BioBuzzField.hiveRails();

        clearPlan("");
        String problem = RouteOptimizer.poseProblem(pose, region, obstacles, Tuning.clearanceIn);
        if (problem != null) {
            noPlanReason = "the robot at " + problem;
            return;
        }
        route = RouteOptimizer.findOptimalRoute(pose, balls, pose, region, obstacles,
                Tuning.clearanceIn, Tuning.intakeWidthIn, false);
        plan = RoutePathBuilder.build(route);
        intakeDrawing = plan.intake == null ? null : BallCollectionTest.fieldPolyline(plan.intake);
        returnDrawing = plan.back == null ? null : BallCollectionTest.fieldPolyline(plan.back);
    }

    // The planner would drop such a ball anyway, but only after it had taken one of the nearest maxBalls places.
    private static boolean nearRail(FieldBall ball) {
        for (Obstacle o : BioBuzzField.hiveRails()) {
            if (o.blocks(ball.x, ball.y, Tuning.clearanceIn)) return true;
        }
        return false;
    }

    private static int maxBalls() {
        return Math.max(0, Math.min(MAX_BALLS, Tuning.maxBalls));
    }

    private static List<Double> settings() {
        List<Double> c = new ArrayList<>(Arrays.asList(
                (double) maxBalls(), Tuning.collectPollen ? 1.0 : 0.0, Tuning.collectRedNectar ? 1.0 : 0.0,
                Tuning.collectBlueNectar ? 1.0 : 0.0, Tuning.includeLastSeen ? 1.0 : 0.0, Tuning.maxRangeIn));
        c.addAll(Arrays.asList(Tuning.wallMarginIn, Tuning.clearanceIn, Tuning.intakeWidthIn,
                (double) VisibilityGraphPlanner.pointsPerCorner, VisibilityGraphPlanner.boundaryMarginIn,
                FieldConfig.fieldWidthInches, (double) Context.allianceColor.ordinal()));
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
        telemetry.addData("Balls", "%d tracked; skipped %d off our half, %d by a HIVE rail, %d beyond %.0f in, %d on the driven path; planning %d",
                camera.getFieldBalls().size(), skippedOutside, skippedRail, skippedRange, Tuning.maxRangeIn, skippedDriven,
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
        if (plan != null) BallCollectionTest.drawPlan(overlay, plan, intakeDrawing, returnDrawing);
    }

    private static String color(BallType type) {
        switch (type) {
            case POLLEN: return "#FFD600";
            case NECTAR_RED: return "#E53935";
            default: return "#1E88E5";
        }
    }
}
