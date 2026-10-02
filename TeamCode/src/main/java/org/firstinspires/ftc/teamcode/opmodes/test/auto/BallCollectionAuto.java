package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.auto.Ball;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.auto.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.biobuzz.BallCollection;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.RobotGeometry;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.BallType;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;

/**
 * {@link BallCollection} through the framework: built in initialize(), scheduled at START inside a timeout, then
 * it plans once from the balls in view and drives there and back.
 */
@Autonomous(name = "Ball Collection Auto", group = "Test")
public class BallCollectionAuto extends BioBuzzOpMode {

    @Config("Ball Collection Auto")
    public static class Tuning {
        public static double startX = 20;
        public static double startY = 72;
        public static double startHeadingDeg = 0;
        public static double timeoutMs = 25000;
        public static BallCollection.Settings collection = new BallCollection.Settings();
    }

    private Pose startPose;
    private BallCollection collection;
    private Command routine;
    private boolean timedOut;
    private RoutePathBuilder.Plan drawnPlan;
    private double[][] intakeDrawing;
    private double[][] returnDrawing;

    @Override
    protected void initialize() {
        startPose = FieldPose.forAlliance(Tuning.startX, Tuning.startY, Math.toRadians(Tuning.startHeadingDeg));
        String problem = RouteOptimizer.poseProblem(startPose, RobotGeometry.shape(),
                BioBuzzField.ownHalf(Tuning.collection.wallGapIn), BioBuzzField.hiveRails(), Tuning.collection.railGapIn);
        if (problem != null) throw new IllegalStateException("Ball Collection Auto start pose " + problem);
        robot.follower.setPose(startPose);

        collection = robot.actions.collectBalls(Tuning.collection);
        // The timeout branch ends NATURALLY only when it fires.
        routine = race(collection, waitMs(Tuning.timeoutMs).setEnd(end -> {
            if (end == EndCondition.NATURALLY) timedOut = true;
        }));
    }

    @Override
    protected void onStart() {
        // Re-seed after the Pinpoint's init recalibration.
        robot.follower.setPose(startPose);
        Scheduler.schedule(routine);
    }

    @Override
    protected void telemetry() {
        String status = collection.status().toString();
        if (collection.status() == BallCollection.Status.DRIVING) status += " " + collection.phase();
        if (!collection.detail().isEmpty()) status += ": " + collection.detail();
        if (timedOut) status += String.format(" (auto timed out after %.0f ms)", Tuning.timeoutMs);
        telemetry.addData("Status", status);
        telemetry.addData("Vision", visionState(robot.camera));
        if (collection.area() != null) telemetry.addData("Keep-in", "%s %s", Context.allianceColor, collection.area());
        for (BallCollection.Skipped s : collection.skipped()) {
            telemetry.addData("Skipped " + label(s.ball), s.reason);
        }
        RouteOptimizer.Route route = collection.route();
        if (route == null) return;

        StringBuilder order = new StringBuilder();
        for (Ball ball : route.order) {
            if (order.length() > 0) order.append(" -> ");
            order.append(label(collection.sourceOf(ball))).append(' ').append(ball);
        }
        telemetry.addData("Order", order.length() == 0 ? "no balls" : order.toString());
        for (RouteOptimizer.Dropped d : route.dropped) {
            telemetry.addData("Dropped " + label(collection.sourceOf(d.ball)), "%s: %s", d.ball, d.reason);
        }
        telemetry.addData("Length", "%.0f in", route.length);
        RoutePathBuilder.Plan plan = collection.plan();
        telemetry.addData("Intake path", plan.intake == null ? "none" : BallCollectionTest.describe(plan.intakeReroute));
        telemetry.addData("Return path", plan.back == null ? "none (already there)"
                : BallCollectionTest.describe(plan.returnReroute));
    }

    static String visionState(Camera camera) {
        if (camera.limelightProblem() != null) return "LIMELIGHT FAULT: " + camera.limelightProblem();
        if (camera.isSyncing()) return "STALE (putting the ball pipeline on the Limelight; wait before START)";
        if (!camera.isFrameStale()) return "live";
        double age = camera.getFrameAgeSeconds();
        return Double.isInfinite(age) ? "STALE (no frame yet)" : String.format("STALE (%.2f s old)", age);
    }

    private static String label(FieldBall ball) {
        return "#" + ball.id + " " + ball.type.label;
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (collection.area() != null) BallCollectionTest.drawKeepIn(overlay, collection.area());
        BallCollectionTest.drawObstacles(overlay, collection.obstacles());
        for (FieldBall b : robot.camera.getFieldBalls()) {
            double[] p = FieldVisualization.toField(b.x, b.y);
            overlay.setStroke(b.visible() ? color(b.type) : "#9E9E9E");
            overlay.strokeCircle(p[0], p[1], b.type.diameterIn / 2);
        }
        Ball[] balls = collection.balls();
        for (int i = 0; i < balls.length; i++) {
            double[] p = FieldVisualization.toField(balls[i].x, balls[i].y);
            overlay.setFill(color(collection.plannedBalls().get(i).type));
            overlay.fillCircle(p[0], p[1], balls[i].radius);
        }
        RoutePathBuilder.Plan plan = collection.plan();
        if (plan == null) return;
        if (plan != drawnPlan) {
            drawnPlan = plan;
            intakeDrawing = plan.intake == null ? null : BallCollectionTest.fieldPolyline(plan.intake);
            returnDrawing = plan.back == null ? null : BallCollectionTest.fieldPolyline(plan.back);
        }
        BallCollectionTest.drawPlan(overlay, plan, intakeDrawing, returnDrawing, collection.robot());
    }

    private static String color(BallType type) {
        switch (type) {
            case POLLEN: return "#FFD600";
            case NECTAR_RED: return "#E53935";
            case NECTAR_BLUE: return "#1E88E5";
            default: throw new IllegalArgumentException("unhandled ball type " + type);
        }
    }
}
