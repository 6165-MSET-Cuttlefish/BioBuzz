package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static org.firstinspires.ftc.teamcode.architecture.command.PathCommands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.planner.Ball;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.planner.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.biobuzz.BallCollection;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.RobotGeometry;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;

@Autonomous(name = "Ball Collection Auto", group = "Test")
public class BallCollectionAuto extends BioBuzzOpMode {

    @Config("Ball Collection Auto")
    public static class Tuning {
        public static double startXIn = 0;
        public static double startYIn = 0;
        public static double startHeadingDeg = 0;
        public static double timeoutMs = 25000;
        public static BallCollection.Settings collection = new BallCollection.Settings();

        static {
            collection.onField = false;
        }
    }

    private BallCollection collection;
    private Command routine;
    private boolean timedOut;
    private RoutePathBuilder.Plan drawnPlan;
    private double[][] intakeDrawing;
    private double[][] returnDrawing;

    @Override
    protected void initialize() {
        boolean onField = Tuning.collection.onField;
        Pose startPose = BallCollection.configuredPose(onField, Tuning.startXIn, Tuning.startYIn, Tuning.startHeadingDeg);
        String problem = RouteOptimizer.poseProblem(startPose, RobotGeometry.shape(),
                BallCollection.keepIn(onField, Tuning.collection.wallGapIn), BallCollection.obstacles(onField),
                Tuning.collection.railGapIn);
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
        Scheduler.schedule(routine);
    }

    @Override
    protected void telemetry() {
        String status = collection.status().toString();
        if (collection.status() == BallCollection.Status.DRIVING) status += " " + collection.phase();
        if (!collection.detail().isEmpty()) status += ": " + collection.detail();
        if (timedOut) status += String.format(" (auto timed out after %.0f ms)", Tuning.timeoutMs);
        telemetry.addData("Status", status);
        telemetry.addData("Vision", CollectionDisplay.visionState(robot.camera));
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
        telemetry.addData("Intake path", plan.intake == null ? "none" : CollectionDisplay.describe(plan.intakeReroute));
        telemetry.addData("Return path", plan.back == null ? "none (already there)"
                : CollectionDisplay.describe(plan.returnReroute));
    }

    private static String label(FieldBall ball) {
        return "#" + ball.id + " " + ball.type.label;
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (collection.area() != null) CollectionDisplay.drawKeepIn(overlay, collection.area());
        CollectionDisplay.drawObstacles(overlay, collection.obstacles());
        CollectionDisplay.drawBalls(overlay, robot.camera.getFieldBalls(), collection.balls(), collection.plannedBalls());
        RoutePathBuilder.Plan plan = collection.plan();
        if (plan == null) return;
        if (plan != drawnPlan) {
            drawnPlan = plan;
            intakeDrawing = plan.intake == null ? null : FieldVisualization.pathPolyline(plan.intake);
            returnDrawing = plan.back == null ? null : FieldVisualization.pathPolyline(plan.back);
        }
        CollectionDisplay.drawPlan(overlay, plan, intakeDrawing, returnDrawing, collection.robot());
    }
}
