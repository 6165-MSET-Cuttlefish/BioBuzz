package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.planner.Ball;
import org.firstinspires.ftc.teamcode.architecture.planner.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.planner.Region;
import org.firstinspires.ftc.teamcode.architecture.planner.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.planner.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteRun;
import org.firstinspires.ftc.teamcode.architecture.planner.VisibilityGraphPlanner;
import org.firstinspires.ftc.teamcode.biobuzz.RobotGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Camera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.BallType;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.FieldBall;

final class CollectionDisplay {
    private static final int DRAW_SAMPLES = 20;

    private CollectionDisplay() {}

    /** Planner settings outside the tests' Tuning; a new planner tunable goes here, or editing it leaves a stale plan. */
    static List<Double> plannerInputs() {
        return Arrays.asList(RobotGeometry.lengthIn, RobotGeometry.widthIn, RobotGeometry.intakeOffsetIn,
                RobotGeometry.intakeWidthIn, (double) VisibilityGraphPlanner.pointsPerCorner,
                VisibilityGraphPlanner.boundaryMarginIn, FieldConfig.fieldWidthInches,
                (double) Context.allianceColor.ordinal());
    }

    static String runState(RouteRun run) {
        if (run.isRunning()) return "DRIVING " + run.phase();
        if (run.finished()) return "DONE (run 0 then 1 to go again)";
        return "ABORTED: " + run.abortReason() + " (run 0 then 1 to go again)";
    }

    static String visionState(Camera camera) {
        if (camera.limelightProblem() != null) return "LIMELIGHT FAULT: " + camera.limelightProblem();
        if (camera.isSyncing()) return "STALE (putting the ball pipeline on the Limelight; wait before START)";
        if (!camera.isFrameStale()) return "live";
        double age = camera.getFrameAgeSeconds();
        return Double.isInfinite(age) ? "STALE (no frame yet)" : String.format("STALE (%.2f s old)", age);
    }

    static String describe(RoutePathBuilder.Reroute reroute) {
        switch (reroute) {
            case SPLINE: return "spline through detours round a HIVE rail or wall";
            case POLYLINE: return "straight hops round a HIVE rail or wall";
            case FORCED_SPLINE: return "spline THROUGH a HIVE rail (forceSplineOnly)";
            default: return "direct";
        }
    }

    static String color(BallType type) {
        switch (type) {
            case POLLEN: return "#FFD600";
            case NECTAR_RED: return "#E53935";
            case NECTAR_BLUE: return "#1E88E5";
            default: throw new IllegalArgumentException("unhandled ball type " + type);
        }
    }

    static void drawKeepIn(Canvas overlay, Region keepIn) {
        double[][] c = fieldRect(keepIn);
        overlay.setStroke("#00C853");
        overlay.strokePolyline(new double[]{c[0][0], c[0][1], c[0][2], c[0][3], c[0][0]},
                new double[]{c[1][0], c[1][1], c[1][2], c[1][3], c[1][0]});
    }

    static void drawObstacles(Canvas overlay, List<Obstacle> obstacles) {
        overlay.setStroke("#CC0000");
        overlay.setFill("#FF000033");
        for (Obstacle o : obstacles) {
            double[][] c = fieldRect(o.bounds);
            overlay.fillPolygon(c[0], c[1]);
            overlay.strokePolygon(c[0], c[1]);
        }
    }

    static void drawBalls(Canvas overlay, List<FieldBall> tracked, Ball[] planned, List<FieldBall> plannedFrom) {
        for (FieldBall b : tracked) {
            double[] p = FieldVisualization.toField(b.x, b.y);
            overlay.setStroke(b.visible() ? color(b.type) : "#9E9E9E");
            overlay.strokeCircle(p[0], p[1], b.type.diameterIn / 2);
        }
        for (int i = 0; i < planned.length; i++) {
            double[] p = FieldVisualization.toField(planned[i].x, planned[i].y);
            overlay.setFill(color(plannedFrom.get(i).type));
            overlay.fillCircle(p[0], p[1], planned[i].radius);
        }
    }

    private static double[][] fieldRect(Region r) {
        double[] a = FieldVisualization.toField(r.minX, r.minY);
        double[] b = FieldVisualization.toField(r.maxX, r.minY);
        double[] c = FieldVisualization.toField(r.maxX, r.maxY);
        double[] d = FieldVisualization.toField(r.minX, r.maxY);
        return new double[][]{{a[0], b[0], c[0], d[0]}, {a[1], b[1], c[1], d[1]}};
    }

    static void drawPlan(Canvas overlay, RoutePathBuilder.Plan plan, double[][] intakeDrawing, double[][] returnDrawing,
                         RobotShape shape) {
        if (intakeDrawing != null) {
            overlay.setStroke("#2962FF");
            overlay.strokePolyline(intakeDrawing[0], intakeDrawing[1]);
            double[][] track = plan.intakeTrack;
            int step = Math.max(1, track[0].length / DRAW_SAMPLES);
            int n = (track[0].length - 1) / step + 1;
            double[] xs = new double[n + 1];
            double[] ys = new double[n + 1];
            for (int i = 0; i < n; i++) {
                double[] p = FieldVisualization.toField(track[0][i * step], track[1][i * step]);
                xs[i] = p[0];
                ys[i] = p[1];
            }
            double[] last = FieldVisualization.toField(track[0][track[0].length - 1], track[1][track[1].length - 1]);
            xs[n] = last[0];
            ys[n] = last[1];
            overlay.setStroke("#FF6D00");
            overlay.strokePolyline(xs, ys);
            drawFootprint(overlay, shape, plan.intakeEnd);
        }
        if (returnDrawing != null) {
            overlay.setStroke("#82B1FF");
            overlay.strokePolyline(returnDrawing[0], returnDrawing[1]);
        }
    }

    private static void drawFootprint(Canvas overlay, RobotShape shape, Pose centre) {
        double[][] corners = shape.corners(centre);
        double[] xs = new double[4];
        double[] ys = new double[4];
        for (int i = 0; i < 4; i++) {
            double[] p = FieldVisualization.toField(corners[0][i], corners[1][i]);
            xs[i] = p[0];
            ys[i] = p[1];
        }
        overlay.setStroke("#0D47A1");
        overlay.strokePolygon(xs, ys);
    }
}
