package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;

import java.util.List;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.planner.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.planner.Region;
import org.firstinspires.ftc.teamcode.architecture.planner.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.planner.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteRun;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.BallType;

final class CollectionDisplay {
    private static final int DRAW_SAMPLES = 20;

    private CollectionDisplay() {}

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
