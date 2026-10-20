package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;

import java.util.List;

public class FieldVisualization {
    public static final double CANVAS_IN = 144;
    public static final double ROBOT_RADIUS = 7.5;

    public static final String COLOR_ROBOT = "#FFFFFF";
    public static final String COLOR_HISTORY = "#ff0015";

    private static final int PATH_SAMPLES = 20;

    private FieldVisualization() {}

    /** Pedro coordinates to the dashboard's FTC-standard field frame: translate by (-72,-72), rotate -90°. */
    public static Pose toField(Pose pedroPose) {
        return new Pose(
                pedroPose.y() - CANVAS_IN / 2,
                -(pedroPose.x() - CANVAS_IN / 2),
                pedroPose.heading() - Math.PI / 2);
    }

    /** Position-only {@link #toField(Pose)}, without allocating a Pose. */
    public static double[] toField(double x, double y) {
        double transX = x - CANVAS_IN / 2;
        double transY = y - CANVAS_IN / 2;
        return new double[]{ transY, -transX };
    }

    public static void drawRobot(Canvas canvas, Pose pose) {
        Pose canvasPose = toField(pose);
        double cx = canvasPose.x();
        double cy = canvasPose.y();
        double heading = canvasPose.heading();

        canvas.setStroke(COLOR_ROBOT);
        canvas.strokeCircle(cx, cy, ROBOT_RADIUS);
        canvas.strokeLine(cx, cy,
                cx + Math.cos(heading) * ROBOT_RADIUS,
                cy + Math.sin(heading) * ROBOT_RADIUS);
    }

    /** One polyline op per curve instead of one strokeLine per segment — same picture, far smaller packet. */
    private static void strokePedroPolyline(Canvas canvas, double[] pedroX, double[] pedroY) {
        int n = Math.min(pedroX.length, pedroY.length);
        if (n < 2) return;
        double[] fx = new double[n];
        double[] fy = new double[n];
        for (int i = 0; i < n; i++) {
            double[] p = toField(pedroX[i], pedroY[i]);
            fx[i] = p[0];
            fy[i] = p[1];
        }
        canvas.strokePolyline(fx, fy);
    }

    /** Field-frame polyline of every segment of {@code path}, for {@code Canvas.strokePolyline}. */
    public static double[][] pathPolyline(Path path) {
        List<PathSegment> segments = path.getSegments();
        double[] xs = new double[segments.size() * PATH_SAMPLES + 1];
        double[] ys = new double[xs.length];
        int i = 0;
        for (PathSegment segment : segments) {
            for (int s = i == 0 ? 0 : 1; s <= PATH_SAMPLES; s++) {
                Vector2D p = segment.curve.get((double) s / PATH_SAMPLES);
                double[] field = toField(p.x(), p.y());
                xs[i] = field[0];
                ys[i] = field[1];
                i++;
            }
        }
        return new double[][]{xs, ys};
    }

    public static void drawPoseHistory(Canvas canvas, PoseRing poseHistory) {
        canvas.setStroke(COLOR_HISTORY);
        strokePedroPolyline(canvas, poseHistory.xs(), poseHistory.ys());
    }
}
