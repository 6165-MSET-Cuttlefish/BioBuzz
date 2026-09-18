package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;

import java.util.List;

@Config
public class FieldVisualization {
    public static final double ROBOT_RADIUS = 7.5;

    public static final String COLOR_ROBOT = "#FFFFFF";
    public static final String COLOR_PATH = "#ea8743";
    public static final String COLOR_CURRENT_PATH = "#00ff26";
    public static final String COLOR_HISTORY = "#ff0015";
    public static final String COLOR_HEADING = "#00e5ff";

    public static int headingTickCount = 8;
    public static double headingTickLength = 5.0;
    /** Perpendicular nudge off the path, so a tick stays visible where planned heading ∥ path tangent. */
    public static double headingTickOffset = 1.5;

    public static int pathSamplesPerSegment = 40;

    private FieldVisualization() {}

    /** Pedro coordinates to the dashboard's FTC-standard field frame: translate by (-72,-72), rotate -90°. */
    public static Pose toField(Pose pedroPose) {
        return new Pose(
                pedroPose.y() - 72.0,
                -(pedroPose.x() - 72.0),
                pedroPose.heading() - Math.PI / 2);
    }

    /** Position-only {@link #toField(Pose)}, without allocating a Pose. */
    public static double[] toField(double x, double y) {
        double transX = x - 72.0;
        double transY = y - 72.0;
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

    /** Sampled per {@link PathSegment} rather than over the whole path, so a compound path draws every leg. */
    public static void drawPath(Canvas canvas, Path path, String color) {
        canvas.setStroke(color);
        int samples = Math.max(2, pathSamplesPerSegment);
        List<PathSegment> segments = path.getSegments();
        for (int s = 0; s < segments.size(); s++) {
            PathSegment segment = segments.get(s);
            double[] pedroX = new double[samples];
            double[] pedroY = new double[samples];
            for (int i = 0; i < samples; i++) {
                Pose point = segment.get((double) i / (samples - 1));
                pedroX[i] = point.x();
                pedroY[i] = point.y();
            }
            strokePedroPolyline(canvas, pedroX, pedroY);
        }
    }

    /**
     * Ticks showing the heading the follower is <em>supposed</em> to hold along the path, to compare against
     * the robot marker's own heading line. Each tick is nudged perpendicular to the path so it stays legible
     * where the planned heading runs parallel to the path instead of hiding inside the path line.
     */
    public static void drawPlannedHeading(Canvas canvas, Path path) {
        canvas.setStroke(COLOR_HEADING);
        int ticks = Math.max(1, headingTickCount);
        for (int i = 0; i <= ticks; i++) {
            double t = (double) i / ticks;
            // get(t) carries the interpolated heading goal; toField rotates position AND heading.
            Pose planned = toField(path.get(t));
            double heading = planned.heading();

            // Path normal in canvas space, from two nearby samples — avoids re-deriving the frame rotation.
            Pose before = toField(path.get(Math.max(0.0, t - 0.01)));
            Pose after = toField(path.get(Math.min(1.0, t + 0.01)));
            double nx = -(after.y() - before.y());
            double ny = after.x() - before.x();
            double norm = Math.hypot(nx, ny);
            double offsetX = norm > 1e-9 ? nx / norm * headingTickOffset : 0.0;
            double offsetY = norm > 1e-9 ? ny / norm * headingTickOffset : 0.0;

            double x = planned.x() + offsetX;
            double y = planned.y() + offsetY;
            canvas.strokeLine(x, y,
                    x + Math.cos(heading) * headingTickLength,
                    y + Math.sin(heading) * headingTickLength);
        }
    }

    public static void drawPoseHistory(Canvas canvas, PoseRing poseHistory) {
        canvas.setStroke(COLOR_HISTORY);
        strokePedroPolyline(canvas, poseHistory.xs(), poseHistory.ys());
    }
}
