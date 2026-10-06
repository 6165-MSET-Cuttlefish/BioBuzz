package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.bezier.BezierCurve;

import java.util.ArrayList;
import java.util.List;

/**
 * The intake leg's curve: Pedro's spline from the start through only the balls the intake doesn't already sweep up
 * in passing, so a wide intake turns a bent curve into a straighter one.
 */
public final class IntakeCurvePlanner {
    private IntakeCurvePlanner() {}

    /**
     * The indices into {@code order} of the balls the curve must pass through, from (x0, y0). From each kept ball it
     * runs straight to the furthest ball it can reach while passing every ball in between within half the intake
     * width; those are swept up and dropped. The last ball always stays.
     */
    public static int[] essentialStops(double x0, double y0, Ball[] order, double intakeWidthIn) {
        int[] stops = new int[order.length];
        int count = 0;
        Pose anchor = new Pose(x0, y0, 0);
        int next = 0;
        while (next < order.length) {
            int reach = next;
            for (int j = order.length - 1; j > next; j--) {
                if (sweepsAll(anchor, order, next, j, intakeWidthIn)) {
                    reach = j;
                    break;
                }
            }
            anchor = order[reach].toPose();
            stops[count++] = reach;
            next = reach + 1;
        }
        int[] kept = new int[count];
        System.arraycopy(stops, 0, kept, 0, count);
        return kept;
    }

    /**
     * Control points of Pedro's spline from {@code start} through every stop in turn: a line for one stop, otherwise
     * {@link BezierCurve#through}, one Bezier passing each point at evenly spaced t.
     */
    public static Pose[] throughCurve(Pose start, List<Pose> stops) {
        List<Pose> points = new ArrayList<>(stops.size() + 1);
        points.add(start);
        points.addAll(stops);
        return through(points);
    }

    public static Pose[] through(List<Pose> points) {
        if (points.size() == 2) {
            Pose a = points.get(0);
            Pose b = points.get(1);
            return new Pose[]{new Pose(a.x(), a.y(), 0), new Pose(b.x(), b.y(), 0)};
        }
        List<Vector2D> controls = BezierCurve.through(points.toArray(new Pose[0])).getControlPoints();
        Pose[] out = new Pose[controls.size()];
        for (int i = 0; i < out.length; i++) out[i] = new Pose(controls.get(i).x(), controls.get(i).y(), 0);
        return out;
    }

    private static boolean sweepsAll(Pose from, Ball[] order, int first, int to, double intakeWidthIn) {
        Pose end = order[to].toPose();
        for (int i = first; i < to; i++) {
            if (!sweptUp(from, end, order[i].toPose(), intakeWidthIn)) return false;
        }
        return true;
    }

    private static boolean sweptUp(Pose a, Pose b, Pose ball, double intakeWidthIn) {
        double dx = b.x() - a.x();
        double dy = b.y() - a.y();
        double lenSq = dx * dx + dy * dy;
        if (lenSq < 1e-9) return false;

        double t = ((ball.x() - a.x()) * dx + (ball.y() - a.y()) * dy) / lenSq;
        if (t < 0 || t > 1) return false;
        return Math.hypot(ball.x() - (a.x() + t * dx), ball.y() - (a.y() + t * dy)) <= intakeWidthIn / 2;
    }
}
