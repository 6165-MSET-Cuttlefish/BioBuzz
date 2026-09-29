package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

/** The intake leg's curve: through only the balls a full-width intake doesn't sweep up in passing. */
public final class IntakeCurvePlanner {
    // Keeps neighbouring curve parameters apart so the control-point solve stays well conditioned.
    private static final double MIN_PARAM_GAP = 0.1;

    private IntakeCurvePlanner() {}

    /**
     * The balls, in visit order, that the curve must pass through. From each kept stop it runs straight to the
     * furthest ball it can reach while passing every ball in between within half the intake width; those are
     * swept up and dropped. The last ball always stays.
     */
    public static List<Pose> essentialStops(Pose start, Ball[] order, double intakeWidthIn) {
        List<Pose> stops = new ArrayList<>();
        Pose anchor = start;
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
            stops.add(anchor);
            next = reach + 1;
        }
        return stops;
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

    /**
     * The lowest-degree Bezier through every point, in order: a line for two, otherwise a degree-(n-1) curve
     * whose interior control points are solved so it passes through each point. Neighbouring points must differ.
     */
    static Segment through(List<Pose> points) {
        int n = points.size() - 1;
        if (n < 1) throw new IllegalArgumentException("a curve needs at least two points");
        for (int i = 1; i <= n; i++) {
            if (points.get(i).distance(points.get(i - 1)) < 1e-9) {
                throw new IllegalArgumentException("coincident neighbouring points " + points.get(i));
            }
        }
        if (n == 1) return new Segment(points.get(0), points.get(1));

        Pose[] p = points.toArray(new Pose[0]);
        double[] t = chordLengthParams(p);

        int m = n - 1;
        double[][] a = new double[m][m];
        double[] xs = new double[m];
        double[] ys = new double[m];
        for (int row = 0; row < m; row++) {
            double ti = t[row + 1];
            double b0 = bernstein(n, 0, ti);
            double bn = bernstein(n, n, ti);
            xs[row] = p[row + 1].x() - b0 * p[0].x() - bn * p[n].x();
            ys[row] = p[row + 1].y() - b0 * p[0].y() - bn * p[n].y();
            for (int col = 0; col < m; col++) a[row][col] = bernstein(n, col + 1, ti);
        }
        solve(a, xs, ys);

        Pose[] controls = new Pose[n + 1];
        controls[0] = p[0];
        controls[n] = p[n];
        for (int i = 0; i < m; i++) controls[i + 1] = new Pose(xs[i], ys[i], 0);
        return new Segment(controls);
    }

    // Chord-length parameters rather than even spacing, which swings unevenly spaced
    // balls into a wider loop than the layout needs.
    private static double[] chordLengthParams(Pose[] points) {
        int n = points.length - 1;
        double[] t = new double[n + 1];
        for (int i = 1; i <= n; i++) t[i] = t[i - 1] + points[i - 1].distance(points[i]);
        double total = t[n];
        for (int i = 1; i <= n; i++) t[i] = total > 1e-9 ? t[i] / total : (double) i / n;
        t[n] = 1;
        double gap = Math.min(MIN_PARAM_GAP, 0.5 / n);
        for (int i = 1; i < n; i++) t[i] = Math.max(t[i], t[i - 1] + gap);
        for (int i = n - 1; i >= 1; i--) t[i] = Math.min(t[i], t[i + 1] - gap);
        return t;
    }

    private static double bernstein(int n, int j, double t) {
        double binomial = 1;
        for (int i = 1; i <= j; i++) binomial = binomial * (n - j + i) / i;
        return binomial * Math.pow(t, j) * Math.pow(1 - t, n - j);
    }

    /** Solves {@code a·x = xs} and {@code a·y = ys} in place, leaving x in xs and y in ys. */
    private static void solve(double[][] a, double[] xs, double[] ys) {
        int m = a.length;
        for (int col = 0; col < m; col++) {
            int pivot = col;
            for (int r = col + 1; r < m; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            }
            double[] row = a[col]; a[col] = a[pivot]; a[pivot] = row;
            double x = xs[col]; xs[col] = xs[pivot]; xs[pivot] = x;
            double y = ys[col]; ys[col] = ys[pivot]; ys[pivot] = y;

            for (int r = col + 1; r < m; r++) {
                double f = a[r][col] / a[col][col];
                for (int c = col; c < m; c++) a[r][c] -= f * a[col][c];
                xs[r] -= f * xs[col];
                ys[r] -= f * ys[col];
            }
        }
        for (int r = m - 1; r >= 0; r--) {
            for (int c = r + 1; c < m; c++) {
                xs[r] -= a[r][c] * xs[c];
                ys[r] -= a[r][c] * ys[c];
            }
            xs[r] /= a[r][r];
            ys[r] /= a[r][r];
        }
    }
}
