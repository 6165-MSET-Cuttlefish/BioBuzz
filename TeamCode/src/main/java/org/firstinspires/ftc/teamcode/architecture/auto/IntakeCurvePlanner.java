package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

/** Which balls the intake leg must aim the intake at: the rest it sweeps up in passing. */
public final class IntakeCurvePlanner {
    private IntakeCurvePlanner() {}

    /**
     * The indices into {@code order} of the balls the intake must be aimed at, from the intake at (x0, y0). From each
     * kept ball it runs straight to the furthest ball it can reach while passing every ball in between within half
     * the intake width; those are swept up and dropped. The last ball always stays.
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
