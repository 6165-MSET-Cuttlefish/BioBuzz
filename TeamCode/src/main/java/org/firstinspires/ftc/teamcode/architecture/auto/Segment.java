package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.curves.bezier.BezierCurve;

import java.util.List;

/**
 * A Bezier by its control points (two for a line), sampled at most {@link #SAMPLE_SPACING_IN} apart along the
 * curve so a candidate can be checked and measured before any Pedro curve is built.
 */
final class Segment {
    static final double SAMPLE_SPACING_IN = 1.0;
    private static final int MIN_SAMPLES = 8;
    private static final int MAX_SAMPLES = 4096;

    final Pose[] controls;
    private double[] xs;
    private double[] ys;
    private boolean tooLong;
    private double length = -1;

    Segment(Pose... controls) {
        if (controls.length < 2) throw new IllegalArgumentException("a segment needs at least two control points");
        this.controls = controls;
    }

    Pose start() {
        return controls[0];
    }

    Pose end() {
        return controls[controls.length - 1];
    }

    /** Polyline length through the samples; Pedro's BezierCurve.length() can stop subdividing early on a loop. */
    double length() {
        sample();
        if (length < 0) {
            double total = 0;
            for (int i = 1; i < xs.length; i++) total += Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
            length = total;
        }
        return length;
    }

    /** Every sample inside {@code keepIn} and farther than {@code clearance} from every obstacle's edge. */
    boolean staysClear(Region keepIn, List<Obstacle> obstacles, double clearance) {
        if (!staysInside(keepIn)) return false;
        for (int i = 0; i < xs.length; i++) {
            for (Obstacle o : obstacles) {
                if (Math.hypot(xs[i] - o.x, ys[i] - o.y) <= o.radius + clearance) return false;
            }
        }
        return true;
    }

    boolean staysInside(Region keepIn) {
        sample();
        if (tooLong) return false;
        for (int i = 0; i < xs.length; i++) {
            if (!keepIn.contains(xs[i], ys[i])) return false;
        }
        return true;
    }

    /** Direction of travel at the end, from the last two samples. */
    double endHeading() {
        sample();
        int last = xs.length - 1;
        return Math.atan2(ys[last] - ys[last - 1], xs[last] - xs[last - 1]);
    }

    Curve toCurve() {
        return controls.length == 2 ? new Line(controls[0], controls[1]) : new BezierCurve(controls);
    }

    // Speed along a degree-n Bezier never exceeds n times its longest control-polygon edge, so this many
    // uniform steps in t are at most SAMPLE_SPACING_IN apart along the curve.
    private void sample() {
        if (xs != null) return;
        int degree = controls.length - 1;
        double longestEdge = 0;
        for (int i = 1; i < controls.length; i++) {
            longestEdge = Math.max(longestEdge, controls[i].distance(controls[i - 1]));
        }
        double needed = Math.ceil(degree * longestEdge / SAMPLE_SPACING_IN);
        tooLong = !(needed <= MAX_SAMPLES);
        int steps = tooLong ? MAX_SAMPLES : Math.max(MIN_SAMPLES, (int) needed);

        xs = new double[steps + 1];
        ys = new double[steps + 1];
        double[] px = new double[controls.length];
        double[] py = new double[controls.length];
        for (int s = 0; s <= steps; s++) {
            double t = (double) s / steps;
            for (int i = 0; i < controls.length; i++) {
                px[i] = controls[i].x();
                py[i] = controls[i].y();
            }
            for (int level = degree; level > 0; level--) {
                for (int i = 0; i < level; i++) {
                    px[i] += t * (px[i + 1] - px[i]);
                    py[i] += t * (py[i + 1] - py[i]);
                }
            }
            xs[s] = px[0];
            ys[s] = py[0];
        }
    }
}
