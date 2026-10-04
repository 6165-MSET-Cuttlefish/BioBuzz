package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.pedropathing.utils.Angle;

import java.util.List;

/**
 * A Bezier by its control points (two for a line) with the robot's heading along it, either the tangent or a turn
 * spread evenly over its length. Sampled closely enough in position and heading that the robot's footprint can be
 * checked and its length measured before any Pedro path is built.
 */
final class Segment {
    static final double SAMPLE_SPACING_IN = 1.0;
    // A turn between samples swings the footprint's corners, so the heading changes at most this much per sample too.
    private static final double MAX_TURN_PER_SAMPLE = Math.toRadians(2);
    private static final int MIN_SAMPLES = 8;
    // Pedro takes anything tighter by spinning on the spot, which the footprint checks along the curve don't describe.
    static final double MIN_TURN_RADIUS_IN = 6;
    private static final int MAX_SAMPLES = 4096;

    final Pose[] controls;
    private final boolean tangent;
    private final double fromHeading;
    private final double turn;
    private double[] xs;
    private double[] ys;
    private double[] hs;
    private boolean tooLong;
    private double length;

    private Segment(boolean tangent, double fromHeading, double toHeading, Pose... controls) {
        if (controls.length < 2) throw new IllegalArgumentException("a segment needs at least two control points");
        if (controls.length == 2 && controls[0].distance(controls[1]) == 0) {
            throw new IllegalArgumentException("a line from " + controls[0] + " to itself");
        }
        this.controls = controls;
        this.tangent = tangent;
        this.fromHeading = Angle.normalize(fromHeading);
        this.turn = Angle.normalizeSigned(toHeading - fromHeading);
    }

    /** Facing along the curve. */
    static Segment tangent(Pose... controls) {
        return new Segment(true, 0, 0, controls);
    }

    /** Turning from {@code from} to {@code to} the short way, evenly over the length: Pedro's {@code linear} heading. */
    static Segment turning(double from, double to, Pose... controls) {
        return new Segment(false, from, to, controls);
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
        return length;
    }

    double startHeading() {
        sample();
        return hs[0];
    }

    double endHeading() {
        sample();
        return hs[hs.length - 1];
    }

    /** The direction the robot travels at the start, which is not its heading when backing up or strafing. */
    double startTravel() {
        return direction(controls[0], controls[1], controls);
    }

    double endTravel() {
        return direction(controls[controls.length - 2], controls[controls.length - 1], controls);
    }

    // A Bezier leaves along its first control edge and arrives along its last; a degenerate edge falls back to the chord.
    private static double direction(Pose a, Pose b, Pose[] controls) {
        double dx = b.x() - a.x();
        double dy = b.y() - a.y();
        if (dx * dx + dy * dy < 1e-18) {
            dx = controls[controls.length - 1].x() - controls[0].x();
            dy = controls[controls.length - 1].y() - controls[0].y();
        }
        return Angle.normalize(Math.atan2(dy, dx));
    }

    /**
     * The footprint inside {@code area} and more than {@code gap} from every obstacle at every sample, with the
     * heading, and the direction of travel, turning no faster than once round a {@link #MIN_TURN_RADIUS_IN} circle
     * per distance driven.
     */
    boolean fits(RobotShape robot, Region area, List<Obstacle> obstacles, double gap) {
        sample();
        if (tooLong || !gentle()) return false;
        // A Bezier stays inside its control points' box.
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (Pose p : controls) {
            minX = Math.min(minX, p.x());
            maxX = Math.max(maxX, p.x());
            minY = Math.min(minY, p.y());
            maxY = Math.max(maxY, p.y());
        }
        if (robot.canTurnAnywhereIn(minX, maxX, minY, maxY, area, obstacles, gap)) return true;
        for (int i = 0; i < xs.length; i++) {
            if (!robot.fits(xs[i], ys[i], hs[i], area, obstacles, gap)) return false;
        }
        return true;
    }

    private boolean gentle() {
        if (!tangent) return Math.abs(turn) <= length / MIN_TURN_RADIUS_IN + 1e-9 && travelsGently();
        for (int i = 1; i < xs.length; i++) {
            double ex = xs[i] - xs[i - 1];
            double ey = ys[i] - ys[i - 1];
            double step = Math.sqrt(ex * ex + ey * ey);
            if (Math.abs(Angle.normalizeSigned(hs[i] - hs[i - 1])) > step / MIN_TURN_RADIUS_IN + 1e-9) return false;
        }
        return true;
    }

    // Infinite where the curve stops dead.
    double[] cubicCurvatures(int points) {
        if (controls.length != 4) throw new IllegalStateException("not a cubic: " + controls.length + " control points");
        double ax = controls[1].x() - controls[0].x(), ay = controls[1].y() - controls[0].y();
        double bx = controls[2].x() - controls[1].x(), by = controls[2].y() - controls[1].y();
        double cx = controls[3].x() - controls[2].x(), cy = controls[3].y() - controls[2].y();
        double[] out = new double[points];
        for (int i = 0; i < points; i++) {
            double t = (double) i / (points - 1);
            double u = 1 - t;
            double dx = 3 * (u * u * ax + 2 * u * t * bx + t * t * cx);
            double dy = 3 * (u * u * ay + 2 * u * t * by + t * t * cy);
            double ddx = 6 * (u * (bx - ax) + t * (cx - bx));
            double ddy = 6 * (u * (by - ay) + t * (cy - by));
            double speed = Math.sqrt(dx * dx + dy * dy);
            out[i] = speed < 1e-9 ? Double.POSITIVE_INFINITY : Math.abs(dx * ddy - dy * ddx) / (speed * speed * speed);
        }
        return out;
    }

    double roughLength() {
        double polygon = 0;
        for (int i = 1; i < controls.length; i++) polygon += distance(controls[i - 1], controls[i]);
        return (polygon + 2 * distance(controls[0], controls[controls.length - 1])) / 3;
    }

    private static double distance(Pose a, Pose b) {
        double ex = b.x() - a.x();
        double ey = b.y() - a.y();
        return Math.sqrt(ex * ex + ey * ey);
    }

    private boolean travelsGently() {
        double before = Double.NaN;
        double stepBefore = 0;
        for (int i = 1; i < xs.length; i++) {
            double ex = xs[i] - xs[i - 1];
            double ey = ys[i] - ys[i - 1];
            double step = Math.sqrt(ex * ex + ey * ey);
            if (step < 1e-9) continue;
            double direction = Math.atan2(ey, ex);
            if (!Double.isNaN(before) && Math.abs(Angle.normalizeSigned(direction - before))
                    > (step + stepBefore) / 2 / MIN_TURN_RADIUS_IN + 1e-9) {
                return false;
            }
            before = direction;
            stepBefore = step;
        }
        return true;
    }

    double[] xs() {
        sample();
        return xs;
    }

    double[] ys() {
        sample();
        return ys;
    }

    double[] headings() {
        sample();
        return hs;
    }

    Path toPath() {
        Path path = Paths.path(controls.length == 2 ? new Line(controls[0], controls[1]) : new BezierCurve(controls));
        return tangent ? path.tangent() : path.linear(fromHeading, fromHeading + turn);
    }

    // Speed along a degree-n Bezier never exceeds n times its longest control-polygon edge, so this many
    // uniform steps in t are at most SAMPLE_SPACING_IN apart along the curve.
    private void sample() {
        if (xs != null) return;
        int degree = controls.length - 1;
        double longestEdge = 0;
        for (int i = 1; i < controls.length; i++) {
            longestEdge = Math.max(longestEdge, distance(controls[i], controls[i - 1]));
        }
        double needed = Math.max(Math.ceil(degree * longestEdge / SAMPLE_SPACING_IN),
                Math.ceil(Math.abs(turn) / MAX_TURN_PER_SAMPLE));
        int steps = Math.max(MIN_SAMPLES, (int) Math.min(needed, MAX_SAMPLES));
        sampleAt(steps);
        if (tangent) {
            double maxTurn = 0;
            for (int i = 1; i < hs.length; i++) {
                maxTurn = Math.max(maxTurn, Math.abs(Angle.normalizeSigned(hs[i] - hs[i - 1])));
            }
            if (maxTurn > MAX_TURN_PER_SAMPLE) {
                needed = Math.max(needed, Math.ceil(steps * maxTurn / MAX_TURN_PER_SAMPLE));
                steps = Math.max(steps, (int) Math.min(needed, MAX_SAMPLES));
                sampleAt(steps);
            }
        }
        tooLong = !(needed <= MAX_SAMPLES);
    }

    private void sampleAt(int steps) {
        int degree = controls.length - 1;
        xs = new double[steps + 1];
        ys = new double[steps + 1];
        hs = new double[steps + 1];
        double[] px = new double[controls.length];
        double[] py = new double[controls.length];
        int firstHeading = -1;
        for (int s = 0; s <= steps; s++) {
            double t = (double) s / steps;
            for (int i = 0; i < controls.length; i++) {
                px[i] = controls[i].x();
                py[i] = controls[i].y();
            }
            // De Casteljau stopped one level early: the two points left span the tangent at t.
            for (int level = degree; level > 1; level--) {
                for (int i = 0; i < level; i++) {
                    px[i] += t * (px[i + 1] - px[i]);
                    py[i] += t * (py[i + 1] - py[i]);
                }
            }
            double dx = px[1] - px[0];
            double dy = py[1] - py[0];
            xs[s] = px[0] + t * dx;
            ys[s] = py[0] + t * dy;
            if (tangent) {
                if (dx * dx + dy * dy < 1e-18) {
                    hs[s] = s == 0 ? Double.NaN : hs[s - 1];
                } else {
                    hs[s] = Angle.normalize(Math.atan2(dy, dx));
                    if (firstHeading < 0) firstHeading = s;
                }
            }
        }
        if (tangent) {
            if (firstHeading < 0) throw new IllegalArgumentException("a curve with no direction of travel: " + controls[0]);
            for (int s = 0; s < firstHeading; s++) hs[s] = hs[firstHeading];
        }
        double total = 0;
        double[] along = new double[steps + 1];
        for (int i = 1; i <= steps; i++) {
            double ex = xs[i] - xs[i - 1];
            double ey = ys[i] - ys[i - 1];
            total += Math.sqrt(ex * ex + ey * ey);
            along[i] = total;
        }
        length = total;
        if (!tangent) {
            for (int s = 0; s <= steps; s++) {
                hs[s] = Angle.normalize(fromHeading + turn * (total > 0 ? along[s] / total : (double) s / steps));
            }
        }
    }
}
