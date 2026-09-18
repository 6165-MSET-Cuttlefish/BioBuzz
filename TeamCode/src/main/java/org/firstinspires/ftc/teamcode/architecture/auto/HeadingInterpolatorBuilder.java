package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds piecewise {@link Interpolator}s over curve-parameter ranges, tracking the running t so
 * chained segments don't have to repeat their start-t.
 */
public class HeadingInterpolatorBuilder {

    private static final class Node {
        final double startT;
        final double endT;
        final Interpolator interpolator;
        final boolean rescaleT;

        Node(double startT, double endT, Interpolator interpolator, boolean rescaleT) {
            this.startT = startT;
            this.endT = endT;
            this.interpolator = interpolator;
            this.rescaleT = rescaleT;
        }
    }

    private final List<Node> nodes = new ArrayList<>();
    private double currentT = 0;

    /**
     * Sweeps start to end over the whole curve by raw t. Pedro's own {@code Interpolator.linear}
     * sweeps by {@code curve.pathCompletion}, which 3.0.0 leaves inverted on {@code Line} and
     * {@code CompoundCurve} — it hands back the end heading at t=0.
     */
    public static Interpolator sweep(double startHeadingRad, double endHeadingRad) {
        double start = Angle.normalize(startHeadingRad);
        double end = Angle.normalize(endHeadingRad);
        // Not Angle.error: it normalizes an exact 180 to -PI, flipping which way the robot turns.
        double delta = Angle.turnDirection(start, end) * Angle.smallestDifference(end, start);
        return (curve, t) -> Angle.normalize(start + delta * t);
    }

    /** {@link #sweep} the long way around the circle. */
    static Interpolator reversedSweep(double startHeadingRad, double endHeadingRad) {
        double start = Angle.normalize(startHeadingRad);
        double end = Angle.normalize(endHeadingRad);
        double delta = -Angle.turnDirection(start, end) * (2 * Math.PI - Angle.smallestDifference(start, end));
        return (curve, t) -> Angle.normalize(start + delta * t);
    }

    public HeadingInterpolatorBuilder linear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
        return add(startT, endT, sweep(startHeadingRad, endHeadingRad), true);
    }

    public HeadingInterpolatorBuilder linear(double endT, double startHeadingRad, double endHeadingRad) {
        return linear(currentT, endT, startHeadingRad, endHeadingRad);
    }

    public HeadingInterpolatorBuilder reversedLinear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
        return add(startT, endT, reversedSweep(startHeadingRad, endHeadingRad), true);
    }

    public HeadingInterpolatorBuilder reversedLinear(double endT, double startHeadingRad, double endHeadingRad) {
        return reversedLinear(currentT, endT, startHeadingRad, endHeadingRad);
    }

    public HeadingInterpolatorBuilder constant(double startT, double endT, double headingRad) {
        return add(startT, endT, Interpolator.constant(headingRad), false);
    }

    public HeadingInterpolatorBuilder constant(double endT, double headingRad) {
        return constant(currentT, endT, headingRad);
    }

    public HeadingInterpolatorBuilder tangent(double startT, double endT) {
        return add(startT, endT, Interpolator.tangent, false);
    }

    public HeadingInterpolatorBuilder tangent(double endT) {
        return tangent(currentT, endT);
    }

    public HeadingInterpolatorBuilder facingPoint(double startT, double endT, double x, double y) {
        return add(startT, endT, Interpolator.facingPoint(Vector2D.cartesian(x, y)), false);
    }

    public HeadingInterpolatorBuilder facingPoint(double endT, double x, double y) {
        return facingPoint(currentT, endT, x, y);
    }

    public HeadingInterpolatorBuilder facingPoint(double startT, double endT, Pose pose) {
        return add(startT, endT, Interpolator.facingPoint(pose), false);
    }

    public HeadingInterpolatorBuilder facingPoint(double endT, Pose pose) {
        return facingPoint(currentT, endT, pose);
    }

    /**
     * The interpolator sees the node's range as a whole path (t rescaled to [0,1]), so one that
     * derives a position from t — {@code tangent}, {@code facingPoint} — samples the wrong point.
     * Use {@link #tangent} / {@link #facingPoint} for those.
     */
    public HeadingInterpolatorBuilder custom(double startT, double endT, Interpolator interpolator) {
        return add(startT, endT, interpolator, true);
    }

    public HeadingInterpolatorBuilder custom(double endT, Interpolator interpolator) {
        return custom(currentT, endT, interpolator);
    }

    private HeadingInterpolatorBuilder add(double startT, double endT, Interpolator interpolator, boolean rescaleT) {
        nodes.add(new Node(startT, endT, interpolator, rescaleT));
        currentT = endT;
        return this;
    }

    public Interpolator build() {
        if (nodes.isEmpty()) {
            throw new IllegalStateException("HeadingInterpolatorBuilder: no segments added");
        }
        for (int i = 0; i < nodes.size(); i++) {
            Node n = nodes.get(i);
            if (n.endT <= n.startT) {
                throw new IllegalStateException(
                        "HeadingInterpolatorBuilder: segment " + i + " has endT ("
                                + n.endT + ") <= startT (" + n.startT + ")");
            }
            if (i > 0 && n.startT < nodes.get(i - 1).endT) {
                throw new IllegalStateException(
                        "HeadingInterpolatorBuilder: segment " + i + " starts at "
                                + n.startT + " before previous segment ends at " + nodes.get(i - 1).endT
                                + " — add segments in ascending t order");
            }
        }
        Node[] built = nodes.toArray(new Node[0]);
        return (curve, t) -> {
            for (Node n : built) {
                if (t >= n.startT && t <= n.endT) {
                    double nodeT = n.rescaleT
                            ? Math.max(0.0, Math.min(1.0, (t - n.startT) / (n.endT - n.startT)))
                            : t;
                    return n.interpolator.interpolate(curve, nodeT);
                }
            }
            return Interpolator.tangent.interpolate(curve, t);
        };
    }

    public double getCurrentT() {
        return currentT;
    }
}
