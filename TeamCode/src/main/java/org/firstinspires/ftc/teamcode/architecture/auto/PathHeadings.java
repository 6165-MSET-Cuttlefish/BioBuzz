package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.utils.Angle;

/**
 * Tangent headings of Pedro paths, for a heading that stays continuous where two segments, or the two parts of one
 * path's heading, meet: a jump there makes the follower turn before it drives on. Positions are fractions of the
 * path's length, the unit {@code Interpolator.piecewise().until(...)} takes. Headings are radians in [0, 2π), taken
 * from the built curve, so they are already right for either alliance: don't map them through {@link FieldPose}.
 */
public final class PathHeadings {
    private PathHeadings() {}

    // A Bezier whose end control points coincide has no derivative there; the direction just inside it is the limit.
    private static final double[] NUDGES = {0, 1e-4, 1e-3, 1e-2};
    private static final double MIN_DERIVATIVE = 1e-9;
    // A switch on a join of Paths.path(a, b) rounds onto either curve, so each side reads this far into its own.
    private static final double JOIN_INSET = 1e-9;

    /** The direction of travel {@code fraction} of the way along {@code path}; on a corner between two curves, ask the curve itself. */
    public static double tangentAt(Path path, double fraction) {
        checkFraction(fraction);
        return directionAt(path.curve, path.curve.parameter(fraction), 0, 1);
    }

    public static double startTangent(Path path) {
        return directionAt(path.curve, 0, 0, 1);
    }

    public static double endTangent(Path path) {
        return directionAt(path.curve, 1, 0, 1);
    }

    /** The heading change the follower is asked for where {@code after} takes over from {@code before}; 0 is seamless. */
    public static double jump(Path before, Path after) {
        return Angle.normalizeSigned(after.heading(0) - before.heading(1));
    }

    /**
     * Turns from {@code startHeading} onto the tangent by {@code switchAt}, then follows the tangent to the end.
     * Not built on {@code Interpolator.piecewise()}: in Pedro 3.0.1 its tangent piece replays the whole curve's
     * tangent, and on some {@code Paths.path(a, b)} it throws when the follower starts the path.
     */
    public static Path linearThenTangent(Path path, double startHeading, double switchAt) {
        double switchT = switchParameter(path, switchAt);
        double tangentFrom = Math.min(1, switchT + JOIN_INSET);
        double start = Angle.normalize(startHeading);
        double turn = Angle.error(start, directionAt(path.curve, tangentFrom, tangentFrom, 1));
        return path.heading((curve, t) -> t >= switchT
                ? directionAt(curve, t, tangentFrom, 1)
                : Angle.normalize(start + turn * Math.max(0, t) / switchT));
    }

    /** Follows the tangent until {@code switchAt}, then turns from it to {@code endHeading} by the end. */
    public static Path tangentThenLinear(Path path, double switchAt, double endHeading) {
        double switchT = switchParameter(path, switchAt);
        double tangentTo = Math.max(0, switchT - JOIN_INSET);
        double start = directionAt(path.curve, tangentTo, 0, tangentTo);
        double turn = Angle.error(start, Angle.normalize(endHeading));
        return path.heading((curve, t) -> t <= switchT
                ? directionAt(curve, t, 0, tangentTo)
                : Angle.normalize(start + turn * Math.min(1, (t - switchT) / (1 - switchT))));
    }

    /** The direction at {@code t} held inside {@code from..to}, looking further inside that range where the curve stalls. */
    private static double directionAt(Curve curve, double t, double from, double to) {
        double at = Math.max(from, Math.min(to, t));
        boolean roomAhead = at - from <= to - at;
        for (double nudge : NUDGES) {
            Vector2D derivative = curve.derivative(roomAhead ? Math.min(to, at + nudge) : Math.max(from, at - nudge));
            // Not Vector2D.theta(): it reads anything shorter than about 3e-5 as 0 rad.
            if (derivative.magnitude() > MIN_DERIVATIVE) {
                return Angle.normalize(Math.atan2(derivative.y(), derivative.x()));
            }
        }
        throw new IllegalArgumentException(String.format("the curve has no direction of travel near t = %s", t));
    }

    private static double switchParameter(Path path, double switchAt) {
        if (!(switchAt > 0 && switchAt < 1)) {
            throw new IllegalArgumentException(String.format("switch fraction %s must be between 0 and 1", switchAt));
        }
        double switchT = path.curve.parameter(switchAt);
        if (!(switchT > 0 && switchT < 1)) {
            throw new IllegalArgumentException(String.format("switch fraction %s lands on an end of the path", switchAt));
        }
        return switchT;
    }

    private static void checkFraction(double fraction) {
        if (!(fraction >= 0 && fraction <= 1)) {
            throw new IllegalArgumentException(String.format("path fraction %s is outside 0..1", fraction));
        }
    }
}
