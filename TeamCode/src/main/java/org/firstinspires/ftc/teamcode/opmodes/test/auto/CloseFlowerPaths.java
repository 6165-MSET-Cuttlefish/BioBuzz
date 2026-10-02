package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.PathHeadings;

/**
 * The CloseFlower route shared by {@link CloseFlowerAuto} and {@link CloseFlowerLinearAuto}, authored for RED and
 * rotated to BLUE through {@link FieldPose}; construct it once the alliance and field symmetry are set.
 * docs/paths/CloseFlowerAuto.pp is the same route in the visualizer.
 */
public final class CloseFlowerPaths {
    public final Pose start = pose(56, 8, 90);

    private final Pose closeFlower = pose(14.9769, 46.9989, 180);
    private final Pose closeFlowerControl1 = point(56, 23);
    private final Pose closeFlowerControl2 = point(34.9769, 46.9989);
    private final Pose path2Turn = point(29.0321, 76.6058);
    private final Pose path2TurnControl1 = point(23.0138, 63.2743);
    private final Pose path2 = pose(46.9748, 130.0998, 90.091);
    private final Pose path2Control1 = point(47.087, 116.7758);
    private final Pose point4Facing = pose(57.0019, 101.165, 90.5);
    private final Pose point4 = pose(57.0019, 101.165, 270);
    private final Pose point5 = pose(45.3325, 8.4566, -176.9646);
    private final Pose point5Control1 = point(61.5713, 0.6836);
    private final Pose point5Control2 = point(77.8952, 10.2674);
    private final Pose point6 = pose(8.3412, 8.3306, -179.8049);

    private static Pose pose(double x, double y, double headingDeg) {
        return FieldPose.forAlliance(x, y, Math.toRadians(headingDeg));
    }

    private static Pose point(double x, double y) {
        return FieldPose.forAlliance(x, y, 0);
    }

    /** The legs in driving order. */
    public Path[] all() {
        return new Path[] {closeFlower(), path2(), path4(), path5(), path6()};
    }

    public Path closeFlower() {
        return curve(start, closeFlowerControl1, closeFlowerControl2, closeFlower).tangent();
    }

    /** Turns off the close flower over the first curve, then follows the tangent through the second without stopping. */
    public Path path2() {
        Path turn = curve(closeFlower, path2TurnControl1, path2Turn);
        Path whole = path(turn, curve(path2Turn, path2Control1, path2));
        return PathHeadings.linearThenTangent(
                whole, PathHeadings.endTangent(closeFlower()), turn.curve.length() / whole.curve.length());
    }

    public Path path4() {
        return line(path2, point4).heading(Interpolator.piecewise()
                .until(0.5, Interpolator.constant(point4Facing))
                .until(1, Interpolator.linear(point4Facing, point4)));
    }

    public Path path5() {
        return curve(point4, point5Control1, point5Control2, point5).tangent();
    }

    public Path path6() {
        return line(point5, point6).tangent();
    }
}
