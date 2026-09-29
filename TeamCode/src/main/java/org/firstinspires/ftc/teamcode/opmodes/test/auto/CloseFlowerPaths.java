package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;

/**
 * The CloseFlower route shared by {@link CloseFlowerAuto} and {@link CloseFlowerLinearAuto}, authored for RED and
 * rotated to BLUE through {@link FieldPose}; construct it once the alliance and field symmetry are set.
 * docs/paths/CloseFlowerAuto.pp is the same route in the visualizer.
 */
public final class CloseFlowerPaths {
    public final Pose start = pose(56, 8, 90);

    private final Pose path1 = pose(14.9769, 46.9989, 180);
    private final Pose path1Control1 = point(56, 23);
    private final Pose path1Control2 = point(34.9769, 46.9989);
    private final Pose point2Turn = pose(29.0321, 76.6058, 65.7039);
    private final Pose point2TurnControl1 = point(23.0138, 63.2743);
    private final Pose point2 = pose(46.9748, 130.0998, 90.4762);
    private final Pose point2Control1 = point(47.087, 116.6002);
    private final Pose point3 = pose(47.0588, 119.9926, 90.4762);
    private final Pose point4 = pose(12.8193, 116.458, -90.0072);
    private final Pose point4Control1 = point(46.9508, 132.9922);
    private final Pose point4Control2 = point(12.8213, 132.458);
    private final Pose point5 = pose(12.8172, 99.8613, -90.0072);
    private final Pose point6 = pose(20.8141, 11.6828, -178.1107);
    private final Pose point6Control1 = point(12.8153, 84.8613);
    private final Pose point6Control2 = point(31.0646, 65.2041);
    private final Pose point6Control3 = point(38.8643, 12.2782);
    private final Pose point7 = pose(10.8466, 11.354, -178.1107);

    private static Pose pose(double x, double y, double headingDeg) {
        return FieldPose.forAlliance(x, y, Math.toRadians(headingDeg));
    }

    private static Pose point(double x, double y) {
        return FieldPose.forAlliance(x, y, 0);
    }

    /** Paths 1 to 7 in driving order. */
    public Path[] all() {
        return new Path[] {path1(), path2(), path3(), path4(), path5(), path6(), path7()};
    }

    public Path path1() {
        return curve(start, path1Control1, path1Control2, path1).tangent();
    }

    public Path path2() {
        // Leaves the left flower still facing it, so the first stretch turns onto the curve's tangent.
        return path(
                curve(path1, point2TurnControl1, point2Turn).linear(path1, point2Turn),
                curve(point2Turn, point2Control1, point2).tangent());
    }

    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4).tangent();
    }

    public Path path5() {
        return line(point4, point5).tangent();
    }

    public Path path6() {
        return curve(point5, point6Control1, point6Control2, point6Control3, point6).tangent();
    }

    public Path path7() {
        return line(point6, point7).tangent();
    }
}
