package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

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
    private final Pose toCloseFlowerControl1 = point(56, 23);
    private final Pose toCloseFlowerControl2 = point(34.9769, 46.9989);
    private final Pose farFlower = point(44.0352, 127.1352);
    private final Pose toFarFlowerControl1 = point(30.8911, 65.9001);
    private final Pose toFarFlowerControl2 = point(24.5773, 86.0907);
    private final Pose hive = point(61.3781, 71.1145);
    private final Pose toHiveControl1 = point(48.4909, 113.2044);
    private final Pose toHiveControl2 = point(69.2810, 106.5252);
    private final Pose audienceWall = pose(45.3325, 8.4566, -176.9646);
    private final Pose toAudienceWallControl1 = point(61.5713, 0.6836);
    private final Pose toAudienceWallControl2 = point(77.9453, 8.5677);
    private final Pose corner = pose(8.3412, 8.3306, -179.8049);

    private static Pose pose(double x, double y, double headingDeg) {
        return FieldPose.forAlliance(x, y, Math.toRadians(headingDeg));
    }

    private static Pose point(double x, double y) {
        return FieldPose.forAlliance(x, y, 0);
    }

    /** The legs in driving order. */
    public Path[] all() {
        return new Path[] {toCloseFlower(), toFarFlower(), toHive(), toAudienceWall(), toCorner()};
    }

    public Path toCloseFlower() {
        return curve(start, toCloseFlowerControl1, toCloseFlowerControl2, closeFlower).tangent();
    }

    public Path toFarFlower() {
        return PathHeadings.linearThenTangent(curve(closeFlower, toFarFlowerControl1, toFarFlowerControl2, farFlower),
                PathHeadings.endTangent(toCloseFlower()), 0.3);
    }

    public Path toHive() {
        return PathHeadings.holdThenTangent(curve(farFlower, toHiveControl1, toHiveControl2, hive),
                PathHeadings.endTangent(toFarFlower()), 0.1, 0.4);
    }

    // Its last control point is on the line to the corner, so it arrives facing along toCorner.
    public Path toAudienceWall() {
        return PathHeadings.linearThenTangent(curve(hive, toAudienceWallControl1, toAudienceWallControl2, audienceWall),
                PathHeadings.endTangent(toHive()), 0.2);
    }

    public Path toCorner() {
        return line(audienceWall, corner).tangent();
    }
}
