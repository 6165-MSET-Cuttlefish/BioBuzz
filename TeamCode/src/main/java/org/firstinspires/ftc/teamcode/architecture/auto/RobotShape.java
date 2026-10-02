package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

import java.util.List;

/**
 * The robot as the ball planner sees it, in inches: a {@code lengthIn} by {@code widthIn} frame centred on the
 * centre of rotation, length along the heading, and an intake {@code intakeWidthIn} wide across the heading whose
 * middle is {@code intakeOffsetIn} ahead of that centre. The footprint the planner keeps clear is the rectangle
 * around both, so an intake may stick out past the frame. A ball is collected when the intake passes over it
 * driving forward.
 */
public final class RobotShape {
    public final double lengthIn;
    public final double widthIn;
    public final double intakeOffsetIn;
    public final double intakeWidthIn;
    private final double front;
    private final double back;
    private final double halfWidth;
    private final double turnRadius;

    public RobotShape(double lengthIn, double widthIn, double intakeOffsetIn, double intakeWidthIn) {
        if (!(lengthIn > 0 && widthIn > 0 && intakeWidthIn > 0 && intakeOffsetIn >= 0)
                || !Double.isFinite(lengthIn + widthIn + intakeOffsetIn + intakeWidthIn)) {
            throw new IllegalArgumentException(String.format(
                    "robot %.1f x %.1f in with an intake %.1f in wide %.1f in ahead: sizes must be > 0 and the offset >= 0",
                    lengthIn, widthIn, intakeWidthIn, intakeOffsetIn));
        }
        this.lengthIn = lengthIn;
        this.widthIn = widthIn;
        this.intakeOffsetIn = intakeOffsetIn;
        this.intakeWidthIn = intakeWidthIn;
        back = lengthIn / 2;
        front = Math.max(back, intakeOffsetIn);
        halfWidth = Math.max(widthIn, intakeWidthIn) / 2;
        turnRadius = Math.sqrt(front * front + halfWidth * halfWidth);
    }

    /** The farthest the footprint reaches from its centre: the circle a turn in place sweeps. */
    public double turnRadius() {
        return turnRadius;
    }

    /** The centre that puts the point {@code side} inches left of the intake's middle on (x, y), facing {@code heading}. */
    public Pose centreFor(double x, double y, double heading, double side) {
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        return new Pose(x - intakeOffsetIn * c + side * s, y - intakeOffsetIn * s - side * c, heading);
    }

    public Pose intakeAt(Pose centre) {
        double h = centre.heading();
        return new Pose(centre.x() + intakeOffsetIn * Math.cos(h), centre.y() + intakeOffsetIn * Math.sin(h), h);
    }

    /** Whether (x, y) lies under the footprint of a robot at {@code centre}. */
    public boolean covers(Pose centre, double x, double y) {
        return under(centre.x(), centre.y(), Math.cos(centre.heading()), Math.sin(centre.heading()), x, y);
    }

    private boolean under(double x, double y, double c, double s, double px, double py) {
        double dx = px - x;
        double dy = py - y;
        double along = dx * c + dy * s;
        return along <= front && along >= -back && Math.abs(-dx * s + dy * c) <= halfWidth;
    }

    /** The footprint's corners at {@code centre}: x then y, front left first, counterclockwise. */
    public double[][] corners(Pose centre) {
        double c = Math.cos(centre.heading());
        double s = Math.sin(centre.heading());
        double[][] along = {{front, halfWidth}, {-back, halfWidth}, {-back, -halfWidth}, {front, -halfWidth}};
        double[][] out = new double[2][4];
        for (int i = 0; i < 4; i++) {
            out[0][i] = centre.x() + along[i][0] * c - along[i][1] * s;
            out[1][i] = centre.y() + along[i][0] * s + along[i][1] * c;
        }
        return out;
    }

    /** Whether the footprint at (x, y) facing {@code heading} is inside {@code area} and more than {@code gap} from every obstacle. */
    public boolean fits(double x, double y, double heading, Region area, List<Obstacle> obstacles, double gap) {
        return clash(x, y, heading, area, obstacles, gap) == NONE;
    }

    /** Null if the footprint fits as {@link #fits} checks, otherwise what it touches. */
    public String footprintProblem(double x, double y, double heading, Region area, List<Obstacle> obstacles,
                                   double gap) {
        int clash = clash(x, y, heading, area, obstacles, gap);
        if (clash == NONE) return null;
        if (clash == OUTSIDE) return "over the edge of " + area;
        Obstacle o = obstacles.get(clash);
        return covers(new Pose(x, y, heading), (o.minX + o.maxX) / 2, (o.minY + o.maxY) / 2)
                ? "on top of " + o : String.format("within %.1f in of %s", gap, o);
    }

    private static final int NONE = -1;
    private static final int OUTSIDE = -2;

    // NONE, OUTSIDE, or the index of the obstacle it comes too close to; called for every sample of every candidate, so no allocation.
    private int clash(double x, double y, double heading, Region area, List<Obstacle> obstacles, double gap) {
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double fx = front * c;
        double fy = front * s;
        double bx = back * c;
        double by = back * s;
        double wx = -halfWidth * s;
        double wy = halfWidth * c;
        double[] xs = {x + fx + wx, x - bx + wx, x - bx - wx, x + fx - wx};
        double[] ys = {y + fy + wy, y - by + wy, y - by - wy, y + fy - wy};
        for (int i = 0; i < 4; i++) {
            if (!area.contains(xs[i], ys[i])) return OUTSIDE;
        }
        double reach = turnRadius + gap;
        for (int k = 0; k < obstacles.size(); k++) {
            Obstacle o = obstacles.get(k);
            if (!o.blocks(x, y, reach)) continue;
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                if (o.intersectsSegment(xs[i], ys[i], xs[j], ys[j], gap)) return k;
            }
            // An obstacle smaller than the footprint can sit under it clear of every edge.
            if (under(x, y, c, s, (o.minX + o.maxX) / 2, (o.minY + o.maxY) / 2)) return k;
        }
        return NONE;
    }

    /** Whether the robot can turn in place at (x, y): the {@link #turnRadius} circle is inside {@code area} and more than {@code gap} from every obstacle. */
    public boolean canTurnAt(double x, double y, Region area, List<Obstacle> obstacles, double gap) {
        double r = turnRadius;
        if (x - r < area.minX || x + r > area.maxX || y - r < area.minY || y + r > area.maxY) return false;
        for (Obstacle o : obstacles) {
            if (o.blocks(x, y, r + gap)) return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return String.format("%.1f x %.1f in, intake %.1f in wide %.1f in ahead", lengthIn, widthIn, intakeWidthIn,
                intakeOffsetIn);
    }
}
