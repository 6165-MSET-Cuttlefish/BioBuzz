package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

/** An axis-aligned rectangle in Pedro field inches; the caller decides what stays inside it, such as the ball planner's footprint. */
public final class Region {
    public final double minX;
    public final double maxX;
    public final double minY;
    public final double maxY;

    public Region(double minX, double maxX, double minY, double maxY) {
        if (!(minX < maxX && minY < maxY)) {
            throw new IllegalArgumentException(String.format(
                    "empty region x [%.1f, %.1f] y [%.1f, %.1f]: is a margin larger than the space it trims?",
                    minX, maxX, minY, maxY));
        }
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    /** The rectangle with {@code a} and {@code b} as opposite corners, in either order. */
    public static Region spanning(Pose a, Pose b) {
        return new Region(Math.min(a.x(), b.x()), Math.max(a.x(), b.x()),
                Math.min(a.y(), b.y()), Math.max(a.y(), b.y()));
    }

    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    public boolean contains(Pose p) {
        return contains(p.x(), p.y());
    }

    @Override
    public String toString() {
        return String.format("x [%.1f, %.1f] y [%.1f, %.1f]", minX, maxX, minY, maxY);
    }
}
