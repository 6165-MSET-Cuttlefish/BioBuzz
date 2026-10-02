package org.firstinspires.ftc.teamcode.architecture.auto;

import java.util.List;

/** A rectangular obstacle with its sides along the field axes, in Pedro field inches. */
public final class Obstacle {
    public final double minX;
    public final double maxX;
    public final double minY;
    public final double maxY;

    public Obstacle(double minX, double maxX, double minY, double maxY) {
        if (!(minX < maxX && minY < maxY)) {
            throw new IllegalArgumentException(String.format(
                    "empty obstacle x [%.1f, %.1f] y [%.1f, %.1f]", minX, maxX, minY, maxY));
        }
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    /** From the point to the nearest edge; 0 inside. */
    public double distanceTo(double x, double y) {
        double dx = Math.max(Math.max(minX - x, x - maxX), 0);
        double dy = Math.max(Math.max(minY - y, y - maxY), 0);
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** From this obstacle to the nearest point of {@code region}; 0 when they overlap. */
    public double distanceTo(Region region) {
        double dx = Math.max(Math.max(minX - region.maxX, region.minX - maxX), 0);
        double dy = Math.max(Math.max(minY - region.maxY, region.minY - maxY), 0);
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Whether a robot centre at the point is within {@code clearance} of this obstacle's edge. */
    public boolean blocks(double x, double y, double clearance) {
        double dx = Math.max(Math.max(minX - x, x - maxX), 0);
        double dy = Math.max(Math.max(minY - y, y - maxY), 0);
        return dx * dx + dy * dy <= clearance * clearance;
    }

    /**
     * Whether the segment passes within {@code clearance} of this obstacle's edge. With clearance at half the
     * robot's width, this checks the robot's body while tracing only its centreline.
     */
    public boolean intersectsSegment(double x1, double y1, double x2, double y2, double clearance) {
        if (crosses(x1, y1, x2, y2)) return true;
        // Apart, a segment and a rectangle are nearest at an end of the segment or a corner of the rectangle.
        double limit = clearance * clearance;
        return blocks(x1, y1, clearance) || blocks(x2, y2, clearance)
                || squaredToSegment(minX, minY, x1, y1, x2, y2) <= limit
                || squaredToSegment(minX, maxY, x1, y1, x2, y2) <= limit
                || squaredToSegment(maxX, minY, x1, y1, x2, y2) <= limit
                || squaredToSegment(maxX, maxY, x1, y1, x2, y2) <= limit;
    }

    public static boolean anyBlocks(List<Obstacle> obstacles,
                                    double x1, double y1, double x2, double y2, double clearance) {
        for (Obstacle o : obstacles) {
            if (o.intersectsSegment(x1, y1, x2, y2, clearance)) return true;
        }
        return false;
    }

    private boolean crosses(double x1, double y1, double x2, double y2) {
        double[] span = {0, 1};
        return clip(x1, x2 - x1, minX, maxX, span) && clip(y1, y2 - y1, minY, maxY, span);
    }

    /** Narrows {@code span} to the part of the segment's t inside {@code min..max} on one axis; false when none is. */
    private static boolean clip(double from, double delta, double min, double max, double[] span) {
        if (delta == 0) return from >= min && from <= max;
        double a = (min - from) / delta;
        double b = (max - from) / delta;
        span[0] = Math.max(span[0], Math.min(a, b));
        span[1] = Math.min(span[1], Math.max(a, b));
        return span[0] <= span[1];
    }

    private static double squaredToSegment(double px, double py, double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double lenSq = dx * dx + dy * dy;
        double t = lenSq == 0 ? 0 : ((px - x1) * dx + (py - y1) * dy) / lenSq;
        t = Math.max(0, Math.min(1, t));
        double ex = x1 + t * dx - px;
        double ey = y1 + t * dy - py;
        return ex * ex + ey * ey;
    }

    @Override
    public String toString() {
        return String.format("x [%.1f, %.1f] y [%.1f, %.1f]", minX, maxX, minY, maxY);
    }
}
