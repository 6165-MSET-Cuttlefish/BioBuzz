package org.firstinspires.ftc.teamcode.architecture.planner;

import java.util.List;

/** A rectangular obstacle with its sides along the field axes, in Pedro field inches. */
public final class Obstacle {
    public final Region bounds;

    public Obstacle(double minX, double maxX, double minY, double maxY) {
        bounds = new Region(minX, maxX, minY, maxY, "empty obstacle %s");
    }

    public double centreX() {
        return (bounds.minX + bounds.maxX) / 2;
    }

    public double centreY() {
        return (bounds.minY + bounds.maxY) / 2;
    }

    /** Whether a robot centre at the point is within {@code clearance} of this obstacle's edge. */
    public boolean blocks(double x, double y, double clearance) {
        return gapSquared(x, x, y, y) <= clearance * clearance;
    }

    /** Whether any point of the box is within {@code clearance} of this obstacle's edge. */
    public boolean blocks(double minX, double maxX, double minY, double maxY, double clearance) {
        return gapSquared(minX, maxX, minY, maxY) <= clearance * clearance;
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
                || squaredToSegment(bounds.minX, bounds.minY, x1, y1, x2, y2) <= limit
                || squaredToSegment(bounds.minX, bounds.maxY, x1, y1, x2, y2) <= limit
                || squaredToSegment(bounds.maxX, bounds.minY, x1, y1, x2, y2) <= limit
                || squaredToSegment(bounds.maxX, bounds.maxY, x1, y1, x2, y2) <= limit;
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
        return clip(x1, x2 - x1, bounds.minX, bounds.maxX, span) && clip(y1, y2 - y1, bounds.minY, bounds.maxY, span);
    }

    private double gapSquared(double minX, double maxX, double minY, double maxY) {
        double dx = Math.max(Math.max(bounds.minX - maxX, minX - bounds.maxX), 0);
        double dy = Math.max(Math.max(bounds.minY - maxY, minY - bounds.maxY), 0);
        return dx * dx + dy * dy;
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
        return bounds.toString();
    }
}
