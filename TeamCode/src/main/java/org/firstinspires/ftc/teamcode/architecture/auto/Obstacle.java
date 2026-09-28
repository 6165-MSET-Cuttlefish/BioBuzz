package org.firstinspires.ftc.teamcode.architecture.auto;

import java.util.List;

/** A circular obstacle, in Pedro field inches. Mutable so a {@code @Config} field can move it live. */
public class Obstacle {
    public double x;
    public double y;
    public double radius;

    public Obstacle(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    /**
     * Whether the segment passes within {@code clearance} of this obstacle's edge. With clearance at half the
     * robot's width, this checks the robot's body while tracing only its centreline.
     */
    public boolean intersectsSegment(double x1, double y1, double x2, double y2, double clearance) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double lenSq = dx * dx + dy * dy;
        double t = lenSq == 0 ? 0 : ((x - x1) * dx + (y - y1) * dy) / lenSq;
        t = Math.max(0, Math.min(1, t));

        double closestX = x1 + t * dx - x;
        double closestY = y1 + t * dy - y;
        double minDist = radius + clearance;
        return closestX * closestX + closestY * closestY <= minDist * minDist;
    }

    public static boolean anyBlocks(List<Obstacle> obstacles,
                                    double x1, double y1, double x2, double y2, double clearance) {
        for (Obstacle o : obstacles) {
            if (o.intersectsSegment(x1, y1, x2, y2, clearance)) return true;
        }
        return false;
    }
}
