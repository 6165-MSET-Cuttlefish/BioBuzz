package org.firstinspires.ftc.teamcode.modules.vision;

import org.opencv.core.Point;

/**
 * A ball followed across frames: smoothed field position plus the velocity implied by how that
 * position has moved. Immutable — {@link BallTracker#update} republishes a fresh list each frame.
 */
public final class TrackedBall {

    public final int id;
    public final BallVisionConstants.BallType type;
    public final double x;
    public final double y;
    public final double vx;
    public final double vy;
    public final double radiusPx;
    public final int hits;
    public final int misses;
    public final double ageSeconds;
    public final boolean visible;

    TrackedBall(int id, BallVisionConstants.BallType type, double x, double y, double vx, double vy,
                double radiusPx, int hits, int misses, double ageSeconds, boolean visible) {
        this.id = id;
        this.type = type;
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.radiusPx = radiusPx;
        this.hits = hits;
        this.misses = misses;
        this.ageSeconds = ageSeconds;
        this.visible = visible;
    }

    /** Inches per second. */
    public double speed() { return Math.hypot(vx, vy); }

    /** Direction of travel in field radians, meaningless below {@link #isMoving()}. */
    public double headingRad() { return Math.atan2(vy, vx); }

    public double headingDeg() { return Math.toDegrees(headingRad()); }

    public boolean isMoving() { return speed() >= BallTracker.Tuning.movingSpeedIn; }

    public double predictedX(double seconds) { return x + vx * seconds; }

    public double predictedY(double seconds) { return y + vy * seconds; }

    /** Constant-velocity extrapolation; balls on carpet decelerate, so keep the horizon short. */
    public Point predict(double seconds) {
        return new Point(predictedX(seconds), predictedY(seconds));
    }

    public Point position() { return new Point(x, y); }

    public double distanceTo(double px, double py) { return Math.hypot(x - px, y - py); }

    /** Seconds until this ball is closest to (px, py) if it keeps its current velocity. */
    public double timeOfClosestApproach(double px, double py) {
        double speedSq = vx * vx + vy * vy;
        if (speedSq < 1e-9) return 0;
        return Math.max(0, ((px - x) * vx + (py - y) * vy) / speedSq);
    }

    @Override
    public String toString() {
        return String.format("#%d %s (%.1f, %.1f)in %.1fin/s @%.0fdeg",
                id, type.label, x, y, speed(), headingDeg());
    }
}
