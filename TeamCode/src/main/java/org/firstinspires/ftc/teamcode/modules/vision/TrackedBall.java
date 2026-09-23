package org.firstinspires.ftc.teamcode.modules.vision;

import org.opencv.core.Point;

/** Camera-relative: a stationary ball reads as moving whenever the robot does. */
public final class TrackedBall {

    public final int id;
    public final BallVisionConstants.BallType type;
    public final double x;
    public final double y;
    public final double vx;
    public final double vy;
    public final double radiusPx;
    public final boolean visible;

    TrackedBall(int id, BallVisionConstants.BallType type, double x, double y, double vx, double vy,
                double radiusPx, boolean visible) {
        this.id = id;
        this.type = type;
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.radiusPx = radiusPx;
        this.visible = visible;
    }

    public double speed() { return Math.hypot(vx, vy); }

    public double headingRad() { return Math.atan2(vy, vx); }

    public double headingDeg() { return Math.toDegrees(headingRad()); }

    public boolean isMoving() { return speed() >= BallTracker.Tuning.movingSpeedIn; }

    /** Constant-velocity extrapolation; balls on carpet decelerate, so keep the horizon short. */
    public Point predict(double seconds) {
        return new Point(x + vx * seconds, y + vy * seconds);
    }

    public Point position() { return new Point(x, y); }

    public double distanceTo(double px, double py) { return Math.hypot(x - px, y - py); }

    @Override
    public String toString() {
        return String.format("#%d %s (%.1f, %.1f)in %.1fin/s @%.0fdeg",
                id, type.label, x, y, speed(), headingDeg());
    }
}
