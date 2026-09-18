package org.firstinspires.ftc.teamcode.modules.vision;

import com.pedropathing.math.Pose;

/**
 * A tracked ball expressed in field coordinates, with the robot's own motion removed — so
 * {@link #speed()} is how fast the ball is moving over the ground, not how fast it is sliding
 * across the camera's view.
 *
 * @see TrackedBall the same ball in camera-relative coordinates
 */
public final class FieldBall {

    public final int id;
    public final BallVisionConstants.BallType type;
    public final double x;
    public final double y;
    public final double vx;
    public final double vy;
    /** The camera-relative track this was derived from. */
    public final TrackedBall source;

    FieldBall(int id, BallVisionConstants.BallType type, double x, double y, double vx, double vy,
              TrackedBall source) {
        this.id = id;
        this.type = type;
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.source = source;
    }

    /** Inches per second over the ground. */
    public double speed() { return Math.hypot(vx, vy); }

    /** Field radians the ball is travelling along, meaningless below {@link #isMoving()}. */
    public double headingRad() { return Math.atan2(vy, vx); }

    public double headingDeg() { return Math.toDegrees(headingRad()); }

    public boolean isMoving() { return speed() >= BallTracker.Tuning.movingSpeedIn; }

    public boolean visible() { return source.visible; }

    public Pose pose() { return new Pose(x, y); }

    /** Constant-velocity extrapolation; balls on carpet decelerate, so keep the horizon short. */
    public Pose predict(double seconds) {
        return new Pose(x + vx * seconds, y + vy * seconds);
    }

    public double distanceTo(double fieldX, double fieldY) {
        return Math.hypot(x - fieldX, y - fieldY);
    }

    public double distanceTo(Pose pose) { return distanceTo(pose.x(), pose.y()); }

    /** Seconds until this ball is closest to (fieldX, fieldY) if it keeps its current velocity. */
    public double timeOfClosestApproach(double fieldX, double fieldY) {
        double speedSq = vx * vx + vy * vy;
        if (speedSq < 1e-9) return 0;
        return Math.max(0, ((fieldX - x) * vx + (fieldY - y) * vy) / speedSq);
    }

    @Override
    public String toString() {
        return String.format("#%d %s (%.1f, %.1f)in %.1fin/s @%.0fdeg",
                id, type.label, x, y, speed(), headingDeg());
    }
}
