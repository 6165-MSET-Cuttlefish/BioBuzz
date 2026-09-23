package org.firstinspires.ftc.teamcode.modules.vision;

import com.pedropathing.math.Pose;

/** Field-relative; velocity is over the ground, with the robot's own motion removed. */
public final class FieldBall {

    public final int id;
    public final BallVisionConstants.BallType type;
    public final double x;
    public final double y;
    public final double vx;
    public final double vy;
    private final boolean visible;

    FieldBall(int id, BallVisionConstants.BallType type, double x, double y, double vx, double vy,
              boolean visible) {
        this.id = id;
        this.type = type;
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.visible = visible;
    }

    public double speed() { return Math.hypot(vx, vy); }

    public double headingRad() { return Math.atan2(vy, vx); }

    public double headingDeg() { return Math.toDegrees(headingRad()); }

    public boolean isMoving() { return speed() >= BallTracker.Tuning.movingSpeedIn; }

    public boolean visible() { return visible; }

    public Pose pose() { return new Pose(x, y); }

    /** Constant-velocity extrapolation; balls on carpet decelerate, so keep the horizon short. */
    public Pose predict(double seconds) {
        return new Pose(x + vx * seconds, y + vy * seconds);
    }

    public double distanceTo(double fieldX, double fieldY) {
        return Math.hypot(x - fieldX, y - fieldY);
    }

    public double distanceTo(Pose pose) { return distanceTo(pose.x(), pose.y()); }

    @Override
    public String toString() {
        return String.format("#%d %s (%.1f, %.1f)in %.1fin/s @%.0fdeg",
                id, type.label, x, y, speed(), headingDeg());
    }
}
