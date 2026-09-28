package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

/** A ball to collect, in Pedro field inches. Mutable so a {@code @Config} field can move it live. */
public class Ball {
    public double x;
    public double y;
    public double radius;

    public Ball(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    public Pose toPose() {
        return new Pose(x, y, 0);
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f)", x, y);
    }
}
