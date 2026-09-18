package org.firstinspires.ftc.teamcode.modules.vision;

import org.opencv.core.Point;

/** One ball found in a single frame, before any frame-to-frame association. */
public final class BallDetection {

    public final double fieldX;
    public final double fieldY;
    public final double imageX;
    public final double imageY;
    public final double imageRadius;

    public BallDetection(double fieldX, double fieldY,
                         double imageX, double imageY, double imageRadius) {
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.imageX = imageX;
        this.imageY = imageY;
        this.imageRadius = imageRadius;
    }

    public Point fieldPoint() { return new Point(fieldX, fieldY); }

    public double distanceTo(double x, double y) {
        return Math.hypot(fieldX - x, fieldY - y);
    }
}
