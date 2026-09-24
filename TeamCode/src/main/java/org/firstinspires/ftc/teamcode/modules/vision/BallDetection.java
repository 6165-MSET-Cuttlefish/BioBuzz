package org.firstinspires.ftc.teamcode.modules.vision;

/** One frame's raw detection; cameraX/cameraY are camera-frame ground inches, not field coordinates. */
public final class BallDetection {

    public final BallVisionConstants.BallType type;
    public final double cameraX;
    public final double cameraY;
    public final double imageX;
    public final double imageY;
    public final double imageRadius;

    public BallDetection(BallVisionConstants.BallType type, double cameraX, double cameraY,
                         double imageX, double imageY, double imageRadius) {
        this.type = type;
        this.cameraX = cameraX;
        this.cameraY = cameraY;
        this.imageX = imageX;
        this.imageY = imageY;
        this.imageRadius = imageRadius;
    }

    public double distanceTo(double cameraX, double cameraY) {
        return Math.hypot(this.cameraX - cameraX, this.cameraY - cameraY);
    }
}
