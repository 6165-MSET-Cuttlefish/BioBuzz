package org.firstinspires.ftc.teamcode.modules.vision;

/** One frame's raw detection; cameraX/cameraY are camera-frame ground inches, not field coordinates. */
public final class BallDetection {

    public final BallType type;
    public final double cameraX;
    public final double cameraY;

    public BallDetection(BallType type, double cameraX, double cameraY) {
        this.type = type;
        this.cameraX = cameraX;
        this.cameraY = cameraY;
    }

    public double distanceTo(double cameraX, double cameraY) {
        return Math.hypot(this.cameraX - cameraX, this.cameraY - cameraY);
    }
}
