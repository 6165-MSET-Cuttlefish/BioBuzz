package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Camera;

public class CameraRobot extends BallCollectionRobot {
    public Camera camera;

    public CameraRobot(EnhancedOpMode opMode) {
        super(opMode);
    }

    @Override
    protected void initializeGameModules() {
        camera = new Camera(opMode.hardwareMap, follower);
    }
}
