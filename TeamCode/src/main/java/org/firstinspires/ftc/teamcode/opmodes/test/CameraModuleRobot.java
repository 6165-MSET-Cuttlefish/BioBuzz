package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

public class CameraModuleRobot extends Robot {
    public Camera camera;

    public CameraModuleRobot(EnhancedOpMode opMode) throws InterruptedException {
        super(opMode);
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return BettaConstants.create(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {
        camera = new Camera(opMode.hardwareMap).withFollower(follower);
    }
}
