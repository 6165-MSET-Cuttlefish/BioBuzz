package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.TrackedBall;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

/**
 * Exercises modules/Camera through the framework (init, read, field-frame transform, stop). Nothing
 * drives, but the framework builds the follower, so the hub config needs the drive motors and the
 * Pinpoint as well as the webcam.
 */
@TeleOp(name = "Camera Module Test", group = "Test")
public class CameraModuleTest extends EnhancedOpMode {

    static class CameraRobot extends Robot {
        Camera camera;

        CameraRobot(EnhancedOpMode opMode) throws InterruptedException {
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

    private CameraRobot cam;

    @Override
    protected Robot createRobot() throws InterruptedException {
        cam = new CameraRobot(this);
        return cam;
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Balls tracked", cam.camera.getBallCount());
        TrackedBall nearest = cam.camera.getNearestBall();
        if (nearest != null) {
            telemetry.addData("Nearest (camera frame)", nearest.toString());
        }
        int i = 0;
        for (FieldBall b : cam.camera.getFieldBalls()) {
            if (i++ >= 3) break;
            telemetry.addData("Field ball " + i, b.toString());
        }
    }
}
