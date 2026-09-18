package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.TrackedBall;

/**
 * Exercises modules/Camera through the framework (init, read, field-frame transform, stop) without
 * touching the drivetrain. Needs the webcam and the Pinpoint in the hub config; the follower is built
 * for pose only, nothing drives.
 */
@TeleOp(name = "Camera Module Test", group = "test")
public class CameraModuleTest extends EnhancedOpMode {
    private CameraModuleRobot cam;

    @Override
    protected Robot createRobot() throws InterruptedException {
        Context.allianceColor = AllianceColor.RED;
        cam = new CameraModuleRobot(this);
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
