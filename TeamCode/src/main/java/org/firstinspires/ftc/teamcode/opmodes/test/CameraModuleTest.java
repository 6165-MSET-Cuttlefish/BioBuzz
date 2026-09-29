package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.TrackedBall;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

/**
 * Starts at (0, 0, 0), so while the robot hasn't moved a field ball reads robot-relative (+X forward,
 * +Y left of the pose reference); the "robot" column stays robot-relative after it moves.
 */
@TeleOp(name = "Camera Module Test", group = "Test")
public class CameraModuleTest extends EnhancedOpMode {

    private static final int SHOWN_BALLS = 5;

    static class CameraRobot extends Robot {
        Camera camera;

        CameraRobot(EnhancedOpMode opMode) {
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
    protected Robot createRobot() {
        cam = new CameraRobot(this);
        return cam;
    }

    @Override
    protected void initialize() {
        cam.follower.setPose(new Pose(0, 0, 0));
    }

    @Override
    protected void telemetry() {
        Camera camera = cam.camera;
        telemetry.addData("Frame", camera.limelightProblem() != null ? "STALE (Limelight fault)"
                : camera.isFrameStale() ? "STALE" : "live");
        telemetry.addData("Balls tracked", camera.getBallCount());
        TrackedBall nearest = camera.getNearestBall();
        if (nearest != null) {
            telemetry.addData("Nearest (camera frame)", nearest.toString());
        }
        Pose robot = cam.follower.pose();
        double cos = Math.cos(robot.heading());
        double sin = Math.sin(robot.heading());
        int i = 0;
        for (FieldBall b : camera.getFieldBalls()) {
            if (i++ >= SHOWN_BALLS) break;
            double dx = b.x - robot.x();
            double dy = b.y - robot.y();
            telemetry.addData("Ball " + b.id, "%s field (%.1f, %.1f) robot (%.1f, %.1f)%s",
                    b.type.label, b.x, b.y, dx * cos + dy * sin, -dx * sin + dy * cos,
                    b.visible() ? "" : " [last seen]");
        }
    }
}
