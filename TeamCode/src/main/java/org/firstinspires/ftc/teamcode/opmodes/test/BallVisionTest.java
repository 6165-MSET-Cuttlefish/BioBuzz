package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetectionPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.TrackedBall;

/**
 * Webcam-only bench check for {@link BallDetectionPipeline}: needs just the webcam in the hub config,
 * unlike Camera Module Test, which pulls in the drivetrain and Pinpoint.
 */
@TeleOp(name = "Ball Vision", group = "Test")
public class BallVisionTest extends LinearOpMode {

    private static final String WEBCAM_NAME = "nerdDetector";

    private WebcamSession session;
    private BallDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        pipeline = new BallDetectionPipeline();
        session = new WebcamSession(hardwareMap, telemetry, WEBCAM_NAME, pipeline);

        while (opModeInInit()) pump();
        while (opModeIsActive()) pump();

        session.close();
    }

    private void pump() {
        session.update();

        BallDetectionPipeline.Frame frame = pipeline.latest();
        telemetry.addData("Calibrated", pipeline.isCalibrated());
        telemetry.addData("FPS", "%.1f", frame.fps);
        telemetry.addData("Balls", frame.balls.size());
        telemetry.addData("ROIs searched", frame.roiCount);
        telemetry.addData("Rejected (color)", frame.rejectedColor);
        telemetry.addData("Rejected (overlap)", frame.rejectedOverlap);

        int shown = 0;
        for (TrackedBall ball : frame.balls) {
            if (shown++ >= 5) break;
            telemetry.addData("Ball " + ball.id, ball.toString());
        }
        telemetry.update();

        sleep(20);
    }
}
