package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamSession;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetectionPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.TrackedBall;

/**
 * Runs the on-robot {@link BallDetectionPipeline} directly against the webcam and streams it to
 * FtcDashboard's camera view via {@link WebcamSession} — the same {@code startCameraStream} call
 * {@code Camera} uses, but without the rest of the framework. Needs only {@code nerdDetector} in the
 * hub config, not the drivetrain/Pinpoint {@code Camera Module Test} pulls in by going through
 * {@code EnhancedOpMode}, so this is the one to run for a webcam-only bench check or to tune Pollen
 * and Nectar detection live.
 *
 * <p>{@code WebcamControls.*} and {@code BallDetectionPipeline}'s {@code @Config} classes
 * ({@code BallVision}, {@code BallVision_Pollen}, {@code BallVision_RedNectar},
 * {@code BallVision_BlueNectar}) are all live on FtcDashboard while it runs; switch
 * {@code BallVision.displayMode} to {@code MASK} there to see each type's colour mask.
 */
@TeleOp(name = "Ball Vision", group = "test")
public class BallVisionTest extends LinearOpMode {

    // Must match the webcam name in res/xml/camera.xml (the Arducam UC-852 / OV9782).
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
