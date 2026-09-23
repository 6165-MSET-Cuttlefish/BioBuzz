package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.vision.WebcamControls;
import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Streams the raw webcam feed to FtcDashboard with the live {@link WebcamControls} exposure / gain /
 * white-balance sliders applied. Must run on the robot — EOCV-Sim never opens an OpenCvWebcam, so
 * the controls only take effect here.
 */
@TeleOp(name = "Camera Tune", group = "Test")
public class CameraTune extends LinearOpMode {

    // Must match the webcam name in res/xml/camera.xml (the Arducam UC-852 / OV9782).
    private static final String WEBCAM_NAME = "nerdDetector";

    private WebcamSession session;

    @Override
    public void runOpMode() {
        session = new WebcamSession(hardwareMap, telemetry, WEBCAM_NAME, new PassThrough());

        while (opModeInInit()) pump();   // stream is live in init, so tuning works before play
        while (opModeIsActive()) pump();

        session.close();
    }

    private void pump() {
        session.update();
        telemetry.addLine("Edit WebcamControls.* in the FtcDashboard config.");
        telemetry.addData("Mode", WebcamControls.manual ? "MANUAL" : "AUTO");
        telemetry.addData("Exposure (ms)", WebcamControls.exposureMs);
        telemetry.addData("Gain", WebcamControls.gain);
        telemetry.addData("White balance (K)", WebcamControls.whiteBalanceK);
        telemetry.addData("Camera FPS", "%.1f", session.webcam().getFps());
        telemetry.update();
        sleep(20);
    }

    private static class PassThrough extends OpenCvPipeline {
        @Override public Mat processFrame(Mat input) {
            return input;
        }
    }
}
