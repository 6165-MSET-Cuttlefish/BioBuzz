package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCVPipelines.PollenDetectionPipeline.PollenDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamControls;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamSession;

/**
 * Runs the Sample detection pipeline on the robot — the on-robot equivalent of selecting the
 * pipeline in EOCV-Sim. {@code WebcamControls.*} is live on FtcDashboard while it runs.
 */
@TeleOp(name = "Sample Detection", group = "test")
@Config
public class SampleDetection extends LinearOpMode {

    // Must match the webcam name in res/xml/camera.xml (the Arducam UC-852 / OV9782).
    private static final String WEBCAM_NAME = "nerdDetector";

    private WebcamSession session;

    @Override
    public void runOpMode() {
        session = new WebcamSession(hardwareMap, telemetry, WEBCAM_NAME,
                new PollenDetectionPipeline(telemetry));

        // Pipeline drives telemetry from the camera thread; here we only pump camera controls.
        while (opModeInInit()) pump();
        while (opModeIsActive()) pump();

        session.close();
    }

    private void pump() {
        session.update();
        sleep(20);
    }
}
