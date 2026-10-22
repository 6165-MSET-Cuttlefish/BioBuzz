package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.modules.vision.WebcamControls;
import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Camera Tune", group = "Test")
public class CameraTune extends LinearOpMode {

    private WebcamSession session;

    @Override
    public void runOpMode() {
        session = new WebcamSession(hardwareMap, CellTipCamera.WEBCAM_NAME, new PassThrough());

        try {
            while (opModeInInit()) pump();
            while (opModeIsActive()) pump();
        } finally {
            session.close();
        }
    }

    private void pump() {
        session.update();
        telemetry.addLine("Edit Webcam Controls in the dashboard config.");
        WebcamControls controls = session.controls();
        if (controls == null) telemetry.addLine("Waiting for the camera to open");
        else controls.addTelemetry(telemetry);
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
