package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;
import org.opencv.core.Mat;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

@TeleOp(name = "Bench: Webcam Thread Failure", group = "Test")
public class WebcamFailureBench extends LinearOpMode {
    static final String FAILURE = "BENCH camera-thread failure ";
    private static final int THROW_ON_FRAME = 30;
    private static final long FRAME_TIMEOUT_MS = 15000;

    private static final class ThrowingPipeline extends TimestampedOpenCvPipeline {
        private final String message;
        private volatile int frames;
        private volatile long thrownAtNs;

        ThrowingPipeline(String message) {
            this.message = message;
        }

        @Override
        public Mat processFrame(Mat input, long captureTimeNanos) {
            int frame = frames + 1;
            frames = frame;
            if (frame == THROW_ON_FRAME) {
                thrownAtNs = System.nanoTime();
                BenchIO.log("webcam failure: throwing on the camera thread at frame %d", frame);
                throw new IllegalStateException(message);
            }
            return input;
        }
    }

    @Override
    public void runOpMode() {
        BenchReport report = new BenchReport("Webcam Thread Failure", "webcam_failure");
        String message = FAILURE + BenchIO.param(report.params, "nonce", "manual");
        WebcamBench.requireWebcam(this);
        report.put("message", message);
        report.put("throwOnFrame", THROW_ON_FRAME);
        report.save();
        while (opModeInInit()) {
            report.sendIfDue();
            sleep(20);
        }
        if (!opModeIsActive()) return;

        report.phase("streaming");
        ThrowingPipeline pipeline = new ThrowingPipeline(message);
        long t0 = System.nanoTime();
        WebcamSession session = new WebcamSession(hardwareMap, CellTipCamera.WEBCAM_NAME, pipeline);
        try {
            while (opModeIsActive()) {
                session.update();
                report.status("frame %d", pipeline.frames);
                if (pipeline.frames < THROW_ON_FRAME && (System.nanoTime() - t0) / 1e6 > FRAME_TIMEOUT_MS) {
                    report.done("the webcam delivered " + pipeline.frames + " frames in " + FRAME_TIMEOUT_MS
                            + " ms, short of the " + THROW_ON_FRAME + " the pipeline throws on");
                    break;
                }
                report.sendIfDue();
                sleep(5);
            }
        } finally {
            report.put("framesSeen", pipeline.frames);
            report.put("msFromStartToThrow", pipeline.thrownAtNs == 0 ? null : (pipeline.thrownAtNs - t0) / 1e6);
            report.put("stopRequestedAfterThrow", isStopRequested());
            session.close();
            report.save();
        }
        while (opModeIsActive()) {
            report.sendIfDue();
            sleep(20);
        }
    }
}
