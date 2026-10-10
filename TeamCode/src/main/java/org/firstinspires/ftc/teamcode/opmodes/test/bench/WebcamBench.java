package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.modules.vision.CellTipPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.WebcamControls;
import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Webcam costs with any UVC webcam configured as aprilTagDetector: EasyOpenCV open, first-frame and close times
 * (sync and async), CellTipPipeline's per-frame cost on the camera thread with detection on, off, with and without
 * the dashboard stream, WebcamControls write stalls, and the real WebcamSession.close() + CellTipPipeline.release()
 * path at several delays after the session is created, including while its async open is still running.
 */
@TeleOp(name = "Bench: Webcam", group = "Test")
public class WebcamBench extends LinearOpMode {
    static final String PLUG_IN = "plug a UVC webcam into a Control Hub USB port and rerun run_bench.py --webcam; "
            + "its probe writes the webcam's serial number into the bench config as " + CellTipCamera.WEBCAM_NAME;
    static final String[] LABELS = {"SCORING", "AUDIENCE"};
    static final int[][] RED_CLUSTERS = {{30, 31, 32, 33}, {34, 35, 36, 37}};

    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final long FIRST_FRAME_TIMEOUT_MS = 8000;

    private BenchReport report;
    private WebcamName name;

    @Override
    public void runOpMode() {
        report = new BenchReport("Webcam", "webcam");
        name = requireWebcam(this);
        report.put("webcamSerial", name.getSerialNumber().getString());
        double sub = BenchIO.param(report.params, "subPhaseSeconds", 5);

        while (opModeInInit()) {
            report.sendIfDue();
            sleep(20);
        }
        if (!opModeIsActive()) return;

        boolean manual = WebcamControls.manual;
        int exposure = WebcamControls.exposureMs;
        int gain = WebcamControls.gain;
        int wb = WebcamControls.whiteBalanceK;
        try {
            report.phase("rawOpenClose");
            List<Object> raw = new ArrayList<>();
            for (int i = 0; i < 3; i++) raw.add(rawCycle(false));
            report.endPhase("rawOpenClose", wrap("createWebcam, openCameraDevice, startStreaming 640x480 MJPEG, "
                    + "first frame, 2 s of frames, closeCameraDevice; no RC-screen monitor", raw));

            report.phase("asyncClose");
            List<Object> async = new ArrayList<>();
            for (int i = 0; i < 2; i++) async.add(rawCycle(true));
            report.endPhase("asyncClose", wrap("the same, closed with closeCameraDeviceAsync: how long the call blocks "
                    + "and when onClose fires", async));

            cellTipProcessing(sub);
            closeAtDelays();
            report.done("webcam phases complete");
        } finally {
            WebcamControls.manual = manual;
            WebcamControls.exposureMs = exposure;
            WebcamControls.gain = gain;
            WebcamControls.whiteBalanceK = wb;
        }
        while (opModeIsActive()) {
            report.sendIfDue();
            sleep(20);
        }
    }

    static WebcamName requireWebcam(LinearOpMode op) {
        WebcamName w = BenchIO.require(op.hardwareMap, WebcamName.class, CellTipCamera.WEBCAM_NAME, PLUG_IN);
        if (!w.isAttached()) {
            throw new IllegalStateException("The configured webcam " + w.getSerialNumber() + " isn't attached: " + PLUG_IN);
        }
        return w;
    }

    private static Map<String, Object> wrap(String what, Object runs) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", what);
        m.put("runs", runs);
        return m;
    }

    private static final class PassThrough extends OpenCvPipeline {
        @Override public Mat processFrame(Mat input) {
            return input;
        }
    }

    private Map<String, Object> rawCycle(boolean async) {
        OpenCvWebcam cam = OpenCvCameraFactory.getInstance().createWebcam(name);
        cam.setPipeline(new PassThrough());
        Map<String, Object> r = new LinkedHashMap<>();
        long t0 = System.nanoTime();
        int code = cam.openCameraDevice();
        long t1 = System.nanoTime();
        r.put("openMs", (t1 - t0) / 1e6);
        if (code != 0) throw new IllegalStateException("openCameraDevice failed with EasyOpenCV error " + code + ": " + PLUG_IN);
        try {
            cam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
        } catch (OpenCvCameraException e) {
            throw new IllegalStateException("This webcam doesn't stream 640x480 MJPEG, which WebcamSession needs", e);
        }
        long t2 = System.nanoTime();
        r.put("startStreamingMs", (t2 - t1) / 1e6);
        waitForFrame(cam);
        long t3 = System.nanoTime();
        r.put("firstFrameAfterStartMs", (t3 - t2) / 1e6);
        sleep(2000);
        r.put("fps", cam.getFps());
        r.put("overheadMs", cam.getOverheadTimeMs());
        long t4 = System.nanoTime();
        if (async) {
            final long[] closedAt = {0};
            cam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                @Override public void onClose() {
                    synchronized (closedAt) {
                        closedAt[0] = System.nanoTime();
                    }
                }
            });
            long t5 = System.nanoTime();
            r.put("closeAsyncCallMs", (t5 - t4) / 1e6);
            long deadline = System.currentTimeMillis() + 10000;
            while (true) {
                synchronized (closedAt) {
                    if (closedAt[0] != 0) break;
                }
                if (System.currentTimeMillis() > deadline) throw new IllegalStateException("closeCameraDeviceAsync never called onClose");
                sleep(5);
            }
            r.put("closeAsyncDoneMs", (closedAt[0] - t4) / 1e6);
        } else {
            cam.closeCameraDevice();
            r.put("closeMs", (System.nanoTime() - t4) / 1e6);
        }
        sleep(500);
        return r;
    }

    private void waitForFrame(OpenCvWebcam cam) {
        long deadline = System.currentTimeMillis() + FIRST_FRAME_TIMEOUT_MS;
        while (cam.getFrameCount() == 0) {
            if (System.currentTimeMillis() > deadline) {
                throw new IllegalStateException("no frame from the webcam within " + FIRST_FRAME_TIMEOUT_MS + " ms of startStreaming");
            }
            sleep(2);
        }
    }

    private void cellTipProcessing(double seconds) {
        report.phase("cellTipProcessing");
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("what", "CellTipPipeline inside the real WebcamSession: processFrame ms on the camera thread, camera fps, "
                + "and this thread's fixed-workload loop, 'seconds' each");
        r.put("seconds", seconds);
        r.put("maxStalenessMs", CellTipCamera.maxStalenessMs);
        r.put("noCamera", loopWhile("noCamera", null, null, seconds, false));

        CellTipPipeline pipe = new CellTipPipeline("RED", LABELS, RED_CLUSTERS);
        TimingPipeline timed = new TimingPipeline(pipe);
        WebcamSession session = new WebcamSession(hardwareMap, CellTipCamera.WEBCAM_NAME, timed);
        try {
            long t0 = System.nanoTime();
            while (timed.frames() == 0) {
                session.update();
                if ((System.nanoTime() - t0) / 1e6 > FIRST_FRAME_TIMEOUT_MS) {
                    throw new IllegalStateException("WebcamSession delivered no frame within " + FIRST_FRAME_TIMEOUT_MS + " ms");
                }
                sleep(5);
            }
            r.put("sessionFirstFrameMs", (System.nanoTime() - t0) / 1e6);

            r.put("detectionOnStreamOn", loopWhile("detectionOnStreamOn", session, timed, seconds, false));
            pipe.setEnabled(false);
            r.put("detectionOffStreamOn", loopWhile("detectionOffStreamOn", session, timed, seconds, false));
            pipe.setEnabled(true);
            session.setCameraStreamEnabled(false);
            r.put("detectionOnStreamOff", loopWhile("detectionOnStreamOff", session, timed, seconds, false));
            session.setCameraStreamEnabled(true);
            r.put("detectionOnStreamOnDsPreviewRequests", loopWhile("dsPreviewRequests", session, timed, seconds, true));
            r.put("webcamControlWrites", controlWrites(session, seconds));
        } finally {
            long c0 = System.nanoTime();
            try {
                session.close();
            } finally {
                long c1 = System.nanoTime();
                pipe.release();
                r.put("closeMs", (c1 - c0) / 1e6);
                r.put("releaseMs", (System.nanoTime() - c1) / 1e6);
                report.endPhase("cellTipProcessing", r);
            }
        }
    }

    /** A loop like an OpMode's around a fixed workload; the run_bench.py side counts dashboard images per phase. */
    private Map<String, Object> loopWhile(String label, WebcamSession session, TimingPipeline timed, double seconds,
                                          boolean dsPreview) {
        report.phase("cellTip:" + label);
        Samples.Synced frameMs = Samples.synced();
        Samples.Synced captureAgeMs = Samples.synced();
        if (timed != null) timed.recordInto(frameMs, captureAgeMs);
        Samples periods = new Samples();
        Samples work = new Samples();
        Samples fps = new Samples();
        Samples overhead = new Samples();
        BenchIO.Cpu cpu = BenchIO.Cpu.now();
        long frames0 = timed == null ? 0 : timed.frames();
        long start = System.nanoTime();
        long last = 0;
        long lastSample = 0;
        long lastPreview = 0;
        double sink = 0;
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            long now = System.nanoTime();
            if (last != 0) periods.addNanos(now - last);
            last = now;
            if (session != null) session.update();
            long w0 = System.nanoTime();
            for (int i = 0; i < 20000; i++) sink += Math.sin(i * 0.001) * Math.cos(i * 0.002);
            work.addNanos(System.nanoTime() - w0);
            if (session != null && now - lastSample > 100_000_000L) {
                fps.add(session.webcam().getFps());
                overhead.add(session.webcam().getOverheadTimeMs());
                lastSample = now;
            }
            if (dsPreview && now - lastPreview > 100_000_000L) {
                CameraStreamServer.getInstance().handleRequestFrame();
                lastPreview = now;
            }
            report.sendIfDue();
            sleep(1);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("loopPeriodMs", periods.summary());
        m.put("workloadMs", work.summary());
        if (timed != null) {
            m.put("framesProcessed", timed.frames() - frames0);
            m.put("processFrameMs", frameMs.summary());
            m.put("captureAgeMs", captureAgeMs.summary());
            m.put("cameraFps", fps.summary());
            m.put("easyOpenCvOverheadMs", overhead.summary());
        }
        m.put("cpu", BenchIO.Cpu.now().since(cpu));
        m.put("checksum", sink);
        return m;
    }

    private Map<String, Object> controlWrites(WebcamSession session, double seconds) {
        report.phase("cellTip:controls");
        WebcamControls.manual = true;
        Samples calm = new Samples();
        Samples changed = new Samples();
        long start = System.nanoTime();
        long lastChange = 0;
        int changes = 0;
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            long now = System.nanoTime();
            boolean change = now - lastChange > 500_000_000L;
            if (change) {
                changes++;
                WebcamControls.exposureMs = changes % 2 == 0 ? 8 : 12;
                WebcamControls.gain = changes % 2 == 0 ? 0 : 10;
                WebcamControls.whiteBalanceK = changes % 2 == 0 ? 3250 : 4000;
                lastChange = now;
            }
            long t0 = System.nanoTime();
            session.update();
            (change ? changed : calm).addNanos(System.nanoTime() - t0);
            report.sendIfDue();
            sleep(5);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "session.update() when exposure, gain and white balance all change (every 500 ms) vs when none do");
        m.put("updateWithChangesMs", changed.summary());
        m.put("updateWithoutChangesMs", calm.summary());
        return m;
    }

    private void closeAtDelays() {
        report.phase("closeAtDelay");
        long[] delays = {0, 50, 200, 500, 1000, -1, 3000};
        List<Object> runs = new ArrayList<>();
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("what", "new CellTipPipeline + new WebcamSession, wait, then CellTipCamera.stop()'s close path: session.close() "
                + "then pipeline.release(); delay -1 waits for the first frame; 3000 is closing while streaming");
        r.put("runs", runs);
        try {
            for (long delay : delays) {
                report.status("delay %d ms", delay);
                runs.add(closeAfter(delay));
                report.endPhase("closeAtDelay", r);
                sleep(1500);
            }
        } finally {
            report.endPhase("closeAtDelay", r);
        }
    }

    private Map<String, Object> closeAfter(long delayMs) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("delayMs", delayMs);
        CellTipPipeline pipe = new CellTipPipeline("RED", LABELS, RED_CLUSTERS);
        long created = System.nanoTime();
        WebcamSession session = new WebcamSession(hardwareMap, CellTipCamera.WEBCAM_NAME, pipe);
        m.put("constructMs", (System.nanoTime() - created) / 1e6);
        try {
            if (delayMs < 0) {
                while (session.webcam().getFrameCount() == 0) {
                    session.update();
                    if ((System.nanoTime() - created) / 1e6 > FIRST_FRAME_TIMEOUT_MS) {
                        throw new IllegalStateException("no first frame within " + FIRST_FRAME_TIMEOUT_MS + " ms");
                    }
                    sleep(2);
                }
            } else {
                while ((System.nanoTime() - created) / 1e6 < delayMs) {
                    session.update();
                    sleep(Math.min(10, delayMs));
                }
            }
        } finally {
            m.put("framesAtClose", session.webcam().getFrameCount());
            m.put("closeStartedAfterCreateMs", (System.nanoTime() - created) / 1e6);
            long t0 = System.nanoTime();
            try {
                session.close();
            } finally {
                long t1 = System.nanoTime();
                pipe.release();
                m.put("closeMs", (t1 - t0) / 1e6);
                m.put("releaseMs", (System.nanoTime() - t1) / 1e6);
            }
        }
        return m;
    }
}
