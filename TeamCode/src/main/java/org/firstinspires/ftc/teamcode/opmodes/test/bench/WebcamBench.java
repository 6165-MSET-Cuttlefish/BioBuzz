package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.CellTipPipeline;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamControls;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Bench: Webcam", group = "Test")
public class WebcamBench extends LinearOpMode {
    static final String PLUG_IN = "plug a UVC webcam into a Control Hub USB port and rerun run_bench.py --webcam; "
            + "its probe writes the webcam's serial number into the bench config as " + CellTipCamera.WEBCAM_NAME;

    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final long FIRST_FRAME_TIMEOUT_MS = 8000;
    private static final double HOLD_S = 3;
    private static final double SETTLE_S = 0.5;
    private static final double BLOCKING_MS = 20;
    private static final int GAIN_ABOVE_MAX = 50;
    private static final int WHITE_BALANCE_TOO_HIGH_K = 20000;
    private static final int FIND_PER_CONTROL = 150;

    private BenchReport report;
    private WebcamName name;
    private final Map<String, String> shownLines = new HashMap<>();
    private final Telemetry shownCapture = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
            new Class<?>[] {Telemetry.class}, (proxy, method, args) -> {
                if (method.getName().equals("addData") && args != null && args.length == 2) {
                    shownLines.put(String.valueOf(args[0]), String.valueOf(args[1]));
                }
                return null;
            });

    @Override
    public void runOpMode() {
        report = new BenchReport("Webcam", "webcam");
        name = requireWebcam(this);
        report.put("webcamSerial", name.getSerialNumber().getString());
        double sub = BenchIO.param(report.params, "subPhaseSeconds", 5);

        if (!report.holdUntilStart(this)) return;

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
            if (opModeIsActive()) rejectedSettings(new int[] {exposure, gain, wb});
            closeAtDelays();
            report.done("webcam phases complete");
        } finally {
            WebcamControls.manual = manual;
            WebcamControls.exposureMs = exposure;
            WebcamControls.gain = gain;
            WebcamControls.whiteBalanceK = wb;
        }
        report.holdUntilStop(this);
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
            long deadline = System.nanoTime() + 10000 * 1_000_000L;
            while (true) {
                synchronized (closedAt) {
                    if (closedAt[0] != 0) break;
                }
                if (System.nanoTime() > deadline) throw new IllegalStateException("closeCameraDeviceAsync never called onClose");
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
        long deadline = System.nanoTime() + FIRST_FRAME_TIMEOUT_MS * 1_000_000L;
        while (cam.getFrameCount() == 0) {
            if (System.nanoTime() > deadline) {
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

        CellTipPipeline pipe = CellTipCamera.newPipeline(AllianceColor.RED);
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
        long lastSample = 0;
        long lastPreview = 0;
        double sink = 0;
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            long now = System.nanoTime();
            periods.tick(now);
            if (session != null) session.update();
            long w0 = System.nanoTime();
            sink += BenchIO.workload();
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
        CellTipPipeline pipe = CellTipCamera.newPipeline(AllianceColor.RED);
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

    /** The real CellTipCamera with its own WebcamControls and its own Webcam fault, read() by hand each loop. */
    static final class ProbeCamera extends CellTipCamera {
        ProbeCamera(HardwareMap hardwareMap) {
            super(hardwareMap);
        }

        void open() {
            initStates();
            init();
        }

        void readNow() {
            read();
        }

        WebcamSession session() {
            try {
                return (WebcamSession) sessionField().get(this);
            } catch (IllegalAccessException e) {
                throw new IllegalStateException("CellTipCamera.session can't be read", e);
            }
        }
    }

    static Field sessionField() {
        try {
            Field f = CellTipCamera.class.getDeclaredField("session");
            if (f.getType() != WebcamSession.class) {
                throw new IllegalStateException("CellTipCamera.session is a " + f.getType() + ", not a WebcamSession");
            }
            f.setAccessible(true);
            return f;
        } catch (NoSuchFieldException e) {
            throw new IllegalStateException("CellTipCamera has no field session for Bench: Webcam to read", e);
        }
    }

    /** As WebcamControls sends each: exposure in whole ms, gain in device units, white balance in K. */
    private enum Knob {
        EXPOSURE("exposure", "Exposure"),
        GAIN("gain", "Gain"),
        WHITE_BALANCE("white balance", "White balance");

        final String label;
        final String caption;

        Knob(String label, String caption) {
            this.label = label;
            this.caption = caption;
        }

        void ask(int value) {
            switch (this) {
                case EXPOSURE: WebcamControls.exposureMs = value; break;
                case GAIN: WebcamControls.gain = value; break;
                default: WebcamControls.whiteBalanceK = value;
            }
        }

        int min(OpenCvWebcam w) {
            switch (this) {
                case EXPOSURE: return (int) w.getExposureControl().getMinExposure(TimeUnit.MILLISECONDS);
                case GAIN: return w.getGainControl().getMinGain();
                default: return w.getWhiteBalanceControl().getMinWhiteBalanceTemperature();
            }
        }

        int max(OpenCvWebcam w) {
            switch (this) {
                case EXPOSURE: return (int) w.getExposureControl().getMaxExposure(TimeUnit.MILLISECONDS);
                case GAIN: return w.getGainControl().getMaxGain();
                default: return w.getWhiteBalanceControl().getMaxWhiteBalanceTemperature();
            }
        }

        boolean send(OpenCvWebcam w, int value) {
            switch (this) {
                case EXPOSURE: return w.getExposureControl().setExposure(value, TimeUnit.MILLISECONDS);
                case GAIN: return w.getGainControl().setGain(value);
                default: return w.getWhiteBalanceControl().setWhiteBalanceTemperature(value);
            }
        }
    }

    private void rejectedSettings(int[] baseline) {
        report.phase("cellTip:clamp");
        WebcamControls.manual = true;
        for (Knob k : Knob.values()) k.ask(baseline[k.ordinal()]);
        ProbeCamera camera = new ProbeCamera(hardwareMap);
        camera.open();
        try {
            WebcamControls controls = openControls(camera);
            OpenCvWebcam webcam = camera.session().webcam();
            Map<String, Object> opened = hold(camera, controls, null, () -> {}, SETTLE_S);
            clamp(camera, controls, webcam, baseline, opened);
            Map<String, Object> found = rejectFind(webcam);
            reject(camera, controls, webcam, found, baseline);
        } finally {
            try {
                camera.stop();
            } finally {
                Faults.clear(CellTipCamera.FAULT_KEY);
            }
        }
    }

    private WebcamControls openControls(ProbeCamera camera) {
        long t0 = System.nanoTime();
        while (true) {
            camera.readNow();
            WebcamControls c = camera.session().controls();
            if (c != null) return c;
            if ((System.nanoTime() - t0) / 1e6 > FIRST_FRAME_TIMEOUT_MS) {
                throw new IllegalStateException(CellTipCamera.WEBCAM_NAME + " didn't open within " + FIRST_FRAME_TIMEOUT_MS
                        + " ms for the rejected-settings phases");
            }
            report.sendIfDue();
            sleep(5);
        }
    }

    private void clamp(ProbeCamera camera, WebcamControls controls, OpenCvWebcam webcam, int[] baseline,
                       Map<String, Object> opened) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "CellTipCamera's read() every loop while WebcamControls asks for gain above the camera's max, "
                + "exposure 0 ms and white balance " + WHITE_BALANCE_TOO_HIGH_K + " K, " + HOLD_S + " s each, then the "
                + "found value again; WebcamControls clamps into [min, max] before sending when max > min. Per hold: "
                + "blocking writes are [loop, ms] for each read() over " + BLOCKING_MS + " ms; shown, rejected and "
                + "fault are [loop, ms since the change, text] on each change");
        m.put("opened", opened);
        List<Object> holds = new ArrayList<>();
        m.put("holds", holds);
        int gainMax = Knob.GAIN.max(webcam);
        Object[][] asks = {{Knob.GAIN, (int) Math.min(Integer.MAX_VALUE, (long) gainMax + GAIN_ABOVE_MAX)},
                {Knob.EXPOSURE, 0}, {Knob.WHITE_BALANCE, WHITE_BALANCE_TOO_HIGH_K}};
        try {
            for (Object[] ask : asks) {
                if (!opModeIsActive()) break;
                Knob k = (Knob) ask[0];
                int asked = (int) ask[1];
                int lo = k.min(webcam);
                int hi = k.max(webcam);
                Map<String, Object> h = hold(camera, controls, k, () -> k.ask(asked), HOLD_S);
                h.put("asked", asked);
                h.put("min", lo);
                h.put("max", hi);
                h.put("clampExpected", hi > lo && (asked < lo || asked > hi));
                h.put("restore", hold(camera, controls, k, () -> k.ask(baseline[k.ordinal()]), SETTLE_S));
                holds.add(h);
                report.endPhase("clamp", m);
            }
        } finally {
            report.endPhase("clamp", m);
        }
    }

    private Map<String, Object> rejectFind(OpenCvWebcam webcam) {
        report.phase("cellTip:rejectFind");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "each control's own set() straight to the camera, in the units WebcamControls sends, every value "
                + "from its min up, at most " + FIND_PER_CONTROL + " per control with the max last; refused are "
                + "[value, ms set() took]");
        try {
            for (Knob k : Knob.values()) {
                if (!opModeIsActive()) break;
                int lo = k.min(webcam);
                int hi = k.max(webcam);
                List<Integer> values = sweep(lo, hi, FIND_PER_CONTROL);
                List<double[]> refused = new ArrayList<>();
                List<Integer> accepted = new ArrayList<>();
                Samples setMs = new Samples();
                for (int v : values) {
                    if (!opModeIsActive()) break;
                    long t0 = System.nanoTime();
                    boolean ok = k.send(webcam, v);
                    double took = (System.nanoTime() - t0) / 1e6;
                    setMs.add(took);
                    if (ok) accepted.add(v);
                    else refused.add(new double[] {v, took});
                    report.status("%s %d", k.label, v);
                    report.sendIfDue();
                }
                Map<String, Object> one = new LinkedHashMap<>();
                one.put("min", lo);
                one.put("max", hi);
                one.put("tried", setMs.count());
                one.put("inRange", Math.max(0, (long) hi - lo + 1));
                one.put("refused", refused);
                one.put("firstAccepted", accepted.isEmpty() ? null : accepted.get(0));
                one.put("setMs", setMs.summary());
                m.put(k.label, one);
                report.endPhase("rejectFind", m);
            }
        } finally {
            report.endPhase("rejectFind", m);
        }
        return m;
    }

    static List<Integer> sweep(int lo, int hi, int cap) {
        List<Integer> values = new ArrayList<>();
        if (hi < lo) return values;
        long count = (long) hi - lo + 1;
        if (count <= cap) {
            for (long v = lo; v <= hi; v++) values.add((int) v);
            return values;
        }
        for (int i = 0; i < cap - 1; i++) values.add(lo + i);
        values.add(hi);
        return values;
    }

    /** The manual toggle hands the camera back to WebcamControls: it resends all three after rejectFind's writes. */
    @SuppressWarnings("unchecked")
    private void reject(ProbeCamera camera, WebcamControls controls, OpenCvWebcam webcam, Map<String, Object> found,
                        int[] baseline) {
        report.phase("cellTip:reject");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "manual off then on, then for each control the first value rejectFind saw refused, through "
                + "WebcamControls: the found value (or the first accepted) " + SETTLE_S + " s, the refused one "
                + HOLD_S + " s, the valid one again up to " + HOLD_S + " s; records as in clamp");
        List<Object> holds = new ArrayList<>();
        m.put("holds", holds);
        try {
            m.put("manualOff", hold(camera, controls, null, () -> WebcamControls.manual = false, SETTLE_S));
            m.put("manualOn", hold(camera, controls, null, () -> WebcamControls.manual = true, SETTLE_S));
            report.endPhase("reject", m);
            for (Knob k : Knob.values()) {
                Map<String, Object> one = (Map<String, Object>) found.get(k.label);
                List<double[]> refused = one == null ? null : (List<double[]>) one.get("refused");
                if (refused == null || refused.isEmpty() || !opModeIsActive()) continue;
                int value = (int) refused.get(0)[0];
                int valid = baseline[k.ordinal()];
                for (double[] r : refused) {
                    if ((int) r[0] == valid && one.get("firstAccepted") != null) valid = (Integer) one.get("firstAccepted");
                }
                int restore = valid;
                Map<String, Object> h = new LinkedHashMap<>();
                h.put("control", k.label);
                h.put("value", value);
                h.put("valid", restore);
                h.put("min", k.min(webcam));
                h.put("max", k.max(webcam));
                h.put("before", hold(camera, controls, k, () -> k.ask(restore), SETTLE_S));
                h.put("held", hold(camera, controls, k, () -> k.ask(value), HOLD_S));
                h.put("restored", hold(camera, controls, k, () -> k.ask(restore), HOLD_S));
                holds.add(h);
                report.endPhase("reject", m);
            }
        } finally {
            report.endPhase("reject", m);
        }
    }

    private Map<String, Object> hold(ProbeCamera camera, WebcamControls controls, Knob knob, Runnable change,
                                     double seconds) {
        String caption = knob == null ? null : knob.caption;
        String shown = shown(controls, caption);
        String rejected = controls.rejected();
        String fault = webcamFault();
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("control", knob == null ? null : knob.label);
        m.put("shownBefore", shown);
        m.put("rejectedBefore", rejected);
        m.put("faultBefore", fault);
        List<double[]> blocking = new ArrayList<>();
        List<Object[]> shownChanges = new ArrayList<>();
        List<Object[]> rejectedChanges = new ArrayList<>();
        List<Object[]> faultChanges = new ArrayList<>();
        Samples readMs = new Samples();
        change.run();
        long c0 = System.nanoTime();
        int loops = 0;
        while (opModeIsActive() && (System.nanoTime() - c0) / 1e9 < seconds) {
            long r0 = System.nanoTime();
            camera.readNow();
            double ms = (System.nanoTime() - r0) / 1e6;
            readMs.add(ms);
            if (ms > BLOCKING_MS) blocking.add(new double[] {loops, ms});
            double since = (System.nanoTime() - c0) / 1e6;
            String s = shown(controls, caption);
            if (!Objects.equals(s, shown)) shownChanges.add(new Object[] {loops, since, s});
            shown = s;
            String r = controls.rejected();
            if (!Objects.equals(r, rejected)) rejectedChanges.add(new Object[] {loops, since, r});
            rejected = r;
            String f = webcamFault();
            if (!Objects.equals(f, fault)) faultChanges.add(new Object[] {loops, since, f});
            fault = f;
            loops++;
            report.sendIfDue();
            sleep(5);
        }
        m.put("loops", loops);
        m.put("readMs", readMs.summary());
        m.put("blockingWrites", blocking);
        m.put("shown", shownChanges);
        m.put("shownAtEnd", shown);
        m.put("rejected", rejectedChanges);
        m.put("rejectedAtEnd", rejected);
        m.put("fault", faultChanges);
        m.put("faultAtEnd", fault);
        return m;
    }

    private String shown(WebcamControls controls, String caption) {
        if (caption == null) return null;
        shownLines.clear();
        controls.addTelemetry(shownCapture);
        return shownLines.get(caption);
    }

    private static String webcamFault() {
        for (Faults.Fault f : Faults.active()) if (f.key.equals(CellTipCamera.FAULT_KEY)) return f.message;
        return null;
    }
}
