package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Camera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The webcam's CellTipCamera and the Limelight's Camera running as in a match. Mode powerloss: the runner has a person
 * pull the camera hub's power bank mid-run and plug it back in; this records when each camera stopped, the faults, the
 * loop period on both sides of the pull, and whether each camera came back without a re-INIT.
 */
@TeleOp(name = "Bench: Full Load Soak", group = "Test")
public class FullLoadSoakBench extends EnhancedOpMode {
    private static final long ROW_INTERVAL_NS = 100_000_000L;
    private static final long SAVE_INTERVAL_NS = 2_000_000_000L;
    private static final int MAX_ROWS = 6_000;
    private static final double BACK_AFTER_MS = 1000;
    private static final double WINDOW_MS = 10_000;
    // Limelight3A's last update time starts at 0, so before the first poll the time since it is the whole epoch.
    private static final long NEVER_POLLED_MS = 1_000_000_000L;

    public static final class SoakRecorder extends Module {
        private final FullLoadSoakBench bench;

        SoakRecorder(FullLoadSoakBench bench) {
            this.bench = bench;
        }

        @Override protected void initStates() {}
        @Override protected void read() {}
        @Override protected void write() {}

        // Module stop() runs in EnhancedOpMode's safe-state pass after a crash too, where onEnd() doesn't.
        @Override
        public void stop() {
            bench.finish();
        }
    }

    public static class SoakRobot extends Robot {
        public CellTipCamera cellTip;
        public Camera camera;
        public SoakRecorder recorder;

        public SoakRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {
            cellTip = new CellTipCamera(opMode.hardwareMap);
            camera = new Camera(opMode.hardwareMap, follower);
            recorder = new SoakRecorder((FullLoadSoakBench) opMode);
        }
    }

    private SoakRobot bench;
    private BenchReport report;
    private Limelight3A ll;
    private long initNs;
    private long startNs;
    private boolean running;
    private boolean finished;
    private long lastLoopNs;
    private double lastLoopMs = Double.NaN;
    private long lastRowNs;
    private long lastSaveNs;
    private long loopCounter;

    private final SoakWindows loopMs = new SoakWindows();
    private final List<Map<String, Object>> events = new ArrayList<>();
    private final List<double[]> rows = new ArrayList<>();
    private List<Faults.Fault> lastFaults = Collections.emptyList();
    private int lastFrames = -1;
    private int framesAtStart;
    private double lastFrameMs = Double.NaN;
    private double pullRaiseMs = Double.NaN;
    private double pullLastPollMs = Double.NaN;
    private double webcamBackMs = Double.NaN;
    private double limelightBackMs = Double.NaN;
    private Map<String, Object> atPull;

    @Override
    protected Robot createRobot() {
        initNs = System.nanoTime();
        String mode = BenchIO.param(BenchIO.params(), "mode", "powerloss");
        if (!mode.equals("powerloss")) throw new IllegalArgumentException("bench param mode must be powerloss, was " + mode);
        report = new BenchReport("Full Load Soak", "full_load_soak_powerloss");
        ll = BenchIO.require(hardwareMap, Limelight3A.class, LimelightBallSource.LIMELIGHT_NAME, BenchIO.RUNNER_HINT);
        bench = new SoakRobot(this);
        return bench;
    }

    @Override
    protected void initialize() {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("mode", "powerloss");
        info.put("devices", BenchIO.deviceNames(hardwareMap));
        info.put("webcam", CellTipCamera.WEBCAM_NAME);
        info.put("maxStalenessMs", CellTipCamera.maxStalenessMs);
        info.put("limelightNoDataFaultMs", LimelightBallSource.Tuning.noDataFaultMs);
        report.put("info", info);
        report.put("events", events);
        report.put("rows", rows);
        report.put("rowColumns", new String[] {"ms", "running", "limelightFault", "webcamFault", "frames",
                "stalenessMs", "scorable", "tipped", "limelightConnected", "msSinceLastPoll", "loopMs"});
        report.phase("init");
        report.save();
    }

    @Override
    protected void onLoopStart() {
        long now = System.nanoTime();
        if (lastLoopNs != 0) lastLoopMs = (now - lastLoopNs) / 1e6;
        lastLoopNs = now;
        if (running && !finished && !Double.isNaN(lastLoopMs)) loopMs.add(sinceStartMs(now), lastLoopMs);
    }

    @Override
    protected void initializeLoop() {
        observe("init");
    }

    @Override
    protected void onStart() {
        startNs = System.nanoTime();
        running = true;
        lastLoopNs = 0;
        framesAtStart = Math.max(0, frames());
        report.phase("running");
    }

    @Override
    protected void gameLoop() {
        if (finished) return;
        observe("running");
        report.status("%.0f s, Limelight %s, webcam frames %d", sinceStartMs(System.nanoTime()) / 1000,
                Faults.has(LimelightBallSource.FAULT_KEY) ? "FAULT" : "ok", lastFrames - framesAtStart);
    }

    private void observe(String phase) {
        loopCounter++;
        long now = System.nanoTime();
        double ms = running ? sinceStartMs(now) : sinceInitMs(now);
        int frames = frames();
        if (frames > lastFrames) {
            if (running && !Double.isNaN(pullRaiseMs) && ms > pullRaiseMs + BACK_AFTER_MS && Double.isNaN(webcamBackMs)) {
                webcamBackMs = ms;
                event("webcam frame back", CellTipCamera.WEBCAM_NAME, "frame " + frames, phase, ms);
            }
            lastFrames = frames;
            lastFrameMs = ms;
        }
        long sincePoll = msSinceLastPoll();
        List<Faults.Fault> active = Faults.active();
        if (active != lastFaults) {
            for (Faults.Fault f : active) {
                Faults.Fault old = find(lastFaults, f.key);
                if (old == null) raised(f, phase, ms, sincePoll);
                else if (!old.message.equals(f.message)) event("update", f.key, f.message, phase, ms);
            }
            for (Faults.Fault f : lastFaults) {
                if (find(active, f.key) == null) cleared(f, phase, ms);
            }
            lastFaults = active;
        }
        if (now - lastRowNs >= ROW_INTERVAL_NS && rows.size() < MAX_ROWS) {
            lastRowNs = now;
            rows.add(new double[] {ms, running ? 1 : 0, Faults.has(LimelightBallSource.FAULT_KEY) ? 1 : 0,
                    Faults.has(CellTipCamera.FAULT_KEY) ? 1 : 0, frames, staleness(), bench.cellTip.isScorable() ? 1 : 0,
                    bench.cellTip.isTipped() ? 1 : 0, ll.isConnected() ? 1 : 0, sincePoll, lastLoopMs});
        }
        if (now - lastSaveNs >= SAVE_INTERVAL_NS) {
            lastSaveNs = now;
            report.save();
        }
    }

    private void raised(Faults.Fault f, String phase, double ms, long sincePoll) {
        Map<String, Object> e = event("raise", f.key, f.message, phase, ms);
        e.put("msSinceLastPoll", sincePoll);
        if (!running || !f.key.equals(LimelightBallSource.FAULT_KEY) || !Double.isNaN(pullRaiseMs)) return;
        pullRaiseMs = ms;
        pullLastPollMs = sincePoll < 0 ? Double.NaN : ms - sincePoll;
        atPull = new LinkedHashMap<>();
        atPull.put("raiseMs", ms);
        atPull.put("msSinceLastPoll", sincePoll);
        atPull.put("lastPollMs", Double.isNaN(pullLastPollMs) ? null : pullLastPollMs);
        atPull.put("lastWebcamFrameMs", Double.isNaN(lastFrameMs) ? null : lastFrameMs);
        atPull.put("webcamFramesSinceStart", lastFrames - framesAtStart);
        atPull.put("stalenessMs", nullIfNaN(staleness()));
        atPull.put("scorable", bench.cellTip.isScorable());
        atPull.put("tipped", bench.cellTip.isTipped());
        report.put("pull", atPull);
        report.save();
    }

    private void cleared(Faults.Fault f, String phase, double ms) {
        event("clear", f.key, f.message, phase, ms);
        if (running && f.key.equals(LimelightBallSource.FAULT_KEY) && !Double.isNaN(pullRaiseMs)
                && Double.isNaN(limelightBackMs)) {
            limelightBackMs = ms;
        }
    }

    private Map<String, Object> event(String what, String key, String message, String phase, double ms) {
        Map<String, Object> e = new LinkedHashMap<>();
        e.put("ms", ms);
        e.put("phase", phase);
        e.put("what", what);
        e.put("key", key);
        e.put("message", message);
        e.put("webcamFrames", lastFrames);
        e.put("lastWebcamFrameMs", nullIfNaN(lastFrameMs));
        events.add(e);
        BenchIO.log("full load soak %s %s at %.0f ms (%s): %s", what, key, ms, phase, message);
        if (!what.equals("update")) report.save();
        return e;
    }

    void finish() {
        if (finished) return;
        finished = true;
        double end = running ? sinceStartMs(System.nanoTime()) : 0;
        double split = Double.isNaN(pullRaiseMs) ? end : (Double.isNaN(pullLastPollMs) ? pullRaiseMs : pullLastPollMs);
        Map<String, Object> loop = new LinkedHashMap<>();
        loop.put("what", "loop period after START, split at the last Limelight poll before its fault (the pull)");
        loop.put("beforePullMs", loopMs.between(0, split));
        loop.put("afterPullMs", loopMs.between(split, Double.POSITIVE_INFINITY));
        loop.put("windowMs", WINDOW_MS);
        loop.put("windows", loopMs.windows(WINDOW_MS));
        report.put("loop", loop);
        Map<String, Object> cameras = new LinkedHashMap<>();
        cameras.put("ranSeconds", end / 1000);
        cameras.put("webcamFramesSinceStart", lastFrames - framesAtStart);
        cameras.put("lastWebcamFrameMs", nullIfNaN(lastFrameMs));
        cameras.put("webcamBackMs", nullIfNaN(webcamBackMs));
        cameras.put("limelightFaultMs", nullIfNaN(pullRaiseMs));
        cameras.put("limelightBackMs", nullIfNaN(limelightBackMs));
        report.put("cameras", cameras);
        report.done(Double.isNaN(pullRaiseMs) ? "no Limelight fault after START"
                : "Limelight fault " + Math.round(pullRaiseMs) + " ms after START");
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(BenchReport.KEY_LOOP, loopCounter);
        // The runner times the pulls and ends with STOP, so it needs BENCH DONE as soon as it runs.
        if (running && !report.isDone()) telemetry.addData(BenchReport.KEY_DONE, "powerloss: recording until STOP");
    }

    @Override
    protected void onEnd() {
        finish();
    }

    private int frames() {
        WebcamSession session = (WebcamSession) privateField(bench.cellTip, "session");
        return session == null ? -1 : session.webcam().getFrameCount();
    }

    private double staleness() {
        return (double) privateField(bench.cellTip, "stalenessMs");
    }

    private static Object privateField(Object target, String name) {
        try {
            Field f = target.getClass().getDeclaredField(name);
            f.setAccessible(true);
            return f.get(target);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new IllegalStateException("Bench: Full Load Soak reads " + target.getClass().getSimpleName() + "."
                    + name + ", which has changed: update the bench", e);
        }
    }

    private long msSinceLastPoll() {
        long ms = ll.getTimeSinceLastUpdate();
        return ms > NEVER_POLLED_MS ? -1 : ms;
    }

    private double sinceStartMs(long now) {
        return (now - startNs) / 1e6;
    }

    private double sinceInitMs(long now) {
        return (now - initNs) / 1e6;
    }

    private static Faults.Fault find(List<Faults.Fault> faults, String key) {
        for (Faults.Fault f : faults) if (f.key.equals(key)) return f;
        return null;
    }

    private static Double nullIfNaN(double v) {
        return Double.isNaN(v) ? null : v;
    }
}
