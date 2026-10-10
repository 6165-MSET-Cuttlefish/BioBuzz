package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BallCollection;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.LimelightBallSource;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The Limelight fault fallback through the real framework: {@link Camera} (with the real LimelightBallSource) and
 * the Drivetrain on a software-localized follower. Runner param "mode": "absent" (a limelight EthernetDevice with
 * nothing plugged in; runs runSeconds after START, then BENCH DONE) or "replug" (a real Limelight; records until
 * STOP while run_bench.py has the user unplug and replug it, then puts back pipeline restorePipeline). Every fault
 * raise, update and clear, the Limelight3A's own connection signals, loop periods split by fault state, ball
 * counts and, in absent mode, how a BallCollection scheduled at START ended go to limelight_fault_&lt;mode&gt;.json.
 */
@TeleOp(name = "Bench: Limelight Fault", group = "Test")
public class LimelightFaultBench extends EnhancedOpMode {
    static final String KEY_LOOP = "Bench loop";
    private static final long ROW_INTERVAL_NS = 100_000_000L;
    private static final int MAX_ROWS = 6_000;
    // Limelight3A.lastUpdateTime starts at 0, so before the first poll the time since it is the whole epoch.
    private static final long NEVER_POLLED_MS = 1_000_000_000L;

    public static class FaultRobot extends Robot {
        public Drivetrain drivetrain;
        public Camera camera;

        public FaultRobot(EnhancedOpMode opMode) {
            super(opMode);
            FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {
            BenchIO.require(opMode.hardwareMap, AnalogInput.class, "floodgate", BenchIO.RUNNER_HINT);
            drivetrain = new Drivetrain(opMode.hardwareMap).withFollower(follower);
            camera = new Camera(opMode.hardwareMap).withFollower(follower);
        }
    }

    private FaultRobot bench;
    private BallCollection collection;
    private Map<String, Object> collectionResult;
    private BenchReport report;
    private Limelight3A ll;
    private String mode;
    private String faultKey;
    private double runSeconds;
    private int restorePipeline;

    private long initStartNs;
    private long runStartNs;
    private boolean running;
    private boolean finished;
    private long loopCounter;
    private long lastLoopStartNs;
    private long lastRowNs;

    private List<Faults.Fault> lastFaults = Collections.emptyList();
    private final List<Map<String, Object>> events = new ArrayList<>();
    private final List<double[]> rows = new ArrayList<>();
    private final List<double[]> connectionChanges = new ArrayList<>();
    private Boolean wasConnected;
    private double reconnectedAtMs = Double.NaN;
    private double firstFaultMs = Double.NaN;
    private String firstFaultPhase;
    private String firstFaultMessage;

    private final Samples initFaulted = new Samples();
    private final Samples initClean = new Samples();
    private final Samples runFaulted = new Samples();
    private final Samples runClean = new Samples();
    private long initLoops;
    private long runLoops;
    private long runLoopsFaulted;
    private int maxBalls;
    private int maxFieldBalls;
    private int maxVisibleFieldBalls;
    private long liveFrameLoops;

    @Override
    protected Robot createRobot() {
        initStartNs = System.nanoTime();
        mode = BenchIO.param(BenchIO.params(), "mode", "absent");
        if (!mode.equals("absent") && !mode.equals("replug")) {
            throw new IllegalArgumentException("bench param mode must be absent or replug, was " + mode);
        }
        report = new BenchReport("Limelight Fault", "limelight_fault_" + mode);
        faultKey = BenchIO.param(report.params, "faultKey", "Limelight");
        runSeconds = BenchIO.param(report.params, "runSeconds", 15);
        restorePipeline = (int) BenchIO.param(report.params, "restorePipeline", -1);
        ll = BenchIO.require(hardwareMap, Limelight3A.class, LimelightBallSource.LIMELIGHT_NAME, BenchIO.RUNNER_HINT);
        bench = new FaultRobot(this);
        return bench;
    }

    @Override
    protected void initialize() {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("mode", mode);
        info.put("faultKey", faultKey);
        info.put("devices", BenchIO.deviceNames(hardwareMap));
        info.put("ballPipeline", LimelightBallSource.Tuning.pipeline);
        info.put("staleFrameSeconds", LimelightBallSource.Tuning.staleFrameSeconds);
        info.put("pollerRunningAtInit", ll.isRunning());
        info.put("dashboardTelemetry", Robot.telemetryToggles.dashboardTelemetry);
        report.put("info", info);
        report.put("events", events);
        report.put("connectionChanges", connectionChanges);
        report.put("rows", rows);
        report.put("rowColumns", new String[] {"ms", "running", "faultActive", "isConnected", "msSinceLastPoll",
                "pID", "ts", "balls", "visibleFieldBalls", "stale"});
        // Absent mode only: with a real Limelight it could plan and drive.
        if (mode.equals("absent")) {
            collection = new BallCollection(bench.follower, bench.drivetrain, bench.camera, new BallCollection.Settings());
        }
        report.phase("init");
        observe("initialize");
        report.save();
    }

    @Override
    protected void onLoopStart() {
        long now = System.nanoTime();
        if (!finished && lastLoopStartNs != 0) {
            double ms = (now - lastLoopStartNs) / 1e6;
            boolean faulted = Faults.has(faultKey);
            if (running) (faulted ? runFaulted : runClean).add(ms);
            else (faulted ? initFaulted : initClean).add(ms);
        }
        lastLoopStartNs = now;
    }

    @Override
    protected void initializeLoop() {
        initLoops++;
        tick("init");
    }

    @Override
    protected void onStart() {
        runStartNs = System.nanoTime();
        running = true;
        lastLoopStartNs = 0;
        endInitPhase();
        report.phase(mode.equals("absent") ? "running" : "replug");
        if (collection != null) Scheduler.schedule(collection);
    }

    @Override
    protected void gameLoop() {
        if (finished) return;
        runLoops++;
        if (Faults.has(faultKey)) runLoopsFaulted++;
        bench.drivetrain.setMecanumTargets(0.2, 0.1, 0.05, false);
        tick("running");
        recordCollection(false);
        double elapsed = (System.nanoTime() - runStartNs) / 1e9;
        report.status("%.0f s, %s", elapsed, Faults.has(faultKey) ? "FAULT " + faultKey : "no fault");
        if (mode.equals("absent") && elapsed >= runSeconds) finish();
    }

    private void recordCollection(boolean atFinish) {
        if (collection == null || collectionResult != null) return;
        BallCollection.Status status = collection.status();
        boolean ended = (status == BallCollection.Status.DONE || status == BallCollection.Status.ABORTED
                || status == BallCollection.Status.SKIPPED) && !Scheduler.isScheduled(collection);
        if (!ended && !atFinish) return;
        collectionResult = new LinkedHashMap<>();
        collectionResult.put("status", status.toString());
        collectionResult.put("detail", collection.detail());
        collectionResult.put("ended", ended);
        collectionResult.put("msSinceStart", (System.nanoTime() - runStartNs) / 1e6);
        collectionResult.put("drivetrainWritesEnabled", bench.drivetrain.isWriteEnabled());
        report.put("collection", collectionResult);
        BenchIO.log("ball collection %s: %s", status, collection.detail());
        report.save();
    }

    private void tick(String phase) {
        loopCounter++;
        observe(phase);
        int balls = bench.camera.getBalls().size();
        List<FieldBall> fieldBalls = bench.camera.getFieldBalls();
        int visible = 0;
        for (FieldBall b : fieldBalls) if (b.visible()) visible++;
        boolean stale = bench.camera.isFrameStale();
        maxBalls = Math.max(maxBalls, balls);
        maxFieldBalls = Math.max(maxFieldBalls, fieldBalls.size());
        maxVisibleFieldBalls = Math.max(maxVisibleFieldBalls, visible);
        if (!stale) liveFrameLoops++;

        long now = System.nanoTime();
        if (now - lastRowNs >= ROW_INTERVAL_NS && rows.size() < MAX_ROWS) {
            lastRowNs = now;
            LLResult r = ll.getLatestResult();
            rows.add(new double[] {msSinceInit(), running ? 1 : 0, Faults.has(faultKey) ? 1 : 0, ll.isConnected() ? 1 : 0,
                    msSinceLastPoll(), r.getPipelineIndex(), r.getTimestamp(), balls, visible, stale ? 1 : 0});
        }
    }

    /** Faults.active() is copy-on-write, so a new list identity means a raise, an update or a clear. */
    private void observe(String phase) {
        boolean connected = ll.isConnected();
        long sincePoll = msSinceLastPoll();
        if (wasConnected == null || connected != wasConnected) {
            double at = msSinceInit();
            connectionChanges.add(new double[] {at, connected ? 1 : 0, sincePoll});
            if (connected && Boolean.FALSE.equals(wasConnected)) reconnectedAtMs = at - Math.max(0, sincePoll);
            wasConnected = connected;
        }

        List<Faults.Fault> now = Faults.active();
        if (now == lastFaults) return;
        for (Faults.Fault f : now) {
            Faults.Fault old = find(lastFaults, f.key);
            if (old == null) event("raise", f.key, f.message, phase, connected, sincePoll);
            else if (!old.message.equals(f.message)) event("update", f.key, f.message, phase, connected, sincePoll);
        }
        for (Faults.Fault f : lastFaults) {
            if (find(now, f.key) == null) event("clear", f.key, f.message, phase, connected, sincePoll);
        }
        lastFaults = now;
    }

    private void event(String what, String key, String message, String phase, boolean connected, long sincePoll) {
        Map<String, Object> e = new LinkedHashMap<>();
        double at = msSinceInit();
        e.put("ms", at);
        if (running) e.put("msSinceStart", (System.nanoTime() - runStartNs) / 1e6);
        e.put("phase", phase);
        e.put("what", what);
        e.put("key", key);
        e.put("message", message);
        e.put("isConnected", connected);
        e.put("msSinceLastPoll", sincePoll);
        if (what.equals("clear") && !Double.isNaN(reconnectedAtMs)) e.put("msSinceReconnect", at - reconnectedAtMs);
        events.add(e);
        BenchIO.log("fault %s %s at %.0f ms (%s): %s", what, key, at, phase, message);
        if (what.equals("raise") && key.equals(faultKey) && Double.isNaN(firstFaultMs)) {
            firstFaultMs = at;
            firstFaultPhase = phase;
            firstFaultMessage = message;
            report.put("firstFaultMs", firstFaultMs);
            report.put("firstFaultPhase", firstFaultPhase);
            report.put("firstFaultMessage", firstFaultMessage);
        }
        // Not on an update: a message with a running time changes every 100 ms, and a save is a flash write.
        if (!what.equals("update")) report.save();
    }

    private static Faults.Fault find(List<Faults.Fault> faults, String key) {
        for (Faults.Fault f : faults) if (f.key.equals(key)) return f;
        return null;
    }

    private long msSinceLastPoll() {
        long ms = ll.getTimeSinceLastUpdate();
        return ms > NEVER_POLLED_MS ? -1 : ms;
    }

    private double msSinceInit() {
        return (System.nanoTime() - initStartNs) / 1e6;
    }

    private void endInitPhase() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "INIT: loop period between onLoopStart calls, split by whether " + faultKey + " was raised");
        m.put("seconds", (runStartNs - initStartNs) / 1e9);
        m.put("loops", initLoops);
        m.put("loopPeriodFaultedMs", initFaulted.summary());
        m.put("loopPeriodCleanMs", initClean.summary());
        report.endPhase("init", m);
    }

    private void finish() {
        Map<String, Object> m = new LinkedHashMap<>();
        double seconds = (System.nanoTime() - runStartNs) / 1e9;
        m.put("what", "after START: loop period split by fault state, Drivetrain writing mecanum targets every loop");
        m.put("seconds", seconds);
        m.put("loops", runLoops);
        m.put("loopsFaulted", runLoopsFaulted);
        m.put("loopsPerSecond", runLoops / seconds);
        m.put("loopPeriodFaultedMs", runFaulted.summary());
        m.put("loopPeriodFaultedHistogram", runFaulted.histogram(2, 100));
        m.put("loopPeriodCleanMs", runClean.summary());
        report.endPhase(mode.equals("absent") ? "running" : "replug", m);

        Map<String, Object> vision = new LinkedHashMap<>();
        vision.put("maxBalls", maxBalls);
        vision.put("maxFieldBalls", maxFieldBalls);
        vision.put("maxVisibleFieldBalls", maxVisibleFieldBalls);
        vision.put("liveFrameLoops", liveFrameLoops);
        vision.put("loops", initLoops + runLoops);
        report.put("vision", vision);
        recordCollection(true);
        finished = true;
        report.done(String.format("%s: %s", mode,
                Double.isNaN(firstFaultMs) ? "no " + faultKey + " fault" : faultKey + " fault at " + Math.round(firstFaultMs) + " ms"));
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(KEY_LOOP, loopCounter);
        // The runner drives replug by hand and ends it with STOP, so it needs BENCH DONE as soon as it runs.
        if (running && mode.equals("replug") && !finished) {
            telemetry.addData(BenchReport.KEY_DONE, "replug: recording until STOP");
        }
    }

    @Override
    protected void onEnd() {
        if (!running) {
            report.save();
            return;
        }
        if (!finished) finish();
        if (restorePipeline >= 0) {
            // Camera.stop() has just stopped the poller, so a recent poll means the Limelight still answers.
            long sincePoll = msSinceLastPoll();
            boolean answering = sincePoll >= 0 && sincePoll < 1000;
            report.put("restorePipeline", restorePipeline);
            report.put("restoreAccepted", answering ? ll.pipelineSwitch(restorePipeline) : "skipped: no poll for " + sincePoll + " ms");
            report.save();
        }
    }
}
