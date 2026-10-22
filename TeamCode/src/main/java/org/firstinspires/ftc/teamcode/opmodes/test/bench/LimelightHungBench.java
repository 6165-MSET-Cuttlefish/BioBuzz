package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.modules.vision.LimelightSync;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Semaphore;

@TeleOp(name = "Bench: Limelight Hung", group = "Test")
public class LimelightHungBench extends EnhancedOpMode {
    static final String KEY_LIMELIGHT = "Bench limelight";
    static final String SENDER_THREAD = "LimelightSync";
    private static final String SENDER_KEY = "LimelightSync.sender";

    private static final String HUNG_INIT = "hung_init";
    private static final String STOP_HUNG = "stop_hung";
    private static final String REINIT = "reinit";
    private static final String REINIT_FRESH = "reinit_fresh";
    static final List<String> MODES = Arrays.asList(HUNG_INIT, STOP_HUNG, REINIT, REINIT_FRESH);

    private static final long ROW_INTERVAL_NS = 100_000_000L;
    private static final int MAX_ROWS = 3_000;
    private static final double LIVE_HOLD_S = 1;
    private static final int STACK_FRAMES = 8;

    // Outlives the OpMode, so the next run finds the previous one's POST still hung; a hung_init or stop_hung closes a leftover.
    private static FakeLimelightServer carried;

    public static class HungRobot extends Robot {
        public TimedCamera camera;

        public HungRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {
            camera = new TimedCamera(opMode.hardwareMap, follower, (LimelightHungBench) opMode);
        }
    }

    /** The real module; only its stop() is timed. */
    public static class TimedCamera extends Camera {
        private final LimelightHungBench bench;

        TimedCamera(HardwareMap hardwareMap, Follower follower, LimelightHungBench bench) {
            super(hardwareMap, follower);
            this.bench = bench;
        }

        @Override
        public void stop() {
            long t0 = System.nanoTime();
            try {
                super.stop();
            } finally {
                bench.stopResult.put("cameraStopMs", (System.nanoTime() - t0) / 1e6);
            }
        }
    }

    /** Discovered before the robot's modules (the OpMode's fields are walked first), so its stop() runs first. */
    private final StopHook clock = new StopHook(this::stopPassStarted);
    final Map<String, Object> stopResult = new LinkedHashMap<>();
    private BenchReport report;
    private String mode;
    private FakeLimelightServer fake;
    private Limelight3A ll;
    private double runSeconds;
    private double holdMs;
    private double waitSeconds;

    private long initNs;
    private long runStartNs;
    private long stopPassNs;
    private boolean running;
    private boolean finished;
    private long loops;
    private long initLoops;
    private long runLoops;
    private long lastLoopNs;
    private long lastRowNs;
    private long firstInitLoopNs;
    private boolean sawSyncing;
    private long syncDoneNs;
    private long firstLiveNs;
    private int maxSenderThreads;
    private int maxOpenPosts;

    private List<Faults.Fault> lastFaults = Collections.emptyList();
    private final List<Map<String, Object>> events = new ArrayList<>();
    private final List<double[]> rows = new ArrayList<>();
    private final Samples initHung = new Samples();
    private final Samples initOther = new Samples();
    private final Samples runLoop = new Samples();

    @Override
    protected Robot createRobot() {
        initNs = System.nanoTime();
        Map<String, Object> params = BenchIO.params();
        mode = BenchIO.param(params, "mode", HUNG_INIT);
        if (!MODES.contains(mode)) {
            throw new IllegalArgumentException("Bench: Limelight Hung has no mode " + mode + "; modes are " + MODES);
        }
        report = new BenchReport("Limelight Hung", "limelight_hung_" + BenchIO.param(params, "runId", mode));
        runSeconds = BenchIO.param(report.params, "runSeconds", 3);
        holdMs = BenchIO.param(report.params, "holdMs", 1000);
        waitSeconds = BenchIO.param(report.params, "waitSeconds", 25);
        report.put("mode", mode);

        Map<String, Object> atInit = new LinkedHashMap<>();
        List<Map<String, Object>> senders = senderThreads(true);
        atInit.put("senderThreads", senders);
        atInit.put("senderThreadCount", senders.size());
        atInit.put("sender", sender());
        maxSenderThreads = senders.size();
        if (mode.equals(REINIT) || mode.equals(REINIT_FRESH)) {
            if (carried == null) {
                throw new IllegalStateException("Bench: Limelight Hung " + mode + " needs the fake Limelight a stop_hung run "
                        + "leaves in this app process: run stop_hung right before it");
            }
            fake = carried;
            atInit.put("openPosts", fake.openPosts());
            atInit.put("previousPosts", posts());
            fake.setMode(FakeLimelightServer.Mode.OK);
            if (mode.equals(REINIT_FRESH)) atInit.put("senderWasRunning", clearSenderRunning());
        } else {
            if (carried != null) {
                carried.close();
                carried = null;
                atInit.put("closedLeftoverFake", true);
            }
            fake = new FakeLimelightServer(0, LimelightSync.STAMP);
            fake.setMode(FakeLimelightServer.Mode.HANG_POSTS);
            carried = fake;
        }
        report.put("atInit", atInit);

        ll = BenchIO.require(hardwareMap, Limelight3A.class, LimelightBallSource.LIMELIGHT_NAME, BenchIO.RUNNER_HINT);
        BenchLimelight.repoint(ll, FakeLimelightServer.PORT);
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("devices", BenchIO.deviceNames(hardwareMap));
        info.put("baseUrl", BenchLimelight.baseUrl(ll));
        info.put("postTimeoutMs", BenchLimelight.postTimeoutMs());
        info.put("ballPipeline", LimelightBallSource.Tuning.pipeline);
        info.put("syncOnInit", LimelightBallSource.Tuning.syncOnInit);
        info.put("pollerRunningAtInit", ll.isRunning());
        report.put("info", info);
        return new HungRobot(this);
    }

    private HungRobot bench() {
        return (HungRobot) robot;
    }

    @Override
    protected void initialize() {
        report.put("initializedMs", msSince(System.nanoTime()));
        report.put("events", events);
        report.put("rows", rows);
        report.put("rowColumns", new String[] {"ms", "running", "faultActive", "syncing", "frameStale", "senderThreads",
                "openPosts", "senderPermits"});
        report.phase("init");
        report.save();
    }

    @Override
    protected void onLoopStart() {
        long now = System.nanoTime();
        if (!finished && lastLoopNs != 0) {
            double ms = (now - lastLoopNs) / 1e6;
            if (running) runLoop.add(ms);
            else (fake.openPosts() > 0 ? initHung : initOther).add(ms);
        }
        lastLoopNs = now;
    }

    @Override
    protected void initializeLoop() {
        initLoops++;
        if (firstInitLoopNs == 0) {
            firstInitLoopNs = System.nanoTime();
            report.put("firstInitLoopMs", msSince(firstInitLoopNs));
        }
        tick("init");
        if (finished) return;
        double elapsed = (System.nanoTime() - initNs) / 1e9;
        if (mode.equals(STOP_HUNG)) {
            double hungMs = longestOpenPostMs();
            if (hungMs >= holdMs) {
                finish(String.format("a POST has hung %.0f ms with the sync waiting on it; STOP it now", hungMs));
            } else if (elapsed >= waitSeconds) {
                finish(String.format("no POST hung within %.0f s", waitSeconds));
            }
        } else if (mode.equals(REINIT) || mode.equals(REINIT_FRESH)) {
            long now = System.nanoTime();
            if (syncDoneNs != 0 && Faults.has(LimelightBallSource.FAULT_KEY)) {
                finish("sync ended in a fault: " + bench().camera.limelightProblem());
            } else if (syncDoneNs != 0 && firstLiveNs != 0 && (now - firstLiveNs) / 1e9 >= LIVE_HOLD_S) {
                finish(String.format("sync done %.0f ms after INIT, live frames from %.0f ms", msSince(syncDoneNs),
                        msSince(firstLiveNs)));
            } else if (elapsed >= waitSeconds) {
                finish(String.format("timed out after %.0f s: syncing %s, frame stale %s", waitSeconds,
                        bench().camera.isSyncing(), bench().camera.isFrameStale()));
            }
        }
    }

    @Override
    protected void onStart() {
        runStartNs = System.nanoTime();
        running = true;
        lastLoopNs = 0;
        endInitPhase();
        report.phase("running");
    }

    @Override
    protected void gameLoop() {
        runLoops++;
        tick("running");
        if (!finished && (System.nanoTime() - runStartNs) / 1e9 >= runSeconds) finish(faultSummary());
    }

    private void tick(String phase) {
        loops++;
        fake.check();
        observe(phase);
        Camera camera = bench().camera;
        long now = System.nanoTime();
        boolean syncing = camera.isSyncing();
        if (syncing) sawSyncing = true;
        if (sawSyncing && !syncing && syncDoneNs == 0) {
            syncDoneNs = now;
            Map<String, Object> sync = new LinkedHashMap<>();
            sync.put("doneMs", msSince(now));
            sync.put("problem", camera.limelightProblem());
            report.put("sync", sync);
            report.save();
        }
        boolean stale = camera.isFrameStale();
        if (!stale && firstLiveNs == 0) {
            firstLiveNs = now;
            report.put("firstLiveFrameMs", msSince(now));
            report.save();
        }
        if (now - lastRowNs >= ROW_INTERVAL_NS) {
            lastRowNs = now;
            int senders = senderThreadCount();
            int open = fake.openPosts();
            maxSenderThreads = Math.max(maxSenderThreads, senders);
            maxOpenPosts = Math.max(maxOpenPosts, open);
            if (rows.size() < MAX_ROWS) {
                rows.add(new double[] {msSince(now), running ? 1 : 0, Faults.has(LimelightBallSource.FAULT_KEY) ? 1 : 0,
                        syncing ? 1 : 0, stale ? 1 : 0, senders, open, senderPermits()});
            }
            report.status("%s, %d POST open, %d sender thread(s)", limelightState(), open, senders);
        }
    }

    /** Faults.active() is copy-on-write, so a new list identity means a raise, an update or a clear. */
    private void observe(String phase) {
        List<Faults.Fault> now = Faults.active();
        if (now == lastFaults) return;
        for (Faults.Fault f : now) {
            Faults.Fault old = find(lastFaults, f.key);
            if (old == null) event("raise", f, phase);
            else if (!old.message.equals(f.message)) event("update", f, phase);
        }
        for (Faults.Fault f : lastFaults) {
            if (find(now, f.key) == null) event("clear", f, phase);
        }
        lastFaults = now;
    }

    private void event(String what, Faults.Fault f, String phase) {
        Map<String, Object> e = new LinkedHashMap<>();
        double at = msSince(System.nanoTime());
        e.put("ms", at);
        e.put("phase", phase);
        e.put("what", what);
        e.put("key", f.key);
        e.put("message", f.message);
        e.put("openPosts", fake.openPosts());
        events.add(e);
        BenchIO.log("fault %s %s at %.0f ms (%s): %s", what, f.key, at, phase, f.message);
        if (what.equals("raise") && f.key.equals(LimelightBallSource.FAULT_KEY)
                && !report.root.containsKey("firstFaultMs")) {
            report.put("firstFaultMs", at);
            report.put("firstFaultPhase", phase);
            report.put("firstFaultMessage", f.message);
        }
        if (!what.equals("update")) report.save();
    }

    private static Faults.Fault find(List<Faults.Fault> faults, String key) {
        for (Faults.Fault f : faults) if (f.key.equals(key)) return f;
        return null;
    }

    private String limelightState() {
        Camera camera = bench().camera;
        if (Faults.has(LimelightBallSource.FAULT_KEY)) return "fault";
        if (camera.isSyncing()) return "syncing";
        return camera.isFrameStale() ? "stale" : "live";
    }

    private String faultSummary() {
        Object at = report.root.get("firstFaultMs");
        return at == null ? mode + ": no Limelight fault"
                : String.format("%s: Limelight fault at %.0f ms: %s", mode, (Double) at, report.root.get("firstFaultMessage"));
    }

    private double longestOpenPostMs() {
        long now = System.nanoTime();
        double longest = 0;
        for (FakeLimelightServer.Request r : fake.posts()) {
            if (r.endedNs() == 0) longest = Math.max(longest, (now - r.receivedNs) / 1e6);
        }
        return longest;
    }

    /** Times are ms since this run's INIT, so a POST left by the previous run has a negative receivedMs. */
    private List<Map<String, Object>> posts() {
        List<Map<String, Object>> list = new ArrayList<>();
        long now = System.nanoTime();
        for (FakeLimelightServer.Request r : fake.posts()) {
            long ended = r.endedNs();
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("line", r.line);
            m.put("held", r.held);
            m.put("receivedMs", msSince(r.receivedNs));
            m.put("endedMs", ended == 0 ? null : msSince(ended));
            m.put("endedBy", ended == 0 ? null : r.endedBy());
            m.put("openMs", ((ended == 0 ? now : ended) - r.receivedNs) / 1e6);
            list.add(m);
        }
        return list;
    }

    /** The longest two POSTs were open on the fake at once; a second sender overlaps a hung POST by seconds. */
    private double maxPostOverlapMs() {
        List<FakeLimelightServer.Request> all = fake.posts();
        long now = System.nanoTime();
        double max = 0;
        for (int i = 0; i < all.size(); i++) {
            for (int j = i + 1; j < all.size(); j++) {
                FakeLimelightServer.Request a = all.get(i);
                FakeLimelightServer.Request b = all.get(j);
                long aEnd = a.endedNs() == 0 ? now : a.endedNs();
                long bEnd = b.endedNs() == 0 ? now : b.endedNs();
                long overlap = Math.min(aEnd, bEnd) - Math.max(a.receivedNs, b.receivedNs);
                max = Math.max(max, overlap / 1e6);
            }
        }
        return max;
    }

    private static List<Map<String, Object>> senderThreads(boolean stacks) {
        List<Map<String, Object>> list = new ArrayList<>();
        for (Thread t : allThreads()) {
            if (!t.getName().equals(SENDER_THREAD)) continue;
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", t.getId());
            m.put("state", t.getState().name());
            m.put("daemon", t.isDaemon());
            if (stacks) {
                StackTraceElement[] frames = t.getStackTrace();
                List<String> top = new ArrayList<>();
                for (int i = 0; i < Math.min(STACK_FRAMES, frames.length); i++) top.add(frames[i].toString());
                m.put("top", top);
            }
            list.add(m);
        }
        return list;
    }

    private static int senderThreadCount() {
        int n = 0;
        for (Thread t : allThreads()) if (t.getName().equals(SENDER_THREAD)) n++;
        return n;
    }

    // Not Thread.getAllStackTraces(): it suspends every thread, which would show up as loop time.
    private static Thread[] allThreads() {
        ThreadGroup root = Thread.currentThread().getThreadGroup();
        while (root.getParent() != null) root = root.getParent();
        Thread[] threads = new Thread[root.activeCount() * 2 + 16];
        return Arrays.copyOf(threads, root.enumerate(threads, true));
    }

    private static Map<String, Object> sender() {
        Map<String, Object> m = new LinkedHashMap<>();
        Object s = OpMode.blackboard.get(SENDER_KEY);
        m.put("present", s != null);
        if (s != null) {
            m.put("identity", System.identityHashCode(s));
            m.put("permits", ((Semaphore) s).availablePermits());
        }
        m.put("syncUsesIt", s != null && s == syncSender());
        return m;
    }

    private static Object syncSender() {
        try {
            Field f = LimelightSync.class.getDeclaredField("sender");
            f.setAccessible(true);
            return f.get(null);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("LimelightSync no longer has the static sender this bench compares", e);
        }
    }

    private static int senderPermits() {
        Object s = OpMode.blackboard.get(SENDER_KEY);
        return s == null ? -1 : ((Semaphore) s).availablePermits();
    }

    private static boolean clearSenderRunning() {
        try {
            Field lock = LimelightSync.class.getDeclaredField("lock");
            Field running = LimelightSync.class.getDeclaredField("running");
            lock.setAccessible(true);
            running.setAccessible(true);
            synchronized (lock.get(null)) {
                boolean was = running.getBoolean(null);
                running.setBoolean(null, false);
                return was;
            }
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("LimelightSync no longer has the static lock and running that reinit_fresh "
                    + "clears", e);
        }
    }

    private double msSince(long nanos) {
        return (nanos - initNs) / 1e6;
    }

    private void endInitPhase() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "INIT: loop period between onLoopStart calls, split by whether a POST was open on the fake");
        m.put("seconds", ((runStartNs == 0 ? System.nanoTime() : runStartNs) - initNs) / 1e9);
        m.put("loops", initLoops);
        m.put("loopPeriodPostOpenMs", initHung.summary());
        m.put("loopPeriodOtherMs", initOther.summary());
        report.endPhase("init", m);
    }

    private void finish(String summary) {
        if (running) {
            Map<String, Object> m = new LinkedHashMap<>();
            double seconds = (System.nanoTime() - runStartNs) / 1e9;
            m.put("what", "after START: loop period, Camera polling the fake");
            m.put("seconds", seconds);
            m.put("loops", runLoops);
            m.put("loopsPerSecond", runLoops / seconds);
            m.put("loopPeriodMs", runLoop.summary());
            m.put("loopPeriodHistogram", runLoop.histogram(2, 100));
            report.endPhase("running", m);
        } else {
            endInitPhase();
        }
        recordPosts();
        finished = true;
        report.done(summary);
    }

    private void recordPosts() {
        report.put("posts", posts());
        report.put("maxPostOverlapMs", maxPostOverlapMs());
        report.put("maxOpenPosts", maxOpenPosts);
        report.put("maxSenderThreads", maxSenderThreads);
        report.put("fakeRequests", fake.requests());
    }

    private void stopPassStarted() {
        long now = System.nanoTime();
        stopPassNs = now;
        stopResult.put("atMs", msSince(now));
        stopResult.put("sinceLastLoopMs", lastLoopNs == 0 ? null : (now - lastLoopNs) / 1e6);
        stopResult.put("wasStarted", running);
        stopResult.put("openPosts", fake == null ? null : fake.openPosts());
        stopResult.put("senderThreads", senderThreads(true));
        stopResult.put("syncing", robot != null && bench().camera != null && bench().camera.isSyncing());
        report.put("stop", stopResult);
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(BenchReport.KEY_LOOP, loops);
        telemetry.addData(KEY_LIMELIGHT, limelightState());
    }

    @Override
    protected void onEnd() {
        long now = System.nanoTime();
        stopResult.put("stopPassMs", stopPassNs == 0 ? null : (now - stopPassNs) / 1e6);
        stopResult.put("openPostsAfter", fake.openPosts());
        List<Map<String, Object>> after = senderThreads(false);
        stopResult.put("senderThreadsAfter", after);
        stopResult.put("senderThreadCountAfter", after.size());
        report.put("stop", stopResult);
        report.put("stoppedBeforeDone", !finished);
        if (!finished && !running) endInitPhase();
        recordPosts();
        report.save();
        if (mode.equals(STOP_HUNG)) {
            fake.check();
            return;
        }
        carried = null;
        fake.close();
        fake.check();
    }
}
