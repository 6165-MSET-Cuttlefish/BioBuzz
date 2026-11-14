package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightSync;

import java.io.File;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.Semaphore;

@TeleOp(name = "Bench: Limelight Reload", group = "Test")
public class LimelightReloadBench extends LinearOpMode {
    static final String SENDER_THREAD = "LimelightSync";
    static final String GO_FILE = "limelight_reload_go.json";
    private static final String SENDER_KEY = "LimelightSync.sender";
    private static final String REAL = "real";
    private static final String HANG = "hang";
    private static final String ANSWER = "answer";
    static final List<String> MODES = Arrays.asList(REAL, HANG, ANSWER);
    private static final String POST_FRAME = "com.qualcomm.hardware.limelightvision.Limelight3A.sendPostRequest";
    private static final String ACQUIRE_FRAME = "java.util.concurrent.Semaphore.acquire";
    private static final String[] ROW_COLUMNS = {"ms", "syncing", "frameStale", "problem", "senderThreads",
            "sendersInPost", "sendersWaiting", "senderPermits", "fakeOpenPosts", "pipelineIndex"};
    private static final long ROW_NS = 100_000_000L;
    private static final int MAX_ROWS = 900;
    private static final long[] AFTER_STOP_MS = {1000, 3000};
    private static final double GO_WAIT_S = 300;

    private BenchReport report;
    private String mode;
    private long initNs;
    private long sourceNs;
    private long syncStartNs;
    private long syncDoneNs;
    private long firstLiveNs;
    private long stopNs;
    private ReloadFake fake;
    private final List<double[]> rows = new ArrayList<>();
    private final List<double[]> pipelineChanges = new ArrayList<>();
    private final List<Map<String, Object>> problems = new ArrayList<>();

    @Override
    public void runOpMode() {
        initNs = System.nanoTime();
        Map<String, Object> params = BenchIO.params();
        String runId = BenchIO.param(params, "runId", "manual");
        mode = BenchIO.param(params, "mode", REAL);
        if (!MODES.contains(mode)) {
            throw new IllegalArgumentException("Bench: Limelight Reload has no mode " + mode + "; modes are " + MODES);
        }
        report = new BenchReport("Limelight Reload", "limelight_reload_" + runId);
        double holdAfterSyncS = BenchIO.param(report.params, "holdAfterSyncS", 5);
        double hungMs = BenchIO.param(report.params, "hungMs", 300);
        double timeoutS = BenchIO.param(report.params, "timeoutS", 30);
        boolean waitForGo = BenchIO.param(report.params, "waitForGo", 0) != 0;
        report.put("mode", mode);
        report.put("pid", android.os.Process.myPid());
        report.put("code", code());
        Map<String, Object> tuning = new LinkedHashMap<>();
        tuning.put("pipeline", LimelightBallSource.Tuning.pipeline);
        tuning.put("syncOnInit", LimelightBallSource.Tuning.syncOnInit);
        report.put("tuning", tuning);
        Map<String, Object> atInit = new LinkedHashMap<>();
        atInit.put("wallMs", System.currentTimeMillis());
        atInit.put("senderThreads", senderThreads());
        atInit.put("sender", sender());
        report.put("atInit", atInit);
        report.put("rowColumns", ROW_COLUMNS);
        report.put("rows", rows);

        Limelight3A ll = BenchIO.require(hardwareMap, Limelight3A.class, LimelightBallSource.LIMELIGHT_NAME,
                BenchIO.RUNNER_HINT);
        if (mode.equals(REAL)) {
            if (BenchLimelight.baseUrl(ll).contains("127.0.0.1")) {
                throw new IllegalStateException("the hardware map's limelight still points at a fake from an earlier "
                        + "bench (" + BenchLimelight.baseUrl(ll) + "): restart the Robot Controller app");
            }
        } else {
            fake = mode.equals(HANG) ? hangingFake(atInit) : carriedFake(atInit);
            BenchLimelight.repoint(ll, fake.port());
        }
        report.put("baseUrl", BenchLimelight.baseUrl(ll));
        report.save();

        if (waitForGo && !waitForGo()) {
            report.put("stoppedBeforeGo", true);
            endFake();
            report.save();
            return;
        }
        List<Map<String, Object>> atStop;
        LimelightBallSource source = new LimelightBallSource(hardwareMap);
        sourceNs = System.nanoTime();
        try {
            initLoop(source, ll, holdAfterSyncS, hungMs, timeoutS);
            atStop = recordStop(source);
        } finally {
            source.stop();
            endFake();
            report.save();
        }
        watchAfterStop(atStop);
    }

    private Map<String, Object> code() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("classLoader", String.valueOf(getClass().getClassLoader()));
        m.put("classLoaderId", System.identityHashCode(getClass().getClassLoader()));
        m.put("syncClassId", System.identityHashCode(LimelightSync.class));
        m.put("stamp", LimelightSync.STAMP);
        m.put("fileScriptId", LimelightSync.FILE_SCRIPT_ID);
        return m;
    }

    private ReloadFake hangingFake(Map<String, Object> atInit) {
        ReloadFake leftover = ReloadFake.carried(OpMode.blackboard);
        if (leftover != null) {
            leftover.close();
            atInit.put("closedLeftoverFake", true);
        }
        return ReloadFake.start(OpMode.blackboard, new FakeLimelightServer(0, LimelightSync.STAMP));
    }

    private ReloadFake carriedFake(Map<String, Object> atInit) {
        ReloadFake carried = ReloadFake.carried(OpMode.blackboard);
        if (carried == null) {
            throw new IllegalStateException("Bench: Limelight Reload answer needs the fake Limelight a hang run leaves "
                    + "in OpMode.blackboard: run hang right before it, in the same app process");
        }
        carried.check();
        atInit.put("fakeOpenPosts", carried.openPosts());
        atInit.put("fakePosts", posts(carried));
        carried.answer();
        return carried;
    }

    private boolean waitForGo() {
        report.phase("waiting");
        File go = new File(BenchIO.dir(), GO_FILE);
        while (opModeInInit()) {
            if (go.exists()) {
                report.put("goMs", msSince(System.nanoTime()));
                return true;
            }
            if ((System.nanoTime() - initNs) / 1e9 > GO_WAIT_S) {
                throw new IllegalStateException("the runner never wrote " + go + " within " + GO_WAIT_S + " s");
            }
            report.sendIfDue();
            sleep(20);
        }
        return false;
    }

    private void initLoop(LimelightBallSource source, Limelight3A ll, double holdAfterSyncS, double hungMs,
                          double timeoutS) {
        report.phase("init");
        Samples loopMs = new Samples();
        Samples updateMs = new Samples();
        long lastLoopNs = 0;
        long lastRowNs = 0;
        boolean sentOrSaved = false;
        int lastPipeline = Integer.MIN_VALUE;
        String lastProblem = null;
        try {
            while (opModeInInit()) {
                long now = System.nanoTime();
                // A loop that sent a phase packet or rewrote the result file times the bench, not LimelightSync.
                if (lastLoopNs != 0 && !sentOrSaved) loopMs.add((now - lastLoopNs) / 1e6);
                lastLoopNs = now;
                sentOrSaved = false;
                long u0 = System.nanoTime();
                source.update();
                updateMs.addNanos(System.nanoTime() - u0);
                boolean syncing = source.isSyncing();
                String problem = source.problem();
                LLResult latest = ll.getLatestResult();
                if (syncing && syncStartNs == 0) {
                    syncStartNs = now;
                    report.put("syncStartMs", msSince(now));
                    report.phase("sync");
                    sentOrSaved = true;
                }
                if (syncStartNs != 0 && !syncing && syncDoneNs == 0) {
                    syncDoneNs = now;
                    Map<String, Object> sync = new LinkedHashMap<>();
                    sync.put("startMs", msSince(syncStartNs));
                    sync.put("doneMs", msSince(now));
                    sync.put("ms", (now - syncStartNs) / 1e6);
                    sync.put("problem", problem);
                    report.put("sync", sync);
                    report.phase("synced");
                    report.save();
                    sentOrSaved = true;
                }
                if (syncDoneNs != 0) {
                    if (latest.getPipelineIndex() != lastPipeline) {
                        lastPipeline = latest.getPipelineIndex();
                        pipelineChanges.add(new double[] {msSince(now), lastPipeline});
                    }
                    if (!Objects.equals(problem, lastProblem)) {
                        lastProblem = problem;
                        Map<String, Object> p = new LinkedHashMap<>();
                        p.put("ms", msSince(now));
                        p.put("problem", problem);
                        problems.add(p);
                    }
                }
                if (firstLiveNs == 0 && !source.latest().stale) {
                    firstLiveNs = now;
                    Map<String, Object> first = new LinkedHashMap<>();
                    first.put("ms", msSince(now));
                    first.put("sinceSyncDoneMs", syncDoneNs == 0 ? null : (now - syncDoneNs) / 1e6);
                    first.put("pipeline", latest.getPipelineIndex());
                    first.put("stamp", latest.getPythonOutput()[0]);
                    first.put("detections", source.latest().detectionCount);
                    report.put("firstFrame", first);
                    report.save();
                    sentOrSaved = true;
                }
                if (now - lastRowNs >= ROW_NS) {
                    lastRowNs = now;
                    row(now, syncing, source, problem, latest);
                }
                if (!report.isDone()) {
                    sentOrSaved |= maybeDone(now, holdAfterSyncS, hungMs, timeoutS);
                }
                report.sendIfDue();
                sleep(2);
            }
        } finally {
            report.put("initLoopMs", loopMs.summary());
            report.put("updateMs", updateMs.summary());
            report.put("pipelineAfterSync", pipelineChanges);
            report.put("problemsAfterSync", problems);
        }
    }

    private void row(long now, boolean syncing, LimelightBallSource source, String problem, LLResult latest) {
        List<Map<String, Object>> senders = senderThreads();
        int inPost = 0;
        int waiting = 0;
        for (Map<String, Object> t : senders) {
            if ((Boolean) t.get("inPost")) inPost++;
            if ((Boolean) t.get("waitingForSender")) waiting++;
        }
        if (rows.size() < MAX_ROWS) {
            rows.add(new double[] {msSince(now), syncing ? 1 : 0, source.latest().stale ? 1 : 0, problem != null ? 1 : 0,
                    senders.size(), inPost, waiting, senderPermits(), fake == null ? -1 : fake.openPosts(),
                    latest.getPipelineIndex()});
        }
        report.status("%s, %d LimelightSync thread(s), %d waiting for the sender", syncing ? "syncing"
                : syncStartNs == 0 ? "before the sync" : "synced", senders.size(), waiting);
    }

    private boolean maybeDone(long now, double holdAfterSyncS, double hungMs, double timeoutS) {
        double elapsedS = (now - sourceNs) / 1e9;
        if (mode.equals(HANG)) {
            double hung = fake.longestOpenPostMs(now);
            if (hung >= hungMs) {
                report.put("hungAtDoneMs", hung);
                report.phase("hung");
                report.done(String.format("a POST has hung %.0f ms on the fake; STOP now", hung));
                return true;
            }
        } else if (syncDoneNs != 0 && (now - syncDoneNs) / 1e9 >= holdAfterSyncS) {
            Map<String, Object> hold = new LinkedHashMap<>();
            hold.put("afterSyncMs", (now - syncDoneNs) / 1e6);
            hold.put("senderThreads", senderThreads());
            report.put("hold", hold);
            Object problem = ((Map<?, ?>) report.root.get("sync")).get("problem");
            report.done(String.format("sync %s in %.0f ms; %s", problem == null ? "done" : "failed: " + problem,
                    (syncDoneNs - syncStartNs) / 1e6, firstLiveNs == 0 ? "no live frame"
                            : String.format("first live frame %.0f ms after INIT", msSince(firstLiveNs))));
            return true;
        }
        if (elapsedS >= timeoutS) {
            report.put("timedOut", true);
            report.done(String.format("timed out after %.0f s: sync started %s, done %s", timeoutS, syncStartNs != 0,
                    syncDoneNs != 0));
            return true;
        }
        return false;
    }

    private List<Map<String, Object>> recordStop(LimelightBallSource source) {
        long now = System.nanoTime();
        stopNs = now;
        Map<String, Object> stop = new LinkedHashMap<>();
        stop.put("ms", msSince(now));
        stop.put("wallMs", System.currentTimeMillis());
        stop.put("started", !isStopRequested());
        stop.put("syncing", source.isSyncing());
        stop.put("syncStarted", syncStartNs != 0);
        stop.put("msIntoSync", syncStartNs != 0 && syncDoneNs == 0 ? (now - syncStartNs) / 1e6 : null);
        List<Map<String, Object>> atStop = senderThreads();
        stop.put("senderThreads", atStop);
        stop.put("senderPermits", senderPermits());
        report.put("stop", stop);
        return atStop;
    }

    private void endFake() {
        if (fake == null) return;
        report.put("posts", posts(fake));
        report.put("fakeOpenPostsAtEnd", fake.openPosts());
        if (mode.equals(ANSWER)) {
            fake.close();
            report.put("fakeClosed", true);
        }
        fake.check();
    }

    /** Outlives this OpMode by 3 s to see which LimelightSync threads alive at STOP are still alive after it. */
    private void watchAfterStop(List<Map<String, Object>> atStop) {
        long stopNs = this.stopNs;
        Set<Object> ids = new HashSet<>();
        for (Map<String, Object> t : atStop) ids.add(t.get("id"));
        String file = report.fileName + "_afterstop";
        Thread watcher = new Thread(() -> {
            Map<String, Object> out = new LinkedHashMap<>();
            List<Map<String, Object>> samples = new ArrayList<>();
            out.put("atStop", atStop);
            out.put("samples", samples);
            for (long afterMs : AFTER_STOP_MS) {
                long waitMs = (stopNs + afterMs * 1_000_000L - System.nanoTime()) / 1_000_000L;
                try {
                    if (waitMs > 0) Thread.sleep(waitMs);
                } catch (InterruptedException e) {
                    out.put("interrupted", true);
                    Thread.currentThread().interrupt();
                    break;
                }
                List<Map<String, Object>> fromStop = new ArrayList<>();
                List<Map<String, Object>> others = new ArrayList<>();
                for (Map<String, Object> t : senderThreads()) (ids.contains(t.get("id")) ? fromStop : others).add(t);
                Map<String, Object> s = new LinkedHashMap<>();
                s.put("afterStopMs", (System.nanoTime() - stopNs) / 1e6);
                s.put("fromStop", fromStop);
                s.put("others", others);
                samples.add(s);
            }
            out.put("done", true);
            BenchIO.write(file, out);
        }, "bench-limelight-reload-watch");
        watcher.setDaemon(true);
        watcher.start();
    }

    private List<Map<String, Object>> posts(ReloadFake f) {
        List<Map<String, Object>> list = new ArrayList<>();
        for (Map<String, Object> p : f.posts()) {
            long ended = (Long) p.get("endedNs");
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("line", p.get("line"));
            m.put("held", p.get("held"));
            m.put("receivedMs", msSince((Long) p.get("receivedNs")));
            m.put("endedMs", ended == 0 ? null : msSince(ended));
            m.put("endedBy", p.get("endedBy"));
            list.add(m);
        }
        return list;
    }

    static List<Map<String, Object>> senderThreads() {
        List<Map<String, Object>> list = new ArrayList<>();
        for (Thread t : allThreads()) {
            if (!t.getName().equals(SENDER_THREAD)) continue;
            boolean inPost = false;
            boolean waiting = false;
            for (StackTraceElement f : t.getStackTrace()) {
                String frame = f.getClassName() + "." + f.getMethodName();
                inPost |= frame.equals(POST_FRAME);
                waiting |= frame.equals(ACQUIRE_FRAME);
            }
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", t.getId());
            m.put("state", t.getState().name());
            m.put("inPost", inPost);
            m.put("waitingForSender", waiting);
            list.add(m);
        }
        return list;
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
        m.put("onBoardBefore", OpMode.blackboard.get(SENDER_KEY) != null);
        Object used = syncSender();
        Object s = OpMode.blackboard.get(SENDER_KEY);
        m.put("identity", s == null ? null : System.identityHashCode(s));
        m.put("permits", s == null ? null : ((Semaphore) s).availablePermits());
        m.put("syncUsesIt", s != null && s == used);
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

    private double msSince(long nanos) {
        return (nanos - initNs) / 1e6;
    }
}
