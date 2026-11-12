package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.biobuzz.modules.TransferCounter;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;

@TeleOp(name = "Bench: Thread Lifecycle", group = "Test")
public class ThreadLifecycleBench extends OpMode {
    static final String COUNTER_THREAD = "TransferCounter";
    static final String CRASH_IN_LOOP = "BENCH thread lifecycle: deliberate crash in loop()";
    static final String CRASH_IN_STOP = "BENCH thread lifecycle: deliberate crash in stop() before TransferCounter.stop()";
    static final String THREAD_DIED = "TransferCounter thread died";

    private static final String NORMAL = "normal";
    private static final String CRASH_LOOP = "crash_in_loop";
    private static final String CRASH_STOP = "crash_in_stop";
    private static final String STOP_BEFORE_START = "stop_before_start";
    private static final String THREAD_FAILURE = "thread_failure";
    private static final String INTERRUPT_PROBE = "interrupt_probe";
    static final List<String> MODES = Arrays.asList(NORMAL, CRASH_LOOP, CRASH_STOP, STOP_BEFORE_START, THREAD_FAILURE,
            INTERRUPT_PROBE);

    private static final String BEFORE_TRANSMIT = "beforeTransmit";
    private static final String AWAITING_RESPONSE = "awaitingResponse";
    private static final String IN_LOCK_RELEASE = "inLockRelease";
    private static final String LOCK_CLASS = "com.qualcomm.hardware.lynx.MessageKeyedLock";
    private static final String NOT_INTERRUPTED = "exitedWithoutInterrupt";

    private static final String UPSTREAM = "analog0";
    private static final String DOWNSTREAM = "analog1";
    private static final double MAX_RANGE_MM = 1300;
    private static final double POLL_MS = 10;
    private static final int INVALID_PORT = 99;
    private static final long SAMPLE_INTERVAL_NS = 1_000_000_000L;
    private static final long CRASH_WATCH_NS = 5_000_000_000L;
    private static final double INIT_DONE_S = 1;
    private static final double DEATH_TIMEOUT_S = 3;
    private static final double BASELINE_S = 3;
    private static final double MAX_SPIN_MS = 3;
    private static final double STALL_MS = 50;
    private static final double LOCK_STALL_MS = 400;
    private static final int MAX_STALL_ROWS = 300;
    private static final long HELPER_JOIN_MS = 1000;

    private BenchReport report;
    private String mode;
    private double seconds;
    private double crashAfterS;
    private double injectAfterS;
    private double probeSeconds;
    private LynxModule hub;
    private TransferCounter counter;
    private Thread counterThread;
    private InterruptProbe probe;
    private final List<Map<String, Object>> samples = new ArrayList<>();
    private long initNs;
    private long startNs;
    private long lastSampleNs;
    private long injectNs;
    private long loopsAfterInjection;

    @Override
    public void init() {
        mode = BenchIO.param(BenchIO.params(), "mode", NORMAL);
        if (!MODES.contains(mode)) {
            throw new IllegalArgumentException("Bench: Thread Lifecycle has no mode " + mode + "; modes are " + MODES);
        }
        report = new BenchReport("Thread Lifecycle", "thread_lifecycle_" + mode);
        seconds = BenchIO.param(report.params, "seconds", 5);
        crashAfterS = BenchIO.param(report.params, "crashAfterS", 2);
        injectAfterS = BenchIO.param(report.params, "injectAfterS", 1);
        probeSeconds = BenchIO.param(report.params, "probeSeconds", 15);
        report.put("mode", mode);
        List<Map<String, Object>> leftover = counterThreads();
        report.put("leftoverAtInit", leftover);
        report.put("leftoverCountAtInit", leftover.size());
        hub = HubRegisters.controlHub(hardwareMap, "Bench: Thread Lifecycle");
        BenchIO.require(hardwareMap, AnalogInput.class, UPSTREAM, BenchIO.RUNNER_HINT);
        BenchIO.require(hardwareMap, AnalogInput.class, DOWNSTREAM, BenchIO.RUNNER_HINT);
        if (!mode.equals(INTERRUPT_PROBE)) {
            counter = new TransferCounter(hardwareMap, UPSTREAM, DOWNSTREAM, MAX_RANGE_MM, POLL_MS);
        }
        report.put("samples", samples);
        initNs = System.nanoTime();
        report.save();
    }

    @Override
    public void init_loop() {
        if (mode.equals(STOP_BEFORE_START) && !report.isDone() && (System.nanoTime() - initNs) / 1e9 >= INIT_DONE_S) {
            report.done("in INIT with TransferCounter built and never started; STOP it without START");
        }
        report.sendIfDue();
    }

    @Override
    public void start() {
        startNs = System.nanoTime();
        lastSampleNs = startNs;
        report.phase(mode);
        if (mode.equals(INTERRUPT_PROBE)) {
            probe = new InterruptProbe(Thread.currentThread());
            return;
        }
        counter.start();
        counterThread = threadOf(counter);
        report.put("counterThreadDaemon", counterThread.isDaemon());
        report.save();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double elapsed = (now - startNs) / 1e9;
        switch (mode) {
            case NORMAL:
                runCounter(now);
                if (!report.isDone() && elapsed >= seconds) {
                    report.done(String.format(Locale.US, "TransferCounter ran %.0f s; STOP it to time stop()", elapsed));
                }
                break;
            case CRASH_LOOP:
                runCounter(now);
                if (elapsed >= crashAfterS) {
                    report.put("thrownAtMs", elapsed * 1000);
                    watchAfterCrash();
                    throw new IllegalStateException(CRASH_IN_LOOP);
                }
                break;
            case CRASH_STOP:
                runCounter(now);
                if (!report.isDone() && elapsed >= crashAfterS) {
                    report.done("running; STOP it: stop() throws before TransferCounter.stop()");
                }
                break;
            case THREAD_FAILURE:
                threadFailure(now, elapsed);
                break;
            case INTERRUPT_PROBE:
                if (!report.isDone() && probe.step()) {
                    probe.halt();
                    report.put("probe", probe.toMap());
                    report.done(probe.line());
                }
                break;
            default:
                break;
        }
        report.sendIfDue();
    }

    @Override
    public void stop() {
        switch (mode) {
            case CRASH_STOP:
                report.put("stopEnteredMs", startNs == 0 ? null : (System.nanoTime() - startNs) / 1e6);
                watchAfterCrash();
                throw new IllegalStateException(CRASH_IN_STOP);
            case INTERRUPT_PROBE:
                if (probe != null) {
                    probe.halt();
                    report.put("probe", probe.toMap());
                    report.save();
                }
                return;
            default:
                stopCounter();
        }
    }

    private void runCounter(long now) {
        counter.count();
        if (now - lastSampleNs < SAMPLE_INTERVAL_NS) return;
        lastSampleNs = now;
        TransferCounter.Stats s = counter.stats();
        Map<String, Object> m = statsMap(s);
        m.put("ms", (now - startNs) / 1e6);
        m.put("threads", counterThreads());
        samples.add(m);
        report.status("%d TransferCounter polls", s == null ? 0 : s.polls);
        report.save();
    }

    private void stopCounter() {
        Map<String, Object> m = new LinkedHashMap<>();
        report.put("stop", m);
        m.put("started", counterThread != null);
        if (counterThread != null) m.put("finalStats", statsMap(counter.stats()));
        boolean returned = false;
        long t0 = System.nanoTime();
        try {
            counter.stop();
            returned = true;
        } finally {
            m.put("returned", returned);
            m.put("stopMs", (System.nanoTime() - t0) / 1e6);
            m.put("counterThreadAliveAfter", counterThread != null && counterThread.isAlive());
            List<Map<String, Object>> after = counterThreads();
            m.put("threadsAfter", after);
            m.put("threadCountAfter", after.size());
            report.save();
        }
    }

    private void threadFailure(long now, double elapsed) {
        if (injectNs == 0) {
            counter.count();
            if (!report.isDone() && elapsed >= injectAfterS) inject();
            return;
        }
        try {
            counter.stats();
        } catch (IllegalStateException e) {
            if (!THREAD_DIED.equals(e.getMessage())) throw e;
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("seenMsAfterInjection", (System.nanoTime() - injectNs) / 1e6);
            m.put("loopsAfterInjection", loopsAfterInjection);
            m.put("cause", String.valueOf(e.getCause()));
            m.put("counterThreadAlive", counterThread.isAlive());
            report.put("death", m);
            report.save();
            throw e;
        }
        loopsAfterInjection++;
        if (!report.isDone() && (now - injectNs) / 1e9 >= DEATH_TIMEOUT_S) {
            report.done("set TransferCounter.portA to " + INVALID_PORT + " but its thread was still running "
                    + DEATH_TIMEOUT_S + " s later");
        }
    }

    // portA is read after each bulk read returns, so the thread throws with the hub's lock already released.
    private void inject() {
        Map<String, Object> m = new LinkedHashMap<>();
        report.put("injection", m);
        try {
            Field port = TransferCounter.class.getDeclaredField("portA");
            port.setAccessible(true);
            m.put("portBefore", port.getInt(counter));
            port.setInt(counter, INVALID_PORT);
            m.put("portAfter", port.getInt(counter));
        } catch (ReflectiveOperationException e) {
            m.put("refused", e.toString());
            report.done("can't inject a TransferCounter thread failure: " + e);
            return;
        }
        injectNs = System.nanoTime();
        m.put("atMs", (injectNs - startNs) / 1e6);
    }

    private void watchAfterCrash() {
        Thread opModeThread = Thread.currentThread();
        Thread watched = counterThread;
        Map<String, Object> after = new LinkedHashMap<>();
        after.put("done", false);
        report.put("afterCrash", after);
        report.save();
        long thrownNs = System.nanoTime();
        Thread watcher = new Thread(() -> {
            try {
                long opModeEndNs = 0;
                long counterEndNs = 0;
                while (System.nanoTime() - thrownNs < CRASH_WATCH_NS && (opModeEndNs == 0 || counterEndNs == 0)) {
                    long now = System.nanoTime();
                    if (opModeEndNs == 0 && !opModeThread.isAlive()) opModeEndNs = now;
                    if (counterEndNs == 0 && (watched == null || !watched.isAlive())) counterEndNs = now;
                    Thread.sleep(2);
                }
                after.put("opModeThreadEndedMs", opModeEndNs == 0 ? null : (opModeEndNs - thrownNs) / 1e6);
                after.put("counterThreadEndedMs", counterEndNs == 0 ? null : (counterEndNs - thrownNs) / 1e6);
                after.put("watchedMs", CRASH_WATCH_NS / 1e6);
                after.put("threads", counterThreads());
                after.put("done", true);
                report.save();
            } catch (Throwable t) {
                RobotLog.ee(BenchIO.TAG, t, "thread lifecycle crash watch failed");
            }
        }, "BenchCrashWatch");
        watcher.setDaemon(true);
        watcher.start();
    }

    private List<Map<String, Object>> counterThreads() {
        List<Map<String, Object>> list = new ArrayList<>();
        for (Thread t : allThreads()) {
            if (!t.getName().equals(COUNTER_THREAD)) continue;
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", t.getId());
            m.put("state", t.getState().name());
            m.put("daemon", t.isDaemon());
            m.put("ours", t == counterThread);
            list.add(m);
        }
        return list;
    }

    // Not Thread.getAllStackTraces(): it suspends every thread, which would show up in the counter as late polls.
    private static Thread[] allThreads() {
        ThreadGroup root = Thread.currentThread().getThreadGroup();
        while (root.getParent() != null) root = root.getParent();
        Thread[] threads = new Thread[root.activeCount() * 2 + 16];
        return Arrays.copyOf(threads, root.enumerate(threads, true));
    }

    private static Thread threadOf(TransferCounter counter) {
        try {
            Field f = TransferCounter.class.getDeclaredField("thread");
            f.setAccessible(true);
            return (Thread) f.get(counter);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("can't read TransferCounter.thread", e);
        }
    }

    private static Map<String, Object> statsMap(TransferCounter.Stats s) {
        Map<String, Object> m = new LinkedHashMap<>();
        if (s == null) return m;
        m.put("polls", s.polls);
        m.put("seconds", s.seconds);
        m.put("pollHz", s.seconds > 0 ? s.polls / s.seconds : null);
        m.put("latePolls", s.latePolls);
        m.put("maxGapMs", s.maxGapMs);
        m.put("meanCommandMs", s.meanCommandMs);
        m.put("newValues", s.newValues);
        return m;
    }

    private long ownBulkRead() {
        long t0 = System.nanoTime();
        try {
            new LynxGetBulkInputDataCommand(hub).sendReceive();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("the OpMode thread was interrupted during its own bulk read", e);
        } catch (LynxNackException e) {
            throw new IllegalStateException("the hub refused the OpMode thread's own bulk read", e);
        }
        return System.nanoTime() - t0;
    }

    private final class InterruptProbe {
        private final Thread opModeThread;
        private final Random random = new Random();
        private final Samples baselineMs = new Samples();
        private final Samples interruptedMs = new Samples();
        private final Samples landedCallMs = new Samples();
        private final List<double[]> stalls = new ArrayList<>();
        private final Map<String, Long> landed = new LinkedHashMap<>();
        private final long beganNs = System.nanoTime();
        private long interruptingNs;
        private Helper helper;
        private long interrupts;
        private long helperStarts;
        private long helperCommands;
        private long readsOverStall;
        private long readsOverLockStall;
        private double maxReadMs;
        private boolean joined;

        InterruptProbe(Thread opModeThread) {
            this.opModeThread = opModeThread;
            for (String k : new String[] {BEFORE_TRANSMIT, AWAITING_RESPONSE, IN_LOCK_RELEASE, NOT_INTERRUPTED}) {
                landed.put(k, 0L);
            }
            helper = startHelper();
        }

        private Helper startHelper() {
            Helper h = new Helper(opModeThread);
            helperStarts++;
            h.start();
            return h;
        }

        boolean step() {
            long t0 = System.nanoTime();
            long readNs = ownBulkRead();
            double readMs = readNs / 1e6;
            boolean interrupting = interruptingNs != 0;
            (interrupting ? interruptedMs : baselineMs).add(readMs);
            maxReadMs = Math.max(maxReadMs, readMs);
            if (readMs >= STALL_MS) {
                readsOverStall++;
                if (stalls.size() < MAX_STALL_ROWS) stalls.add(new double[] {(t0 - beganNs) / 1e6, readMs, interrupts});
            }
            if (readMs >= LOCK_STALL_MS) readsOverLockStall++;
            helper.check();
            long now = System.nanoTime();
            if (!interrupting) {
                if ((now - beganNs) / 1e9 >= BASELINE_S) {
                    interruptingNs = now;
                    report.phase("interrupt_probe interrupting");
                }
                return false;
            }
            boolean last = (now - interruptingNs) / 1e9 >= probeSeconds;
            if (!helper.isAlive()) {
                collect(helper);
                helper = startHelper();
            } else if (!last) {
                long until = now + (long) (random.nextDouble() * MAX_SPIN_MS * 1e6);
                while (System.nanoTime() < until) { }
                helper.interrupt();
                interrupts++;
            }
            report.status("%d interrupts, slowest own read %.0f ms", interrupts, maxReadMs);
            return last;
        }

        void halt() {
            if (joined) return;
            helper.running = false;
            try {
                helper.join(HELPER_JOIN_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("interrupted while joining the interrupt probe's helper", e);
            }
            if (helper.isAlive()) {
                throw new IllegalStateException("the interrupt probe's helper was still running " + HELPER_JOIN_MS
                        + " ms after it was told to stop");
            }
            collect(helper);
            joined = true;
        }

        private void collect(Helper h) {
            h.check();
            helperCommands += h.commands;
            String where = h.landed == null ? NOT_INTERRUPTED : h.landed;
            landed.put(where, landed.get(where) + 1);
            if (h.landed != null) landedCallMs.add(h.callMs);
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("baselineSeconds", BASELINE_S);
            m.put("probeSeconds", probeSeconds);
            m.put("baselineOwnReadMs", baselineMs.summary());
            m.put("baselineOwnReadHistogram", baselineMs.histogram(1, 30));
            m.put("interruptedOwnReadMs", interruptedMs.summary());
            m.put("interruptedOwnReadHistogram", interruptedMs.histogram(25, 30));
            m.put("maxOwnReadMs", maxReadMs);
            m.put("ownReadsOver" + (int) STALL_MS + "Ms", readsOverStall);
            m.put("ownReadsOver" + (int) LOCK_STALL_MS + "Ms", readsOverLockStall);
            m.put("stallColumns", new String[] {"ms", "readMs", "interruptsSoFar"});
            m.put("stalls", stalls);
            m.put("interrupts", interrupts);
            m.put("helperStarts", helperStarts);
            m.put("helperCommands", helperCommands);
            m.put("landed", landed);
            m.put("landedCallMs", landedCallMs.summary());
            m.put("helperJoined", joined);
            return m;
        }

        String line() {
            return String.format(Locale.US, "%d interrupts (%d before transmit, %d awaiting response, %d in the lock release); "
                            + "slowest own read %.0f ms, %d over %.0f ms", interrupts, landed.get(BEFORE_TRANSMIT),
                    landed.get(AWAITING_RESPONSE), landed.get(IN_LOCK_RELEASE), maxReadMs, readsOverLockStall,
                    LOCK_STALL_MS);
        }
    }

    private final class Helper extends Thread {
        private final Thread opModeThread;
        volatile boolean running = true;
        private volatile Throwable failure;
        private String landed;
        private double callMs;
        private long commands;

        Helper(Thread opModeThread) {
            super("BenchInterruptHelper");
            this.opModeThread = opModeThread;
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                while (running && opModeThread.isAlive()) {
                    LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(hub);
                    long t0 = System.nanoTime();
                    try {
                        command.sendReceive();
                    } catch (InterruptedException e) {
                        callMs = (System.nanoTime() - t0) / 1e6;
                        landed = landing(e, command);
                        return;
                    }
                    commands++;
                }
            } catch (Throwable t) {
                failure = t;
            }
        }

        void check() {
            Throwable t = failure;
            if (t != null) throw new IllegalStateException("the interrupt probe's helper thread failed", t);
        }

        private String landing(InterruptedException e, LynxGetBulkInputDataCommand command) {
            for (StackTraceElement frame : e.getStackTrace()) {
                if (frame.getClassName().equals(LOCK_CLASS)) {
                    return frame.getMethodName().equals("release") ? IN_LOCK_RELEASE : BEFORE_TRANSMIT;
                }
            }
            return command.hasBeenTransmitted() ? AWAITING_RESPONSE : BEFORE_TRANSMIT;
        }
    }
}
