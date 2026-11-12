package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppAliveNotifier;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "Bench: Freeze Dump", group = "Test")
public class FreezeDumpBench extends OpMode {
    static final String WATCH_THREAD = "BenchFreezeWatch";
    static final String INDEX = "freeze_watch";
    static final String DUMP_PREFIX = "freeze_dump_";
    static final String STOP_FILE = "freeze_watch.stop";

    private BenchReport report;
    private Watch watch;

    @Override
    public void init() {
        report = new BenchReport("Freeze Dump", "freeze_dump");
        for (Thread t : Thread.getAllStackTraces().keySet()) {
            if (t.getName().equals(WATCH_THREAD)) {
                throw new IllegalStateException("a " + WATCH_THREAD + " from an earlier run is still armed: create "
                        + STOP_FILE + " in " + BenchIO.dir() + " or wait for it to expire, then INIT again");
            }
        }
        File dir = BenchIO.dir();
        File[] old = dir.listFiles();
        if (old != null) {
            for (File f : old) {
                if (f.getName().startsWith(DUMP_PREFIX) && !f.delete()) throw new IllegalStateException("can't delete " + f);
            }
        }
        File stop = new File(dir, STOP_FILE);
        if (stop.exists() && !stop.delete()) throw new IllegalStateException("can't delete " + stop);
        watch = new Watch(report.params, aliveTimer());
        report.put("watch", watch.config());
        report.save();
        watch.start();
    }

    // The event loop thread sends the Control Hub OS this broadcast; ~10 s without one, the OS kills the app.
    private static ElapsedTime aliveTimer() {
        try {
            AppAliveNotifier notifier = AppAliveNotifier.getInstance();
            Field enabled = AppAliveNotifier.class.getDeclaredField("enabled");
            enabled.setAccessible(true);
            if (!enabled.getBoolean(notifier)) {
                throw new IllegalStateException("AppAliveNotifier is off: this device has no Control Hub OS watchdog");
            }
            Field timer = AppAliveNotifier.class.getDeclaredField("timeSinceLastAliveNotification");
            timer.setAccessible(true);
            return (ElapsedTime) timer.get(notifier);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("can't read AppAliveNotifier's alive timer", e);
        }
    }

    @Override
    public void init_loop() {
        tick();
    }

    @Override
    public void loop() {
        tick();
    }

    private void tick() {
        watch.beat();
        watch.check();
        if (!report.isDone() && watch.snapshots() > 0) {
            report.done(String.format(Locale.US, "armed for %.0f s; STOP it, then send a hardware-config switch", watch.armSeconds));
        }
        report.sendIfDue();
    }

    @Override
    public void stop() {
        watch.opModeEnded();
    }

    private static final class ThreadState {
        final String name;
        final long id;
        final Thread.State state;
        final boolean daemon;
        final int priority;
        final StackTraceElement[] frames;

        ThreadState(Thread t, StackTraceElement[] frames) {
            name = t.getName();
            id = t.getId();
            state = t.getState();
            daemon = t.isDaemon();
            priority = t.getPriority();
            this.frames = frames;
        }

        boolean suspect() {
            if (state == Thread.State.BLOCKED) return true;
            if (state == Thread.State.TERMINATED) return false;
            for (StackTraceElement f : frames) {
                String c = f.getClassName();
                String m = f.getMethodName();
                if (c.equals("com.acmerobotics.dashboard.Mutex") || m.equals("requestRobotRestart")
                        || (c.contains("DashWebSocket") && m.equals("onMessage"))) {
                    return true;
                }
            }
            return false;
        }

        void write(Writer w, int maxFrames) throws IOException {
            w.write(String.format(Locale.US, "\"%s\" id=%d state=%s%s prio=%d%n", name, id, state,
                    daemon ? " daemon" : "", priority));
            for (int i = 0; i < Math.min(frames.length, maxFrames); i++) w.write("    at " + frames[i] + "\n");
            if (frames.length > maxFrames) w.write("    ... " + (frames.length - maxFrames) + " more\n");
        }

        String top() {
            return name + " " + state + (frames.length > 0 ? " at " + frames[0] : "");
        }
    }

    private static final class Snapshot {
        final long nanos;
        final long wallMs = System.currentTimeMillis();
        final List<ThreadState> threads = new ArrayList<>();

        Snapshot(long nanos) {
            this.nanos = nanos;
            for (Map.Entry<Thread, StackTraceElement[]> e : Thread.getAllStackTraces().entrySet()) {
                threads.add(new ThreadState(e.getKey(), e.getValue()));
            }
            Collections.sort(threads, (a, b) -> a.name.compareTo(b.name));
        }

        List<String> suspects() {
            List<String> list = new ArrayList<>();
            for (ThreadState t : threads) if (t.suspect()) list.add(t.top());
            return list;
        }
    }

    private static final class Watch extends Thread {
        private static final int FULL_FRAMES = 200;
        private static final int SUSPECT_FRAMES = 40;

        final double armSeconds;
        private final long armNs;
        private final double stallMs;
        private final double loopStallMs;
        private final long snapshotMs;
        private final long dumpEveryNs;
        private final int maxDumps;
        private final int ringSize;
        private final boolean sigquit;
        private final ElapsedTime alive;
        private final File dir = BenchIO.dir();
        private final ArrayDeque<Snapshot> ring = new ArrayDeque<>();
        private final Map<String, Object> index = new LinkedHashMap<>();
        private final List<Map<String, Object>> events = new ArrayList<>();
        private final SimpleDateFormat clock = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US);
        private volatile long loopNs = System.nanoTime();
        private volatile boolean opModeRunning = true;
        private volatile int snapshots;
        private volatile Throwable failure;
        private long armedNs;
        private long stallNs;
        private long nextDumpNs;
        private int dumps;
        private double maxQuietAgeMs;
        private boolean sigquitSent;

        Watch(Map<String, Object> params, ElapsedTime alive) {
            super(WATCH_THREAD);
            setDaemon(true);
            this.alive = alive;
            armSeconds = BenchIO.param(params, "armSeconds", 120);
            armNs = (long) (armSeconds * 1e9);
            stallMs = BenchIO.param(params, "stallMs", 3500);
            loopStallMs = BenchIO.param(params, "loopStallMs", 1500);
            snapshotMs = (long) BenchIO.param(params, "snapshotMs", 500);
            dumpEveryNs = (long) (BenchIO.param(params, "dumpEveryMs", 2000) * 1e6);
            maxDumps = (int) BenchIO.param(params, "maxDumps", 8);
            ringSize = (int) BenchIO.param(params, "ring", 6);
            sigquit = BenchIO.param(params, "sigquit", 1) != 0;
        }

        Map<String, Object> config() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("armSeconds", armSeconds);
            m.put("stallMs", stallMs);
            m.put("loopStallMs", loopStallMs);
            m.put("snapshotMs", snapshotMs);
            m.put("dumpEveryMs", dumpEveryNs / 1e6);
            m.put("maxDumps", maxDumps);
            m.put("ring", ringSize);
            m.put("sigquit", sigquit);
            m.put("aliveAgeMsAtInit", alive.milliseconds());
            return m;
        }

        void beat() {
            loopNs = System.nanoTime();
        }

        void opModeEnded() {
            opModeRunning = false;
        }

        int snapshots() {
            return snapshots;
        }

        void check() {
            Throwable t = failure;
            if (t != null) throw new IllegalStateException("the freeze watch thread failed", t);
        }

        @Override
        public void run() {
            try {
                armedNs = System.nanoTime();
                index.put("pid", android.os.Process.myPid());
                index.put("armedWallMs", System.currentTimeMillis());
                index.put("config", config());
                index.put("events", events);
                event("armed", armedNs, alive.milliseconds(), null, null, null);
                String end = "armExpired";
                while (System.nanoTime() - armedNs < armNs) {
                    if (new File(dir, STOP_FILE).exists()) {
                        end = "stopFile";
                        break;
                    }
                    if (dumps >= maxDumps) {
                        end = "maxDumps";
                        break;
                    }
                    tick();
                    Thread.sleep(snapshotMs);
                }
                event("ended", System.nanoTime(), alive.milliseconds(), end, null, null);
            } catch (Throwable t) {
                failure = t;
                RobotLog.ee(BenchIO.TAG, t, "BENCH freeze watch failed");
                StackTraceElement[] frames = t.getStackTrace();
                index.put("failure", t.toString());
                index.put("failureFrames", Arrays.toString(Arrays.copyOf(frames, Math.min(8, frames.length))));
                // A throw here would kill the whole app from this thread; the failure is already logged and rethrown by check().
                try {
                    BenchIO.write(INDEX, index);
                } catch (RuntimeException e) {
                    RobotLog.ee(BenchIO.TAG, e, "BENCH freeze watch couldn't record its failure");
                }
            }
        }

        private void tick() throws IOException {
            long now = System.nanoTime();
            Snapshot s = new Snapshot(now);
            ring.addLast(s);
            while (ring.size() > ringSize) ring.removeFirst();
            snapshots++;
            double age = alive.milliseconds();
            String reason = null;
            if (age > stallMs) {
                reason = String.format(Locale.US, "no RC alive broadcast for %.0f ms", age);
            } else if (opModeRunning && (now - loopNs) / 1e6 > loopStallMs) {
                reason = String.format(Locale.US, "the OpMode's loop() not called for %.0f ms", (now - loopNs) / 1e6);
            }
            if (reason == null) {
                maxQuietAgeMs = Math.max(maxQuietAgeMs, age);
                if (stallNs != 0) {
                    event("resumed", now, age, String.format(Locale.US, "stalled %.0f ms", (now - stallNs) / 1e6), null,
                            s.suspects());
                    stallNs = 0;
                    sigquitSent = false;
                }
                return;
            }
            if (stallNs == 0) {
                stallNs = now;
                if (sigquit && !sigquitSent) {
                    android.os.Process.sendSignal(android.os.Process.myPid(), android.os.Process.SIGNAL_QUIT);
                    sigquitSent = true;
                    index.put("sigquitWallMs", System.currentTimeMillis());
                }
                dump(reason, now, age, new ArrayList<>(ring));
                nextDumpNs = now + dumpEveryNs;
            } else if (now >= nextDumpNs) {
                dump(reason, now, age, Collections.singletonList(s));
                nextDumpNs += dumpEveryNs;
            }
        }

        private void dump(String reason, long now, double age, List<Snapshot> shots) throws IOException {
            dumps++;
            String name = String.format(Locale.US, "%s%02d.txt", DUMP_PREFIX, dumps);
            Snapshot latest = shots.get(shots.size() - 1);
            try (FileOutputStream out = new FileOutputStream(new File(dir, name));
                 Writer w = new BufferedWriter(new OutputStreamWriter(out, StandardCharsets.UTF_8))) {
                w.write("BENCH freeze dump " + dumps + " (at most " + maxDumps + ")\n");
                w.write("reason: " + reason + "\n");
                w.write(String.format(Locale.US, "wall %s (%d ms), uptime %d ms, elapsedRealtime %d ms, %.0f ms after arming%n",
                        clock.format(new Date(latest.wallMs)), latest.wallMs, SystemClock.uptimeMillis(),
                        SystemClock.elapsedRealtime(), (now - armedNs) / 1e6));
                w.write(String.format(Locale.US, "RC alive age %.0f ms; stalled for %.0f ms; pid %d; SIGQUIT %s%n", age,
                        (now - stallNs) / 1e6, android.os.Process.myPid(), sigquitSent ? "sent at the first dump" : "not sent"));
                w.write("\n=== suspects in the latest snapshot (BLOCKED, or waiting in the event loop, OpMode manager, "
                        + "RC service or dashboard) ===\n");
                for (ThreadState t : latest.threads) if (t.suspect()) t.write(w, SUSPECT_FRAMES);
                for (Snapshot shot : shots) {
                    w.write(String.format(Locale.US, "%n=== snapshot %+.0f ms from this dump, wall %s, %d threads ===%n",
                            (shot.nanos - now) / 1e6, clock.format(new Date(shot.wallMs)), shot.threads.size()));
                    for (ThreadState t : shot.threads) t.write(w, FULL_FRAMES);
                }
                w.flush();
                out.getFD().sync();
            }
            event("dump", now, age, reason, name, latest.suspects());
        }

        private void event(String type, long now, double age, String detail, String file, List<String> suspects) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("type", type);
            m.put("sinceArmMs", (now - armedNs) / 1e6);
            m.put("wallMs", System.currentTimeMillis());
            m.put("aliveAgeMs", age);
            m.put("snapshots", snapshots);
            if (detail != null) m.put("detail", detail);
            if (file != null) m.put("file", file);
            if (suspects != null) m.put("suspects", suspects);
            events.add(m);
            index.put("dumps", dumps);
            index.put("maxAliveAgeMsOutsideStalls", maxQuietAgeMs);
            BenchIO.write(INDEX, index);
            BenchIO.log("freeze watch %s%s", type, detail == null ? "" : ": " + detail);
        }
    }
}
