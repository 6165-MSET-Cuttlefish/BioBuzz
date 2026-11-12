package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.LockSupport;

// Outlives the OpMode thread by 1.5 s on purpose: what the hubs do after STOP is what it measures.
final class StopSampler {
    private static final long AFTER_END_NS = 1_500_000_000L;
    private static final long DEADLINE_NS = 60_000_000_000L;
    private static final long GATE_POLL_NS = 250_000L;
    private static final int MAX_SWEEPS = 4000;
    private static final int MAX_FAILED_SWEEPS_IN_A_ROW = 5;
    private static final int LOOP_RING = 64;

    private final String fileName;
    private final PhotonRig rig;
    private final Thread opModeThread;
    private final Map<String, Object> info;
    private final long[] loopStarts = new long[LOOP_RING];
    private volatile int loops;
    private final List<HubSweeper> sweepers = new ArrayList<>();
    private volatile long startNs;
    private volatile long gateNs;
    private volatile long opModeEndNs;
    private volatile boolean finished;
    private volatile Throwable failure;
    private volatile Map<String, Object> extra = new LinkedHashMap<>();

    StopSampler(String fileName, PhotonRig rig, Thread opModeThread, Map<String, Object> info) {
        this.fileName = fileName;
        this.rig = rig;
        this.opModeThread = opModeThread;
        this.info = info;
    }

    void start() {
        startNs = System.nanoTime();
        for (LynxModule hub : rig.hubs) sweepers.add(new HubSweeper(hub));
        for (HubSweeper s : sweepers) s.start();
        Thread watcher = new Thread(this::watch, "PhotonStopGateWatcher");
        watcher.setDaemon(true);
        watcher.start();
    }

    void noteLoop() {
        int n = loops;
        loopStarts[n % LOOP_RING] = System.nanoTime();
        loops = n + 1;
    }

    void put(String key, Object value) {
        Map<String, Object> m = new LinkedHashMap<>(extra);
        m.put(key, value);
        extra = m;
    }

    boolean failSafeSeenOnEveryHub() {
        for (HubSweeper s : sweepers) if (!s.failSafeSeen) return false;
        return !sweepers.isEmpty();
    }

    private void watch() {
        try {
            long deadline = startNs + DEADLINE_NS;
            while (!PhotonProbe.gateUp() && System.nanoTime() < deadline) LockSupport.parkNanos(GATE_POLL_NS);
            if (PhotonProbe.gateUp()) gateNs = System.nanoTime();
            Map<String, Object> phase = loopPhaseAtGate();
            while (opModeThread.isAlive() && System.nanoTime() < deadline) LockSupport.parkNanos(1_000_000L);
            opModeEndNs = System.nanoTime();
            long end = Math.min(deadline, opModeEndNs + AFTER_END_NS);
            while (System.nanoTime() < end) LockSupport.parkNanos(2_000_000L);
            finished = true;
            for (HubSweeper s : sweepers) s.join(2000);
            write(phase);
        } catch (Throwable t) {
            failure = t;
            BenchIO.log("stop sampler failed: %s", t);
            write(null);
        }
    }

    private Map<String, Object> loopPhaseAtGate() {
        Map<String, Object> m = new LinkedHashMap<>();
        int n = loops;
        m.put("loopsBeforeGate", n);
        if (gateNs == 0 || n < 3) return m;
        long last = 0;
        long prev = 0;
        for (int i = n - 1; i >= Math.max(0, n - LOOP_RING); i--) {
            long t = loopStarts[i % LOOP_RING];
            if (t <= gateNs) {
                last = t;
                prev = loopStarts[(i - 1 + LOOP_RING) % LOOP_RING];
                break;
            }
        }
        if (last == 0 || prev == 0 || last <= prev) return m;
        double period = (last - prev) / 1e6;
        double into = (gateNs - last) / 1e6;
        m.put("loopPeriodMs", period);
        m.put("msIntoLoop", into);
        m.put("fractionOfLoop", Math.min(1.0, into / period));
        return m;
    }

    private synchronized void write(Map<String, Object> phase) {
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("bench", "Photon Stop Sampler");
        root.put("complete", failure == null && gateNs != 0);
        root.put("info", info);
        root.put("extra", extra);
        root.put("gateSeen", gateNs != 0);
        root.put("msStartToGate", gateNs == 0 ? null : (gateNs - startNs) / 1e6);
        root.put("msGateToOpModeEnd", gateNs == 0 || opModeEndNs == 0 ? null : (opModeEndNs - gateNs) / 1e6);
        root.put("loopAtGate", phase);
        root.put("failure", failure == null ? null : failure.toString());
        List<Map<String, Object>> hubs = new ArrayList<>();
        for (HubSweeper s : sweepers) hubs.add(s.toMap());
        root.put("hubs", hubs);
        BenchIO.write(fileName, root);
        BenchIO.log("BENCH DONE %s: stop sampler wrote %d hubs", fileName, hubs.size());
    }

    private final class HubSweeper extends Thread {
        private final LynxModule hub;
        private final List<PhotonChannel> channels;
        private final List<double[]> sweeps = new ArrayList<>();
        private final List<Double> failedSweepsMs = new ArrayList<>();
        private volatile boolean failSafeSeen;
        private volatile Throwable error;

        HubSweeper(LynxModule hub) {
            super("PhotonStopSampler-" + hub.getModuleAddress());
            this.hub = hub;
            channels = rig.on(hub);
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                int failedInARow = 0;
                while (!finished && System.nanoTime() - startNs < DEADLINE_NS) {
                    long t0 = System.nanoTime();
                    boolean gate = PhotonProbe.gateUp();
                    double[] row = new double[3 + 2 * channels.size()];
                    boolean allOff = true;
                    try {
                        for (int i = 0; i < channels.size(); i++) {
                            PhotonChannel c = channels.get(i);
                            boolean on = c.enabled();
                            int raw = on && c.isMotor() ? HubRegisters.motorPowerRaw(hub, c.port) : 0;
                            row[3 + 2 * i] = on ? 1 : 0;
                            row[4 + 2 * i] = raw;
                            allOff &= !on;
                        }
                    } catch (IllegalStateException e) {
                        // A write given up marks the hub not responding until its next reply: the SDK fails reads in between.
                        if (++failedInARow > MAX_FAILED_SWEEPS_IN_A_ROW) throw e;
                        if (failedSweepsMs.size() < MAX_SWEEPS) failedSweepsMs.add((t0 - startNs) / 1e6);
                        continue;
                    }
                    failedInARow = 0;
                    long t1 = System.nanoTime();
                    row[0] = (t0 - startNs) / 1e6;
                    row[1] = (t1 - startNs) / 1e6;
                    row[2] = gate ? 1 : 0;
                    if (gate && allOff) failSafeSeen = true;
                    if (sweeps.size() < MAX_SWEEPS) sweeps.add(row);
                }
            } catch (Throwable t) {
                error = t;
            }
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("address", hub.getModuleAddress());
            m.put("key", rig.key(hub));
            List<String> names = new ArrayList<>();
            List<String> kinds = new ArrayList<>();
            for (PhotonChannel c : channels) {
                names.add(c.name);
                kinds.add(c.isMotor() ? "motor" : "servo");
            }
            m.put("channels", names);
            m.put("kinds", kinds);
            m.put("columns", "startMs, endMs, gateUp, then enabled and raw power per channel (raw 0 when disabled or a servo)");
            m.put("gateMs", gateNs == 0 ? null : (gateNs - startNs) / 1e6);
            m.put("sweeps", new ArrayList<>(sweeps));
            m.put("failedSweepsMs", new ArrayList<>(failedSweepsMs));
            m.put("error", error == null ? null : error.toString());
            return m;
        }
    }
}
