package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.atomic.AtomicInteger;

public final class TransferCounter {
    private static final long JOIN_TIMEOUT_MS = 250;
    private static final double ADC_FULL_SCALE_MV = 3300;
    // The hub refreshes its ADC about every 10 ms (p10 9.3 ms); a gap longer than this can skip a refresh.
    private static final long LATE_POLL_NS = 9_000_000L;

    public static final class Edge {
        private boolean armed;

        public boolean update(double diffMm, double thresholdMm, double hysteresisMm) {
            if (armed && diffMm > thresholdMm) {
                armed = false;
                return true;
            }
            if (diffMm <= thresholdMm - hysteresisMm) armed = true;
            return false;
        }
    }

    public static final class Stats {
        public final double mmA;
        public final double mmB;
        public final long polls;
        public final long newValues;
        public final long latePolls;
        public final double maxGapMs;
        public final double meanCommandMs;
        public final double seconds;

        Stats(double mmA, double mmB, long polls, long newValues, long latePolls, double maxGapMs, double meanCommandMs,
              double seconds) {
            this.mmA = mmA;
            this.mmB = mmB;
            this.polls = polls;
            this.newValues = newValues;
            this.latePolls = latePolls;
            this.maxGapMs = maxGapMs;
            this.meanCommandMs = meanCommandMs;
            this.seconds = seconds;
        }
    }

    private final LynxModule hub;
    private final int portA;
    private final int portB;
    private final double maxRangeMm;
    private final long periodNs;
    private final AtomicInteger count = new AtomicInteger();
    private volatile double thresholdMm;
    private volatile double hysteresisMm;
    private volatile boolean running;
    private volatile Throwable failure;
    private volatile Stats stats;
    private Thread thread;

    public TransferCounter(HardwareMap hardwareMap, String upstreamName, String downstreamName, double maxRangeMm,
                           double periodMs) {
        AnalogInput a = hardwareMap.get(AnalogInput.class, upstreamName);
        AnalogInput b = hardwareMap.get(AnalogInput.class, downstreamName);
        LynxModule found = null;
        for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
            if (portOn(m, a) >= 0) found = m;
        }
        if (found == null) throw new IllegalStateException(upstreamName + " (" + a.getConnectionInfo() + ") is on no REV hub");
        hub = found;
        portA = portOn(hub, a);
        portB = portOn(hub, b);
        if (portB < 0) {
            throw new IllegalStateException(downstreamName + " must be on the same hub as " + upstreamName + ", is on "
                    + b.getConnectionInfo());
        }
        if (!(periodMs > 0)) throw new IllegalArgumentException("periodMs must be positive, was " + periodMs);
        this.maxRangeMm = maxRangeMm;
        this.periodNs = (long) (periodMs * 1e6);
    }

    private static int portOn(LynxModule module, AnalogInput input) {
        for (int port = LynxAnalogInputController.apiPortFirst; port <= LynxAnalogInputController.apiPortLast; port++) {
            if (input.getConnectionInfo().equals(module.getConnectionInfo() + "; analog port " + port)) return port;
        }
        return -1;
    }

    public void setThreshold(double thresholdMm, double hysteresisMm) {
        this.thresholdMm = thresholdMm;
        this.hysteresisMm = hysteresisMm;
    }

    public void start() {
        if (thread != null) throw new IllegalStateException("TransferCounter already started");
        Thread opModeThread = Thread.currentThread();
        running = true;
        thread = new Thread(() -> run(opModeThread), "TransferCounter");
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        if (thread == null) return;
        running = false;
        try {
            thread.join(JOIN_TIMEOUT_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted while stopping the transfer counter", e);
        }
        if (thread.isAlive()) {
            throw new IllegalStateException("TransferCounter still running " + JOIN_TIMEOUT_MS + " ms after stop");
        }
        thread = null;
        check();
    }

    public int count() {
        check();
        return count.get();
    }

    public void resetCount() {
        count.set(0);
    }

    public Stats stats() {
        check();
        return stats;
    }

    private void check() {
        Throwable t = failure;
        if (t != null) throw new IllegalStateException("TransferCounter thread died", t);
    }

    private void run(Thread opModeThread) {
        try {
            Edge edge = new Edge();
            long startNs = System.nanoTime();
            long next = startNs;
            long lastPollNs = 0;
            long polls = 0;
            long newValues = 0;
            long latePolls = 0;
            long maxGapNs = 0;
            long commandNs = 0;
            int lastMvA = -1;
            while (running && opModeThread.isAlive()) {
                long t0 = System.nanoTime();
                LynxGetBulkInputDataResponse r = new LynxGetBulkInputDataCommand(hub).sendReceive();
                long t1 = System.nanoTime();
                int mvA = r.getAnalogInput(portA);
                int mvB = r.getAnalogInput(portB);
                double mmA = mvA / ADC_FULL_SCALE_MV * maxRangeMm;
                double mmB = mvB / ADC_FULL_SCALE_MV * maxRangeMm;
                if (edge.update(mmA - mmB, thresholdMm, hysteresisMm)) count.incrementAndGet();

                if (polls > 0) {
                    long gap = t1 - lastPollNs;
                    maxGapNs = Math.max(maxGapNs, gap);
                    if (gap > LATE_POLL_NS) latePolls++;
                    if (mvA != lastMvA) newValues++;
                }
                lastPollNs = t1;
                lastMvA = mvA;
                polls++;
                commandNs += t1 - t0;
                stats = new Stats(mmA, mmB, polls, newValues, latePolls, maxGapNs / 1e6, commandNs / 1e6 / polls,
                        (t1 - startNs) / 1e9);

                next += periodNs;
                long wait = next - System.nanoTime();
                if (wait > 0) Thread.sleep(wait / 1_000_000, (int) (wait % 1_000_000));
                else next = System.nanoTime();
            }
        } catch (Throwable t) {
            failure = t;
        }
    }
}
