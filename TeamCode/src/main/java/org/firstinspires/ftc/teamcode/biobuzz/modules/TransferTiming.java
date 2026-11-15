package org.firstinspires.ftc.teamcode.biobuzz.modules;

public final class TransferTiming {
    public static final int RECENT = 8;

    public static final class Snapshot {
        public final boolean upstreamOnly;
        public final long periods;
        public final double lastPeriodMs;
        public final double shortestPeriodMs;
        public final double meanPeriodMs;
        public final double lastArmedMs;
        public final double shortestArmedMs;
        public final double lastRearmMs;
        public final double shortestRearmMs;
        private final double[] recentPeriodsMs;

        Snapshot(boolean upstreamOnly, long periods, double lastPeriodMs, double shortestPeriodMs, double meanPeriodMs,
                 double lastArmedMs, double shortestArmedMs, double lastRearmMs, double shortestRearmMs,
                 double[] recentPeriodsMs) {
            this.upstreamOnly = upstreamOnly;
            this.periods = periods;
            this.lastPeriodMs = lastPeriodMs;
            this.shortestPeriodMs = shortestPeriodMs;
            this.meanPeriodMs = meanPeriodMs;
            this.lastArmedMs = lastArmedMs;
            this.shortestArmedMs = shortestArmedMs;
            this.lastRearmMs = lastRearmMs;
            this.shortestRearmMs = shortestRearmMs;
            this.recentPeriodsMs = recentPeriodsMs;
        }

        public double[] recentPeriodsMs() {
            return recentPeriodsMs.clone();
        }
    }

    private final TransferCounter.Edge edge = new TransferCounter.Edge();
    private boolean started;
    private boolean upstreamOnly;
    private double thresholdMm;
    private double hysteresisMm;
    private boolean hasCount;
    private long lastCountNs;
    private boolean armedTimed;
    private long armedAtNs;
    private long periods;
    private long periodSumNs;
    private long lastPeriodNs;
    private long shortestPeriodNs;
    private long lastArmedNs;
    private long shortestArmedNs;
    private long lastRearmNs;
    private long shortestRearmNs;
    private final long[] recentNs = new long[RECENT];
    private int recentSize;
    private int recentNext;
    private Snapshot snapshot;

    public TransferTiming() {
        reset();
    }

    public boolean update(long nowNs, double signal, boolean upstreamOnly, double thresholdMm, double hysteresisMm) {
        // The edge's state predates a first sample or a signal change, so that sample's arming or count times nothing.
        boolean sameSignal = started && upstreamOnly == this.upstreamOnly
                && Double.compare(thresholdMm, this.thresholdMm) == 0
                && Double.compare(hysteresisMm, this.hysteresisMm) == 0;
        if (!sameSignal) {
            reset();
            this.upstreamOnly = upstreamOnly;
            this.thresholdMm = thresholdMm;
            this.hysteresisMm = hysteresisMm;
        }
        boolean wasArmed = edge.armed();
        boolean counted = edge.update(signal, thresholdMm, hysteresisMm);
        if (counted) {
            if (sameSignal) onCount(nowNs);
        } else if (!wasArmed && edge.armed()) {
            armedTimed = sameSignal;
            armedAtNs = nowNs;
            if (hasCount) {
                lastRearmNs = nowNs - lastCountNs;
                shortestRearmNs = Math.min(shortestRearmNs, lastRearmNs);
                snapshot = null;
            }
        }
        started = true;
        return counted;
    }

    private void onCount(long nowNs) {
        if (armedTimed) {
            lastArmedNs = nowNs - armedAtNs;
            shortestArmedNs = Math.min(shortestArmedNs, lastArmedNs);
        }
        armedTimed = false;
        if (hasCount) {
            lastPeriodNs = nowNs - lastCountNs;
            shortestPeriodNs = Math.min(shortestPeriodNs, lastPeriodNs);
            periodSumNs += lastPeriodNs;
            periods++;
            recentNs[recentNext] = lastPeriodNs;
            recentNext = (recentNext + 1) % RECENT;
            recentSize = Math.min(recentSize + 1, RECENT);
        }
        hasCount = true;
        lastCountNs = nowNs;
        snapshot = null;
    }

    public void reset() {
        hasCount = false;
        armedTimed = false;
        periods = 0;
        periodSumNs = 0;
        lastPeriodNs = -1;
        shortestPeriodNs = Long.MAX_VALUE;
        lastArmedNs = -1;
        shortestArmedNs = Long.MAX_VALUE;
        lastRearmNs = -1;
        shortestRearmNs = Long.MAX_VALUE;
        recentSize = 0;
        recentNext = 0;
        snapshot = null;
    }

    public Snapshot snapshot() {
        if (snapshot == null) {
            double[] recent = new double[recentSize];
            int oldest = (recentNext - recentSize + RECENT) % RECENT;
            for (int i = 0; i < recentSize; i++) recent[i] = recentNs[(oldest + i) % RECENT] / 1e6;
            snapshot = new Snapshot(upstreamOnly, periods, ms(lastPeriodNs), ms(shortestPeriodNs),
                    periods > 0 ? periodSumNs / 1e6 / periods : Double.NaN, ms(lastArmedNs), ms(shortestArmedNs),
                    ms(lastRearmNs), ms(shortestRearmNs), recent);
        }
        return snapshot;
    }

    private static double ms(long ns) {
        return ns < 0 || ns == Long.MAX_VALUE ? Double.NaN : ns / 1e6;
    }
}
