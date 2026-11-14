package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Kept free of hardware so it runs under JUnit. */
final class PinpointMath {
    static final double MATCH_SECONDS = 150;
    static final double ROBOT_LOOP_HZ = 33;
    static final double SLOW_MS = 20;
    static final double STALL_MS = 40;

    private PinpointMath() {}

    static double ruleOfThree(long reads) {
        return reads <= 0 ? 1.0 : Math.min(1.0, 3.0 / reads);
    }

    static double atLeastOnce(double rate, double trials) {
        return 1 - Math.pow(1 - rate, trials);
    }

    static double matchReads() {
        return MATCH_SECONDS * ROBOT_LOOP_HZ;
    }

    static double perMatchBound(long reads) {
        return atLeastOnce(ruleOfThree(reads), matchReads());
    }

    static double turnDeg(double aRad, double bRad) {
        double d = Math.toDegrees(Math.IEEEremainder(aRad - bRad, 2 * Math.PI));
        return d == -180 ? 180 : d;
    }

    /** A histogram, so an hour of reads allocates nothing per read. */
    static final class Tail {
        static final double BIN_MS = 0.05;
        static final double RANGE_MS = 100;
        private static final int BINS = (int) Math.round(RANGE_MS / BIN_MS);

        private final int[] bins = new int[BINS + 1];
        private long count;
        private long over20;
        private long over40;
        private double sum;
        private double max = Double.NaN;

        void add(double ms) {
            int b = ms < 0 ? 0 : (int) (ms / BIN_MS);
            bins[Math.min(b, BINS)]++;
            count++;
            sum += ms;
            if (!(ms <= max)) max = ms;
            if (ms > SLOW_MS) over20++;
            if (ms > STALL_MS) over40++;
        }

        long count() {
            return count;
        }

        long over20() {
            return over20;
        }

        long over40() {
            return over40;
        }

        double max() {
            return max;
        }

        double percentile(double p) {
            if (count == 0) return Double.NaN;
            long rank = Math.max(1, (long) Math.ceil(p / 100.0 * count));
            long seen = 0;
            for (int b = 0; b < BINS; b++) {
                seen += bins[b];
                if (seen >= rank) return Math.min((b + 1) * BIN_MS, max);
            }
            return max;
        }

        Map<String, Object> summary() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("count", count);
            if (count == 0) return m;
            m.put("mean", sum / count);
            m.put("p50", percentile(50));
            m.put("p95", percentile(95));
            m.put("p99", percentile(99));
            m.put("max", max);
            m.put("over20", over20);
            m.put("over40", over40);
            m.put("binMs", BIN_MS);
            return m;
        }
    }

    static final class SlowReads {
        private final int keep;
        private final List<double[]> rows = new ArrayList<>();
        private long over20;
        private long over20Gc;
        private long over40;
        private long over40Gc;

        SlowReads(int keep) {
            this.keep = keep;
        }

        void add(long loop, double msSinceStart, double ms, long blockingGcDelta, long gcDelta) {
            if (ms <= SLOW_MS) return;
            boolean gc = blockingGcDelta > 0;
            over20++;
            if (gc) over20Gc++;
            if (ms > STALL_MS) {
                over40++;
                if (gc) over40Gc++;
            }
            if (rows.size() < keep) rows.add(new double[] {loop, msSinceStart, ms, blockingGcDelta, gcDelta});
        }

        long over40WithoutGc() {
            return over40 - over40Gc;
        }

        Map<String, Object> summary(long reads) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("reads", reads);
            m.put("over20", over20);
            m.put("over20WithBlockingGc", over20Gc);
            m.put("over20Rate", reads == 0 ? null : (double) over20 / reads);
            m.put("over40", over40);
            m.put("over40WithBlockingGc", over40Gc);
            m.put("over40WithoutGc", over40WithoutGc());
            m.put("over40Rate", reads == 0 ? null : (double) over40 / reads);
            m.put("rowColumns", new String[] {"loop", "msSinceStart", "readMs", "blockingGcDelta", "gcDelta"});
            m.put("rows", rows);
            m.put("rowsDropped", Math.max(0, over20 - rows.size()));
            return m;
        }
    }

    static final class Streaks {
        private final int keep;
        private final List<Map<String, Object>> streaks = new ArrayList<>();
        private final Map<String, Long> statusCounts = new LinkedHashMap<>();
        private boolean seenReady;
        private long readsAfterReady;
        private long notReady;
        private long count;
        private Map<String, Object> open;
        private double openStartMs;
        private long openReads;
        private double longestMs;
        private long longestReads;
        private double guardStopMs = Double.NaN;
        private String guardStopPhase;

        Streaks(int keep) {
            this.keep = keep;
        }

        void add(double msSinceCreate, String status, boolean ready, String phase, String pose) {
            Long c = statusCounts.get(status);
            statusCounts.put(status, c == null ? 1 : c + 1);
            if (!seenReady) {
                if (!ready) return;
                seenReady = true;
            }
            readsAfterReady++;
            if (ready) {
                if (open != null) close(msSinceCreate);
                return;
            }
            notReady++;
            if (open == null) {
                count++;
                if (Double.isNaN(guardStopMs)) {
                    guardStopMs = msSinceCreate;
                    guardStopPhase = phase;
                }
                open = new LinkedHashMap<>();
                open.put("startMs", msSinceCreate);
                open.put("phase", phase);
                open.put("status", status);
                open.put("pose", pose);
                openStartMs = msSinceCreate;
                openReads = 0;
                if (streaks.size() < keep) streaks.add(open);
            }
            openReads++;
            open.put("reads", openReads);
            open.put("lastStatus", status);
        }

        private void close(double readyMs) {
            double ms = readyMs - openStartMs;
            open.put("readyAgainAfterMs", ms);
            longestMs = Math.max(longestMs, ms);
            longestReads = Math.max(longestReads, openReads);
            open = null;
        }

        long notReady() {
            return notReady;
        }

        long readsAfterReady() {
            return readsAfterReady;
        }

        double guardStopMs() {
            return guardStopMs;
        }

        Map<String, Object> summary(double nowMs) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("statusCounts", statusCounts);
            m.put("seenReady", seenReady);
            m.put("readsAfterFirstReady", readsAfterReady);
            m.put("notReadyAfterFirstReady", notReady);
            m.put("streakCount", count);
            m.put("longestStreakReads", Math.max(longestReads, open == null ? 0 : openReads));
            m.put("longestStreakMs", Math.max(longestMs, open == null ? 0 : nowMs - openStartMs));
            m.put("openAtEnd", open != null);
            m.put("guardWouldStopMs", guardStopMs);
            m.put("guardWouldStopPhase", guardStopPhase);
            m.put("streaks", streaks);
            m.put("streaksDropped", Math.max(0, count - streaks.size()));
            m.put("perReadBound", ruleOfThree(readsAfterReady));
            m.put("perMatchBound", perMatchBound(readsAfterReady));
            m.put("matchReads", matchReads());
            return m;
        }
    }

    static final class Drift {
        static final double STEP_S = 5;
        static final double WINDOW_S = 120;

        private double t0 = Double.NaN;
        private double lastRaw;
        private double unwrappedDeg;
        private double at5 = Double.NaN;
        private double atWindow = Double.NaN;
        private double maxAbs;
        private double lastT;
        private long n;
        private double st;
        private double sh;
        private double stt;
        private double sth;
        private long samples;

        void add(double seconds, double headingRad) {
            if (Double.isNaN(t0)) {
                t0 = seconds;
                lastRaw = headingRad;
            }
            unwrappedDeg += turnDeg(headingRad, lastRaw);
            lastRaw = headingRad;
            double t = seconds - t0;
            lastT = t;
            samples++;
            maxAbs = Math.max(maxAbs, Math.abs(unwrappedDeg));
            if (Double.isNaN(at5) && t >= STEP_S) at5 = unwrappedDeg;
            if (Double.isNaN(atWindow) && t >= WINDOW_S) atWindow = unwrappedDeg;
            if (t >= STEP_S) {
                double minutes = t / 60;
                n++;
                st += minutes;
                sh += unwrappedDeg;
                stt += minutes * minutes;
                sth += minutes * unwrappedDeg;
            }
        }

        double firstStepDeg() {
            return at5;
        }

        double windowDeg() {
            return atWindow - at5;
        }

        double slopeDegPerMin() {
            double d = n * stt - st * st;
            if (n < 2 || d == 0) return Double.NaN;
            return (n * sth - st * sh) / d;
        }

        double maxAbsDeg() {
            return maxAbs;
        }

        Map<String, Object> summary() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("samples", samples);
            m.put("seconds", lastT);
            m.put("change0to5Deg", at5);
            m.put("change5to120Deg", windowDeg());
            m.put("slopeDegPerMin", slopeDegPerMin());
            m.put("maxAbsChangeDeg", samples == 0 ? Double.NaN : maxAbs);
            m.put("finalChangeDeg", samples == 0 ? Double.NaN : unwrappedDeg);
            return m;
        }
    }
}
