package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

/** A growable list of millisecond samples with a percentile summary. Not thread-safe; see {@link #synced()}. */
final class Samples {
    private double[] values = new double[256];
    private int count;

    void add(double ms) {
        if (count == values.length) values = Arrays.copyOf(values, count * 2);
        values[count++] = ms;
    }

    void addNanos(long nanos) {
        add(nanos / 1e6);
    }

    int count() {
        return count;
    }

    double mean() {
        if (count == 0) return Double.NaN;
        double sum = 0;
        for (int i = 0; i < count; i++) sum += values[i];
        return sum / count;
    }

    double percentile(double p) {
        if (count == 0) return Double.NaN;
        double[] sorted = Arrays.copyOf(values, count);
        Arrays.sort(sorted);
        int index = (int) Math.ceil(p / 100.0 * count) - 1;
        return sorted[Math.max(0, Math.min(count - 1, index))];
    }

    Map<String, Object> summary() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("count", count);
        if (count == 0) return m;
        double[] sorted = Arrays.copyOf(values, count);
        Arrays.sort(sorted);
        double sum = 0;
        for (double v : sorted) sum += v;
        m.put("mean", sum / count);
        m.put("min", sorted[0]);
        m.put("p50", pick(sorted, 50));
        m.put("p90", pick(sorted, 90));
        m.put("p95", pick(sorted, 95));
        m.put("p99", pick(sorted, 99));
        m.put("max", sorted[count - 1]);
        int top = Math.min(8, count);
        double[] largest = new double[top];
        for (int i = 0; i < top; i++) largest[i] = sorted[count - 1 - i];
        m.put("largest", largest);
        return m;
    }

    /** Counts per bucket of {@code widthMs}, the last bucket open-ended; shows bimodal loops that percentiles hide. */
    Map<String, Object> histogram(double widthMs, int buckets) {
        int[] counts = new int[buckets];
        for (int i = 0; i < count; i++) {
            int b = (int) (values[i] / widthMs);
            counts[Math.max(0, Math.min(buckets - 1, b))]++;
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("bucketMs", widthMs);
        m.put("counts", counts);
        return m;
    }

    private static double pick(double[] sorted, double p) {
        int index = (int) Math.ceil(p / 100.0 * sorted.length) - 1;
        return sorted[Math.max(0, Math.min(sorted.length - 1, index))];
    }

    /** For samples written on a camera or poller thread and read on the OpMode thread. */
    static final class Synced {
        private final Samples samples = new Samples();

        synchronized void add(double ms) {
            samples.add(ms);
        }

        synchronized int count() {
            return samples.count();
        }

        synchronized Map<String, Object> summary() {
            return samples.summary();
        }
    }

    static Synced synced() {
        return new Synced();
    }
}
