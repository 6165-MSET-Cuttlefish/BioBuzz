package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Timed samples (a loop period, a frame gap) summarised over a time range or in fixed windows. */
final class SoakWindows {
    private double[] tMs = new double[1024];
    private double[] values = new double[1024];
    private int count;

    void add(double atMs, double value) {
        if (count == values.length) {
            tMs = Arrays.copyOf(tMs, count * 2);
            values = Arrays.copyOf(values, count * 2);
        }
        tMs[count] = atMs;
        values[count] = value;
        count++;
    }

    int count() {
        return count;
    }

    /** Samples timed in [fromMs, toMs). */
    Map<String, Object> between(double fromMs, double toMs) {
        double[] picked = new double[count];
        int n = 0;
        for (int i = 0; i < count; i++) {
            if (tMs[i] >= fromMs && tMs[i] < toMs) picked[n++] = values[i];
        }
        return summary(Arrays.copyOf(picked, n));
    }

    /** One summary per widthMs window from the first sample's window to the last, empty windows included. */
    List<Map<String, Object>> windows(double widthMs) {
        List<Map<String, Object>> out = new ArrayList<>();
        if (count == 0) return out;
        double first = Math.floor(tMs[0] / widthMs) * widthMs;
        for (double start = first; start <= tMs[count - 1]; start += widthMs) {
            Map<String, Object> w = between(start, start + widthMs);
            w.put("startMs", start);
            out.add(w);
        }
        return out;
    }

    static Map<String, Object> summary(double[] v) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("count", v.length);
        if (v.length == 0) return m;
        double[] sorted = Arrays.copyOf(v, v.length);
        Arrays.sort(sorted);
        double sum = 0;
        for (double x : sorted) sum += x;
        m.put("mean", sum / sorted.length);
        m.put("p50", percentile(sorted, 50));
        m.put("p95", percentile(sorted, 95));
        m.put("max", sorted[sorted.length - 1]);
        return m;
    }

    static double percentile(double[] sorted, double p) {
        int index = (int) Math.ceil(p / 100.0 * sorted.length) - 1;
        return sorted[Math.max(0, Math.min(sorted.length - 1, index))];
    }
}
