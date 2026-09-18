package org.firstinspires.ftc.teamcode.architecture.telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class LoopProfiler {

    private static final int WINDOW = 20;

    private final LinkedHashMap<String, Bucket> sections = new LinkedHashMap<>();
    private long markAnchorNs;

    public boolean enabled = true;

    public void start() {
        if (!enabled) return;
        markAnchorNs = System.nanoTime();
    }

    public void mark(String section) {
        if (!enabled) return;
        long now = System.nanoTime();
        accumulate(section, (now - markAnchorNs) / 1_000_000.0, true);
        markAnchorNs = now;
    }

    /** Tokens are single-use — leaving twice with the same token double-counts the elapsed time. */
    public long enterSection() {
        return enabled ? System.nanoTime() : 0L;
    }

    public void leaveSection(String section, long startTokenNs) {
        if (!enabled || startTokenNs == 0L) return;
        accumulate(section, (System.nanoTime() - startTokenNs) / 1_000_000.0, false);
    }

    public List<Map.Entry<String, Double>> snapshotSortedDesc() {
        List<Map.Entry<String, Double>> out = new ArrayList<>(sections.size());
        for (Map.Entry<String, Bucket> e : sections.entrySet()) {
            out.add(new java.util.AbstractMap.SimpleEntry<>(e.getKey(), e.getValue().avg()));
        }
        Collections.sort(out, (a, b) -> Double.compare(b.getValue(), a.getValue()));
        return out;
    }

    /**
     * One compact, paste-friendly line. Header: loop count, whole-run loop avg/max, the sum of the
     * top-level stage marks, and {@code unprofiled} (loopAvg - sumMarks = the post-render minLoopMs
     * hold-sleep + record overhead — should be ~0 unless minLoopMs is set). Then each section as
     * {@code name=avg/peak/count} sorted by avg desc; {@code *} flags nested drill-downs (read./write./
     * auto.*) that are already counted inside a stage — do not add them to the stage total.
     */
    public String report(long loops, double loopAvgMs, double loopMaxMs) {
        List<Map.Entry<String, Bucket>> entries = new ArrayList<>(sections.entrySet());
        double sumMarks = 0;
        for (int i = 0; i < entries.size(); i++) {
            if (entries.get(i).getValue().partition) sumMarks += entries.get(i).getValue().lifetimeAvg();
        }
        Collections.sort(entries, (a, b) ->
                Double.compare(b.getValue().lifetimeAvg(), a.getValue().lifetimeAvg()));
        StringBuilder sb = new StringBuilder(700);
        sb.append("loops=").append(loops)
          .append(" loopAvg=").append(String.format("%.2f", loopAvgMs))
          .append(" loopMax=").append(String.format("%.1f", loopMaxMs))
          .append(" sumMarks=").append(String.format("%.2f", sumMarks))
          .append(" unprofiled=").append(String.format("%.2f", loopAvgMs - sumMarks)).append("ms ||");
        for (int i = 0; i < entries.size(); i++) {
            Bucket b = entries.get(i).getValue();
            sb.append(' ').append(entries.get(i).getKey()).append(b.partition ? "=" : "*=")
              .append(String.format("%.2f", b.lifetimeAvg())).append('/')
              .append(String.format("%.1f", b.peak)).append('/').append(b.count).append(';');
        }
        return sb.toString();
    }

    public void reset() {
        sections.clear();
        markAnchorNs = 0L;
    }

    private void accumulate(String section, double ms, boolean partition) {
        Bucket b = sections.get(section);
        if (b == null) {
            b = new Bucket();
            sections.put(section, b);
        }
        b.partition = partition;
        b.add(ms);
    }

    private static final class Bucket {
        final double[] window = new double[WINDOW];
        int idx = 0;
        int filled = 0;
        double sum = 0;
        // Lifetime (since reset) stats for the paste-back dump; the window above drives the live view.
        long count = 0;
        double total = 0;
        double peak = 0;
        // True for a top-level stage (mark()); false for a nested drill-down (leaveSection()).
        boolean partition = false;

        void add(double v) {
            sum -= window[idx];
            window[idx] = v;
            sum += v;
            idx = (idx + 1) % WINDOW;
            if (filled < WINDOW) filled++;
            count++;
            total += v;
            if (v > peak) peak = v;
        }

        double avg() {
            return filled == 0 ? 0 : sum / filled;
        }

        double lifetimeAvg() {
            return count == 0 ? 0 : total / count;
        }
    }
}
