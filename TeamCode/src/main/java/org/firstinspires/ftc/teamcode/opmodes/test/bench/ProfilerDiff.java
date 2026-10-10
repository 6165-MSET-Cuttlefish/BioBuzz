package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import org.firstinspires.ftc.teamcode.architecture.telemetry.LoopProfiler;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Per-section loop-profile averages over one phase. LoopProfiler only keeps lifetime totals, and resetting it
 * mid-loop corrupts the next mark, so this differences two {@link LoopProfiler#report} snapshots instead.
 */
final class ProfilerDiff {
    private static final class Section {
        double avg;
        double peak;
        long count;
        boolean nested;
    }

    private final Map<String, Section> start;

    ProfilerDiff(LoopProfiler profiler) {
        start = parse(profiler.report(0, 0, 0));
    }

    /** Sections sorted by mean ms per loop they ran in; "nested" ones are inside a stage, not extra time. */
    List<Map<String, Object>> since(LoopProfiler profiler) {
        Map<String, Section> end = parse(profiler.report(0, 0, 0));
        List<Map<String, Object>> out = new ArrayList<>();
        for (Map.Entry<String, Section> e : end.entrySet()) {
            Section b = start.get(e.getKey());
            Section a = e.getValue();
            long count = a.count - (b == null ? 0 : b.count);
            if (count <= 0) continue;
            double total = a.avg * a.count - (b == null ? 0 : b.avg * b.count);
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("section", e.getKey());
            m.put("meanMs", total / count);
            m.put("count", count);
            m.put("nested", a.nested);
            m.put("lifetimePeakMs", a.peak);
            out.add(m);
        }
        Collections.sort(out, (x, y) -> Double.compare((Double) y.get("meanMs"), (Double) x.get("meanMs")));
        return out;
    }

    private static Map<String, Section> parse(String report) {
        Map<String, Section> sections = new HashMap<>();
        int bar = report.indexOf("||");
        if (bar < 0) throw new IllegalStateException("LoopProfiler.report() format changed: " + report);
        for (String part : report.substring(bar + 2).split(";")) {
            String entry = part.trim();
            if (entry.isEmpty()) continue;
            int eq = entry.lastIndexOf('=');
            String[] stats = entry.substring(eq + 1).split("/");
            if (eq < 0 || stats.length != 3) throw new IllegalStateException("LoopProfiler.report() entry format changed: " + entry);
            Section s = new Section();
            s.nested = entry.charAt(eq - 1) == '*';
            String name = entry.substring(0, s.nested ? eq - 1 : eq);
            s.avg = Double.parseDouble(stats[0]);
            s.peak = Double.parseDouble(stats[1]);
            s.count = Long.parseLong(stats[2]);
            sections.put(name, s);
        }
        return sections;
    }
}
