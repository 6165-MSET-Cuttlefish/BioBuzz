package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Per-ball position spread, reported speed and track ids for balls that stand still. A ball is a place: each visible
 * track joins the ball whose mean position is nearest within the radius, so a new id at the same place is an id change.
 */
final class TrackStats {
    private final double radiusIn;
    private final List<Ball> balls = new ArrayList<>();
    private final List<double[]> pending = new ArrayList<>();
    private int frames;

    TrackStats(double radiusIn) {
        this.radiusIn = radiusIn;
    }

    void beginFrame() {
        pending.clear();
    }

    /** Coasting tracks (not visible this frame) are left out: they carry no new measurement. */
    void add(int id, double x, double y, double vx, double vy, boolean visible) {
        if (visible) pending.add(new double[] {id, x, y, vx, vy});
    }

    void endFrame() {
        frames++;
        Set<Ball> seen = new LinkedHashSet<>();
        for (double[] s : pending) {
            Ball nearest = null;
            double best = radiusIn;
            for (Ball b : balls) {
                double d = Math.hypot(s[1] - b.meanX(), s[2] - b.meanY());
                if (d <= best) {
                    best = d;
                    nearest = b;
                }
            }
            if (nearest == null) {
                nearest = new Ball();
                balls.add(nearest);
            }
            nearest.add(s);
            seen.add(nearest);
        }
        for (Ball b : seen) b.framesSeen++;
    }

    int frames() {
        return frames;
    }

    /** Sorted by mean x. */
    List<Map<String, Object>> balls() {
        List<Ball> sorted = new ArrayList<>(balls);
        Collections.sort(sorted, Comparator.comparingDouble(Ball::meanX));
        List<Map<String, Object>> out = new ArrayList<>();
        for (Ball b : sorted) out.add(b.summary(frames));
        return out;
    }

    private static final class Ball {
        final List<double[]> samples = new ArrayList<>();
        final Set<Integer> ids = new LinkedHashSet<>();
        int framesSeen;
        double sumX;
        double sumY;

        void add(double[] s) {
            samples.add(s);
            ids.add((int) s[0]);
            sumX += s[1];
            sumY += s[2];
        }

        double meanX() {
            return sumX / samples.size();
        }

        double meanY() {
            return sumY / samples.size();
        }

        Map<String, Object> summary(int frames) {
            double mx = meanX();
            double my = meanY();
            double squares = 0;
            double maxDeviation = 0;
            double maxSpread = 0;
            double[] speeds = new double[samples.size()];
            for (int i = 0; i < samples.size(); i++) {
                double[] s = samples.get(i);
                double d = Math.hypot(s[1] - mx, s[2] - my);
                squares += d * d;
                maxDeviation = Math.max(maxDeviation, d);
                speeds[i] = Math.hypot(s[3], s[4]);
                for (int j = 0; j < i; j++) {
                    double[] o = samples.get(j);
                    maxSpread = Math.max(maxSpread, Math.hypot(s[1] - o[1], s[2] - o[2]));
                }
            }
            Arrays.sort(speeds);
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("meanXIn", mx);
            m.put("meanYIn", my);
            m.put("samples", samples.size());
            m.put("framesSeen", framesSeen);
            m.put("framesMissing", frames - framesSeen);
            m.put("sdIn", Math.sqrt(squares / samples.size()));
            m.put("maxDeviationIn", maxDeviation);
            m.put("maxSpreadIn", maxSpread);
            m.put("speedP50Ips", rank(speeds, 50));
            m.put("speedP95Ips", rank(speeds, 95));
            m.put("speedMaxIps", speeds[speeds.length - 1]);
            m.put("ids", new ArrayList<>(ids));
            m.put("idChanges", ids.size() - 1);
            return m;
        }
    }

    private static double rank(double[] sorted, double p) {
        int index = (int) Math.ceil(p / 100.0 * sorted.length) - 1;
        return sorted[Math.max(0, Math.min(sorted.length - 1, index))];
    }
}
