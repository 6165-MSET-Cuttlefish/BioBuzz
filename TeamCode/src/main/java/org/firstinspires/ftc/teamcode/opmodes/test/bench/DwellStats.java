package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Level steps in an ADC trace of a target moving through a rangefinder's beam. A new level starts when a new reading is
 * more than the threshold from the mean of the current level's readings, so ADC noise between the sensor's own updates
 * doesn't split a level.
 * Dwell is the time between two level steps of one pass (steps no further apart than maxGapMs).
 */
final class DwellStats {
    final int steps;
    final int timedSteps;
    final int fastSteps;
    final double seconds;
    final double[] dwellMs;

    private DwellStats(int steps, int timedSteps, int fastSteps, double seconds, double[] dwellMs) {
        this.steps = steps;
        this.timedSteps = timedSteps;
        this.fastSteps = fastSteps;
        this.seconds = seconds;
        this.dwellMs = dwellMs;
    }

    /**
     * A timed step is fast when its change, scaled to one expectedMs update, still exceeds the threshold: the target
     * moved fast enough that a sensor updating every expectedMs would step past the noise at every update.
     */
    static DwellStats of(double[] tMs, int[] mv, int count, double thresholdMv, double expectedMs, double maxGapMs) {
        if (count == 0) return new DwellStats(0, 0, 0, 0, new double[0]);
        double[] dwells = new double[count];
        int timed = 0;
        int steps = 0;
        int fast = 0;
        double levelSum = mv[0];
        int levelCount = 1;
        double lastStep = Double.NaN;
        for (int i = 1; i < count; i++) {
            if (mv[i] == mv[i - 1]) continue;
            double change = Math.abs(mv[i] - levelSum / levelCount);
            if (change <= thresholdMv) {
                levelSum += mv[i];
                levelCount++;
                continue;
            }
            steps++;
            double since = tMs[i] - lastStep;
            if (since > 0 && since <= maxGapMs) {
                dwells[timed++] = since;
                if (change * expectedMs / since > thresholdMv) fast++;
            }
            lastStep = tMs[i];
            levelSum = mv[i];
            levelCount = 1;
        }
        return new DwellStats(steps, timed, fast, (tMs[count - 1] - tMs[0]) / 1000.0, Arrays.copyOf(dwells, timed));
    }

    double fastFraction() {
        return timedSteps == 0 ? 0 : (double) fastSteps / timedSteps;
    }

    double dwellPercentile(double p) {
        if (dwellMs.length == 0) return Double.NaN;
        double[] sorted = Arrays.copyOf(dwellMs, dwellMs.length);
        Arrays.sort(sorted);
        int index = (int) Math.ceil(p / 100.0 * sorted.length) - 1;
        return sorted[Math.max(0, Math.min(sorted.length - 1, index))];
    }

    double stepsPerSecond() {
        return seconds <= 0 ? 0 : steps / seconds;
    }

    Map<String, Object> toMap() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("levelSteps", steps);
        m.put("timedSteps", timedSteps);
        m.put("fastSteps", fastSteps);
        m.put("fastFraction", fastFraction());
        m.put("dwellP10Ms", nullIfNaN(dwellPercentile(10)));
        m.put("dwellP50Ms", nullIfNaN(dwellPercentile(50)));
        m.put("dwellP90Ms", nullIfNaN(dwellPercentile(90)));
        m.put("stepsPerSecond", stepsPerSecond());
        m.put("seconds", seconds);
        return m;
    }

    private static Double nullIfNaN(double v) {
        return Double.isNaN(v) ? null : v;
    }
}
