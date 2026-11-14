package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

public class DwellStatsTest {
    private static final double READ_MS = 1.3;
    private static final double ADC_MS = 10.0;
    private static final double NOISE_SD_MV = 3.0;
    private static final double THRESHOLD_MV = 3 * NOISE_SD_MV;

    /** Back-to-back reads of a hub ADC that refreshes every 10 ms from a sensor stepping every updateMs. */
    private static final class Trace {
        final double[] t;
        final int[] mv;
        final int count;

        Trace(double seconds, double updateMs, double mvPerUpdate, double passMs, double pauseMs, long seed) {
            Random random = new Random(seed);
            int n = (int) (seconds * 1000 / READ_MS);
            t = new double[n];
            mv = new int[n];
            int refresh = -1;
            int held = 0;
            for (int i = 0; i < n; i++) {
                double now = i * READ_MS;
                int r = (int) Math.floor((now + 3.7) / ADC_MS);
                if (r != refresh) {
                    refresh = r;
                    held = (int) Math.round(sensor(r * ADC_MS - 3.7, updateMs, mvPerUpdate, passMs, pauseMs)
                            + random.nextGaussian() * NOISE_SD_MV);
                }
                t[i] = now;
                mv[i] = held;
            }
            count = n;
        }

        private static double sensor(double ms, double updateMs, double mvPerUpdate, double passMs, double pauseMs) {
            double cycle = passMs + pauseMs;
            double inCycle = ((ms % cycle) + cycle) % cycle;
            double moving = Math.min(inCycle, passMs);
            return 400 + Math.floor(moving / updateMs) * mvPerUpdate;
        }

        DwellStats stats() {
            return DwellStats.of(t, mv, count, THRESHOLD_MV, 10, 60);
        }
    }

    @Test
    public void tenMillisecondStaircaseDwellsAboutTenMilliseconds() {
        DwellStats s = new Trace(8, 10, 40, 500, 300, 1).stats();
        assertTrue("steps " + s.timedSteps, s.timedSteps >= 200);
        assertTrue("fast " + s.fastFraction(), s.fastFraction() >= 0.8);
        double p50 = s.dwellPercentile(50);
        assertTrue("p50 " + p50, p50 >= 9 && p50 <= 12);
    }

    @Test
    public void twentyMillisecondStaircaseDwellsAboutTwentyMilliseconds() {
        DwellStats s = new Trace(12, 20, 80, 500, 300, 2).stats();
        assertTrue("steps " + s.timedSteps, s.timedSteps >= 200);
        assertTrue("fast " + s.fastFraction(), s.fastFraction() >= 0.8);
        double p50 = s.dwellPercentile(50);
        assertTrue("p50 " + p50, p50 >= 18 && p50 <= 22);
    }

    @Test
    public void slowSweepIsNotEnoughMotion() {
        DwellStats s = new Trace(12, 10, 5, 500, 300, 3).stats();
        assertTrue("fast " + s.fastFraction(), s.fastFraction() < 0.8);
        assertTrue("p50 " + s.dwellPercentile(50), s.dwellPercentile(50) > 12);
    }

    @Test
    public void stillTargetHasAlmostNoLevelSteps() {
        DwellStats s = new Trace(5, 10, 0, 500, 300, 4).stats();
        int refreshes = (int) (5000 / ADC_MS);
        assertTrue("timed steps " + s.timedSteps, s.timedSteps <= refreshes / 100);
        assertTrue("steps " + s.steps, s.steps <= refreshes / 50);
    }

    @Test
    public void pausesBetweenPassesAreNotDwells() {
        DwellStats s = new Trace(8, 10, 40, 200, 400, 5).stats();
        double max = 0;
        for (double d : s.dwellMs) max = Math.max(max, d);
        assertTrue("longest dwell " + max, max <= 60);
        assertTrue(s.steps > s.timedSteps);
    }

    @Test
    public void emptyTraceIsEmpty() {
        DwellStats s = DwellStats.of(new double[0], new int[0], 0, THRESHOLD_MV, 10, 60);
        assertEquals(0, s.steps);
        assertEquals(0, s.stepsPerSecond(), 0);
        assertEquals(0, s.fastFraction(), 0);
        assertTrue(Double.isNaN(s.dwellPercentile(50)));
    }
}
