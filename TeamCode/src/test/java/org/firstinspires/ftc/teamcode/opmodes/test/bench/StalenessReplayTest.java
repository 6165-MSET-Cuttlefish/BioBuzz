package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class StalenessReplayTest {
    private static final double[] THRESHOLDS = {0.4, 0.5, 0.6, 0.7};

    /** Arrivals 233 ms apart for the given seconds, with the listed extra gaps (s) after every tenth frame. */
    private static long[] stream(double seconds, double... gapsS) {
        List<Long> out = new ArrayList<>();
        long t = 1_000_000_000L;
        int gap = 0;
        for (int i = 0; t < 1_000_000_000L + seconds * 1e9; i++) {
            out.add(t);
            boolean extra = i % 10 == 9 && gap < gapsS.length;
            t += extra ? Math.round(gapsS[gap++] * 1e9) : 233_000_000L;
        }
        long[] a = new long[out.size()];
        for (int i = 0; i < a.length; i++) a[i] = out.get(i);
        return a;
    }

    @Test
    public void aSteadyStreamNeverGoesStale() {
        long[] a = stream(60);
        for (double t : THRESHOLDS) assertEquals(0, StalenessReplay.episodes(a, t));
        assertEquals(0.4, StalenessReplay.smallestClean(a, THRESHOLDS), 0);
        assertEquals(233, StalenessReplay.longestGapMs(a), 1e-9);
    }

    @Test
    public void oneGapOf550MsIsAnEpisodeAtHalfASecondButNotAt600Ms() {
        long[] a = stream(60, 0.55);
        assertEquals(1, StalenessReplay.episodes(a, 0.4));
        assertEquals(1, StalenessReplay.episodes(a, 0.5));
        assertEquals(0, StalenessReplay.episodes(a, 0.6));
        assertEquals(0, StalenessReplay.episodes(a, 0.7));
        assertEquals(0.6, StalenessReplay.smallestClean(a, THRESHOLDS), 0);
        assertEquals(550, StalenessReplay.longestGapMs(a), 1e-9);
    }

    @Test
    public void aGapExactlyAtTheThresholdIsNotStale() {
        long[] a = stream(10, 0.5);
        assertEquals(0, StalenessReplay.episodes(a, 0.5));
        assertEquals(1, StalenessReplay.episodes(a, 0.4));
    }

    @Test
    public void everyLongGapCounts() {
        long[] a = stream(60, 0.55, 0.65, 0.55);
        assertEquals(3, StalenessReplay.episodes(a, 0.5));
        assertEquals(1, StalenessReplay.episodes(a, 0.6));
        assertEquals(0.7, StalenessReplay.smallestClean(a, THRESHOLDS), 0);
    }

    @Test
    public void noCleanThresholdIsNaN() {
        assertTrue(Double.isNaN(StalenessReplay.smallestClean(stream(10, 0.8), THRESHOLDS)));
    }

    @Test
    public void fewerThanTwoArrivalsHaveNoGap() {
        assertEquals(0, StalenessReplay.episodes(new long[0], 0.5));
        assertEquals(0, StalenessReplay.episodes(new long[] {5}, 0.5));
        assertEquals(0, StalenessReplay.longestGapMs(new long[] {5}), 0);
    }
}
