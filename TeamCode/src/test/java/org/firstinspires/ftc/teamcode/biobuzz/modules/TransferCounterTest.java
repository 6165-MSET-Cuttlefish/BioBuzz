package org.firstinspires.ftc.teamcode.biobuzz.modules;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TransferCounterTest {
    private static final long MS = 1_000_000L;
    private static final double FAR = 117;
    private static final double BALL = 40;
    private static final double THRESHOLD = 80;
    private static final double HYSTERESIS = 10;

    private final TransferCounter.Tally tally = new TransferCounter.Tally();
    private final TransferCounter.Sampler sampler = new TransferCounter.Sampler(tally);

    private boolean sample(long ms, double mm) {
        return sampler.sample(tally.read(), ms * MS, mm, true, THRESHOLD, HYSTERESIS);
    }

    private void ball(long frontMs, long backMs) {
        for (long t = frontMs; t < backMs; t += 10) sample(t, BALL);
        sample(backMs, FAR);
    }

    @Test
    public void aResetZeroesTheCountAndStartsANewEpoch() {
        long read = tally.read();
        assertTrue(tally.increment(read));
        assertEquals(1, tally.count());
        tally.reset();
        assertEquals(0, tally.count());
        assertEquals(1, tally.epoch());
        assertTrue(tally.increment(tally.read()));
        assertEquals(1, tally.count());
        assertEquals(1, tally.epoch());
    }

    @Test
    public void aBallCountedFromAReadTakenBeforeAResetIsLeftOutOfTheCountAndTheTiming() {
        sample(0, FAR);
        ball(10, 30);
        ball(40, 60);
        sample(70, BALL);
        long readBeforeReset = tally.read();
        tally.reset();
        assertFalse(sampler.sample(readBeforeReset, 80 * MS, FAR, true, THRESHOLD, HYSTERESIS));
        assertEquals(0, tally.count());

        ball(100, 120);
        ball(130, 150);
        assertEquals(2, tally.count());
        TransferTiming.Snapshot s = sampler.snapshot();
        assertEquals(1, s.periods);
        assertEquals(30, s.lastPeriodMs, 1e-9);
        assertEquals(tally.epoch(), sampler.timingEpoch());
    }

    @Test
    public void theTimingKeepsTheOldEpochUntilTheFirstSampleAfterAReset() {
        sample(0, FAR);
        ball(10, 30);
        ball(40, 60);
        tally.reset();
        assertEquals(0, sampler.timingEpoch());
        assertEquals(1, sampler.snapshot().periods);
        sample(70, FAR);
        assertEquals(1, sampler.timingEpoch());
        assertEquals(0, sampler.snapshot().periods);
    }
}
