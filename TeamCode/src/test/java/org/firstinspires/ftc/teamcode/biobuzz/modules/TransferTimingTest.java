package org.firstinspires.ftc.teamcode.biobuzz.modules;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TransferTimingTest {
    private static final double EPS = 1e-9;
    private static final double FAR = 117;
    private static final double BALL = 40;
    private static final double THRESHOLD = 80;
    private static final double HYSTERESIS = 10;
    private static final long MS = 1_000_000L;

    private final TransferTiming timing = new TransferTiming();
    private int counts;

    private void sample(long ms, double signal) {
        if (timing.update(ms * MS, signal, true, THRESHOLD, HYSTERESIS)) counts++;
    }

    private void oneSensorBall(long frontMs, long backMs) {
        for (long t = frontMs; t < backMs; t += 10) sample(t, BALL);
        sample(backMs, FAR);
    }

    @Test
    public void edgeCountsOnceEachTimeTheSignalRisesPastTheThresholdAfterArming() {
        TransferCounter.Edge edge = new TransferCounter.Edge();
        assertFalse(edge.update(100, 50, 10));
        assertFalse(edge.armed());
        assertFalse(edge.update(41, 50, 10));
        assertFalse(edge.armed());
        assertFalse(edge.update(40, 50, 10));
        assertTrue(edge.armed());
        assertFalse(edge.update(50, 50, 10));
        assertTrue(edge.update(51, 50, 10));
        assertFalse(edge.armed());
        assertFalse(edge.update(100, 50, 10));
        assertFalse(edge.update(41, 50, 10));
        assertFalse(edge.update(60, 50, 10));
        assertFalse(edge.update(40, 50, 10));
        assertTrue(edge.update(60, 50, 10));
    }

    @Test
    public void oneSensorTimesTheBallInFrontOfTheBeamTheGapAndTheBackEdgeToBackEdgePeriod() {
        sample(0, FAR);
        sample(10, FAR);
        oneSensorBall(100, 120);
        sample(130, FAR);
        sample(140, FAR);
        oneSensorBall(150, 170);
        sample(180, FAR);
        oneSensorBall(190, 230);
        sample(240, FAR);
        sample(250, FAR);
        sample(260, FAR);
        oneSensorBall(270, 290);

        TransferTiming.Snapshot s = timing.snapshot();
        assertEquals(4, counts);
        assertEquals(3, s.periods);
        assertArrayEquals(new double[] {50, 60, 60}, s.recentPeriodsMs(), EPS);
        assertEquals(60, s.lastPeriodMs, EPS);
        assertEquals(50, s.shortestPeriodMs, EPS);
        assertEquals(170.0 / 3, s.meanPeriodMs, EPS);
        assertEquals(20, s.lastArmedMs, EPS);
        assertEquals(20, s.shortestArmedMs, EPS);
        assertEquals(40, s.lastRearmMs, EPS);
        assertEquals(20, s.shortestRearmMs, EPS);
    }

    @Test
    public void armingStartsOnlyBelowThresholdMinusHysteresis() {
        sample(0, FAR);
        sample(10, 75);
        sample(20, 72);
        sample(30, BALL);
        sample(40, 75);
        sample(50, FAR);
        assertEquals(1, counts);
        assertEquals(20, timing.snapshot().lastArmedMs, EPS);
    }

    @Test
    public void aSignalAlreadyArmedAtTheFirstSampleIsNotTimedAsArmed() {
        sample(0, 0);
        sample(10, 0);
        sample(20, FAR);
        TransferTiming.Snapshot s = timing.snapshot();
        assertEquals(1, counts);
        assertTrue(Double.isNaN(s.lastArmedMs));
        assertTrue(Double.isNaN(s.shortestArmedMs));
        assertTrue(Double.isNaN(s.lastPeriodMs));
        assertTrue(Double.isNaN(s.meanPeriodMs));
        assertEquals(0, s.periods);
        sample(30, 0);
        assertEquals(10, timing.snapshot().lastRearmMs, EPS);
    }

    @Test
    public void twoSensorsTimeTheBackEdgeFromAToBWhenTheNextBallIsFartherBack() {
        double threshold = 25;
        double hysteresis = 10;
        double[][] aB = {
                {FAR, FAR}, {BALL, FAR}, {BALL, BALL}, {FAR, BALL}, {FAR, BALL}, {FAR, FAR}, {FAR, FAR},
                {BALL, FAR}, {BALL, BALL}, {FAR, BALL}, {FAR, BALL}, {FAR, BALL}, {FAR, FAR},
        };
        int twoSensorCounts = 0;
        for (int i = 0; i < aB.length; i++) {
            if (timing.update(i * 10 * MS, aB[i][0] - aB[i][1], false, threshold, hysteresis)) twoSensorCounts++;
        }
        TransferTiming.Snapshot s = timing.snapshot();
        assertEquals(2, twoSensorCounts);
        assertEquals(1, s.periods);
        assertEquals(60, s.lastPeriodMs, EPS);
        assertEquals(30, s.lastRearmMs, EPS);
        assertEquals(20, s.shortestRearmMs, EPS);
        assertEquals(40, s.lastArmedMs, EPS);
    }

    @Test
    public void twoSensorsReArmWhenTheNextBallReachesABeforeTheFirstClearsB() {
        double[][] aB = {{FAR, FAR}, {BALL, FAR}, {BALL, BALL}, {FAR, BALL}, {BALL, BALL}, {FAR, BALL}};
        int twoSensorCounts = 0;
        for (int i = 0; i < aB.length; i++) {
            if (timing.update(i * 10 * MS, aB[i][0] - aB[i][1], false, 25, 10)) twoSensorCounts++;
        }
        TransferTiming.Snapshot s = timing.snapshot();
        assertEquals(2, twoSensorCounts);
        assertEquals(10, s.lastRearmMs, EPS);
        assertEquals(20, s.lastPeriodMs, EPS);
    }

    @Test
    public void resetClearsEveryTimingAndForgetsTheSpellInProgress() {
        sample(0, FAR);
        oneSensorBall(10, 30);
        oneSensorBall(40, 60);
        sample(70, BALL);
        timing.reset();
        TransferTiming.Snapshot cleared = timing.snapshot();
        assertEquals(0, cleared.periods);
        assertEquals(0, cleared.recentPeriodsMs().length);
        assertTrue(Double.isNaN(cleared.lastPeriodMs));
        assertTrue(Double.isNaN(cleared.shortestPeriodMs));
        assertTrue(Double.isNaN(cleared.lastArmedMs));
        assertTrue(Double.isNaN(cleared.lastRearmMs));
        assertTrue(Double.isNaN(cleared.shortestRearmMs));

        sample(80, FAR);
        TransferTiming.Snapshot afterFirst = timing.snapshot();
        assertEquals(3, counts);
        assertEquals(0, afterFirst.periods);
        assertTrue(Double.isNaN(afterFirst.lastArmedMs));

        oneSensorBall(100, 130);
        TransferTiming.Snapshot afterSecond = timing.snapshot();
        assertEquals(1, afterSecond.periods);
        assertEquals(50, afterSecond.lastPeriodMs, EPS);
        assertEquals(30, afterSecond.lastArmedMs, EPS);
        assertEquals(20, afterSecond.lastRearmMs, EPS);
    }

    @Test
    public void keepsTheLastEightPeriodsOldestFirst() {
        sample(0, FAR);
        long t = 10;
        for (int ball = 0; ball < 11; ball++) {
            oneSensorBall(t, t + 20);
            t += 30 + ball * 10;
        }
        TransferTiming.Snapshot s = timing.snapshot();
        assertEquals(10, s.periods);
        assertArrayEquals(new double[] {50, 60, 70, 80, 90, 100, 110, 120}, s.recentPeriodsMs(), EPS);
        assertEquals(30, s.shortestPeriodMs, EPS);
        assertEquals(75, s.meanPeriodMs, EPS);
    }

    @Test
    public void aSnapshotDoesNotChangeAfterLaterSamples() {
        sample(0, FAR);
        oneSensorBall(10, 30);
        oneSensorBall(40, 60);
        TransferTiming.Snapshot before = timing.snapshot();
        double[] recent = before.recentPeriodsMs();
        recent[0] = -1;
        oneSensorBall(100, 130);
        assertEquals(1, before.periods);
        assertArrayEquals(new double[] {30}, before.recentPeriodsMs(), EPS);
        assertEquals(2, timing.snapshot().periods);
    }

    @Test
    public void aResetBetweenACountAndTheReArmStillTimesTheNextBall() {
        sample(0, FAR);
        oneSensorBall(10, 30);
        oneSensorBall(40, 60);
        timing.reset();
        oneSensorBall(70, 90);
        TransferTiming.Snapshot s = timing.snapshot();
        assertEquals(3, counts);
        assertEquals(20, s.lastArmedMs, EPS);
        assertEquals(0, s.periods);
        assertTrue(Double.isNaN(s.lastRearmMs));
    }

    @Test
    public void aLongIdleCountsInThePeriodAndItsMean() {
        sample(0, FAR);
        oneSensorBall(10, 30);
        oneSensorBall(40, 60);
        for (long t = 70; t < 1000; t += 10) sample(t, FAR);
        oneSensorBall(1000, 1020);
        TransferTiming.Snapshot s = timing.snapshot();
        assertArrayEquals(new double[] {30, 960}, s.recentPeriodsMs(), EPS);
        assertEquals(495, s.meanPeriodMs, EPS);
        assertEquals(940, s.lastRearmMs, EPS);
    }

    @Test
    public void withTwoSensorsTheArmedTimeIncludesTheWaitForTheNextBall() {
        double[][] aB = {
                {FAR, FAR}, {BALL, FAR}, {BALL, BALL}, {FAR, BALL}, {FAR, FAR}, {FAR, FAR}, {FAR, FAR}, {FAR, FAR},
                {FAR, FAR}, {BALL, FAR}, {BALL, BALL}, {FAR, BALL}, {FAR, FAR},
        };
        for (int i = 0; i < aB.length; i++) timing.update(i * 10 * MS, aB[i][0] - aB[i][1], false, 25, 10);
        TransferTiming.Snapshot s = timing.snapshot();
        assertEquals(80, s.lastPeriodMs, EPS);
        assertEquals(10, s.lastRearmMs, EPS);
        assertEquals(70, s.lastArmedMs, EPS);
    }

    @Test
    public void switchingToOneSensorClearsTheTimingsAndTheSwitchSampleAnchorsNoPeriod() {
        double[][] aB = {{FAR, FAR}, {BALL, FAR}, {BALL, BALL}, {FAR, BALL}, {FAR, FAR}, {FAR, FAR}};
        for (int i = 0; i < aB.length; i++) timing.update(i * 10 * MS, aB[i][0] - aB[i][1], false, 25, 10);
        assertEquals(0, timing.snapshot().periods);
        assertTrue(timing.update(60 * MS, FAR, true, THRESHOLD, HYSTERESIS));
        TransferTiming.Snapshot switched = timing.snapshot();
        assertTrue(switched.upstreamOnly);
        assertTrue(Double.isNaN(switched.lastRearmMs));
        assertTrue(Double.isNaN(switched.lastArmedMs));

        sample(70, FAR);
        oneSensorBall(5000, 5020);
        TransferTiming.Snapshot first = timing.snapshot();
        assertEquals(0, first.periods);
        assertEquals(20, first.lastArmedMs, EPS);
        assertTrue(Double.isNaN(first.lastRearmMs));
        oneSensorBall(5030, 5050);
        assertEquals(30, timing.snapshot().lastPeriodMs, EPS);
    }

    @Test
    public void switchingToTwoSensorsDoesNotTimeTheArmingItCauses() {
        sample(0, FAR);
        oneSensorBall(10, 30);
        timing.update(40 * MS, 0, false, 25, 10);
        TransferTiming.Snapshot s = timing.snapshot();
        assertFalse(s.upstreamOnly);
        assertTrue(Double.isNaN(s.lastRearmMs));
        assertTrue(Double.isNaN(s.lastArmedMs));
        timing.update(50 * MS, FAR - BALL, false, 25, 10);
        assertTrue(Double.isNaN(timing.snapshot().lastArmedMs));
    }

    @Test
    public void aThresholdChangeAloneClearsTheTimings() {
        sample(0, FAR);
        oneSensorBall(10, 30);
        oneSensorBall(40, 60);
        assertEquals(1, timing.snapshot().periods);
        timing.update(70 * MS, FAR, true, THRESHOLD + 5, HYSTERESIS);
        assertEquals(0, timing.snapshot().periods);
        assertTrue(Double.isNaN(timing.snapshot().lastArmedMs));
    }

    @Test
    public void aDipInsideTheHysteresisBandAfterACountIsNotANewBallButOneBelowItIs() {
        sample(0, FAR);
        sample(10, 65);
        sample(20, 85);
        sample(30, 75);
        sample(40, 85);
        assertEquals(1, counts);
        sample(50, 68);
        sample(60, 85);
        assertEquals(2, counts);
    }
}
