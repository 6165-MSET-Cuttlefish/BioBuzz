package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Map;

import org.junit.Test;

public class PinpointMathTest {
    private static final double EPS = 1e-9;

    @Test
    public void ruleOfThreeBoundsAnHourAndFiveMinutesAsThePlanSays() {
        assertEquals(9.58e-6, PinpointMath.ruleOfThree(313_000), 1e-8);
        assertEquals(1.15e-4, PinpointMath.ruleOfThree(26_000), 1e-6);
        assertEquals(4950, PinpointMath.matchReads(), EPS);
        assertEquals(0.046, PinpointMath.perMatchBound(313_000), 0.001);
        assertTrue(PinpointMath.perMatchBound(26_000) > 0.4);
        assertEquals(1.0, PinpointMath.ruleOfThree(0), EPS);
        assertEquals(1.0, PinpointMath.ruleOfThree(2), EPS);
    }

    @Test
    public void turnFoldsIntoPlusMinus180() {
        assertEquals(2, PinpointMath.turnDeg(Math.toRadians(1), Math.toRadians(359)), 1e-9);
        assertEquals(-2, PinpointMath.turnDeg(Math.toRadians(359), Math.toRadians(1)), 1e-9);
        assertEquals(180, PinpointMath.turnDeg(0, Math.PI), 1e-9);
        assertEquals(180, PinpointMath.turnDeg(Math.PI, 0), 1e-9);
    }

    @Test
    public void tailPercentilesComeFromItsBinsAndItsCountsAreExact() {
        PinpointMath.Tail tail = new PinpointMath.Tail();
        for (int i = 0; i < 99; i++) tail.add(6.0);
        tail.add(41.8);
        assertEquals(100, tail.count());
        assertEquals(6.05, tail.percentile(50), 1e-9);
        assertEquals(6.05, tail.percentile(99), 1e-9);
        assertEquals(41.8, tail.percentile(100), 1e-9);
        assertEquals(41.8, tail.max(), EPS);
        assertEquals(1, tail.over20());
        assertEquals(1, tail.over40());
        tail.add(20.0);
        tail.add(40.0);
        assertEquals(2, tail.over20());
        assertEquals(1, tail.over40());
    }

    @Test
    public void tailBeyondItsRangeReportsTheMax() {
        PinpointMath.Tail tail = new PinpointMath.Tail();
        tail.add(5);
        tail.add(250);
        assertEquals(250, tail.percentile(100), EPS);
        assertEquals(5.05, tail.percentile(50), 1e-9);
        Map<String, Object> s = tail.summary();
        assertEquals(250.0, (Double) s.get("max"), EPS);
        assertEquals(2L, s.get("count"));
        assertTrue(new PinpointMath.Tail().summary().containsKey("count"));
        assertFalse(new PinpointMath.Tail().summary().containsKey("p50"));
    }

    @Test
    public void slowReadsSplitByBlockingGc() {
        PinpointMath.SlowReads slow = new PinpointMath.SlowReads(1);
        slow.add(1, 10, 19.9, 0, 0);
        slow.add(2, 20, 45, 0, 1);
        slow.add(3, 30, 45, 1, 1);
        slow.add(4, 40, 25, 1, 1);
        assertEquals(1, slow.over40WithoutGc());
        Map<String, Object> s = slow.summary(1000);
        assertEquals(3L, s.get("over20"));
        assertEquals(2L, s.get("over20WithBlockingGc"));
        assertEquals(2L, s.get("over40"));
        assertEquals(1L, s.get("over40WithBlockingGc"));
        assertEquals(0.003, (Double) s.get("over20Rate"), EPS);
        assertEquals(1, ((List<?>) s.get("rows")).size());
        assertEquals(2L, s.get("rowsDropped"));
    }

    @Test
    public void streaksCountOnlyAfterTheFirstReadyAndSayWhereTheGuardWouldStop() {
        PinpointMath.Streaks s = new PinpointMath.Streaks(10);
        s.add(0, "FAULT_BAD_READ", false, "init", "p0");
        s.add(71, "READY", true, "init", null);
        s.add(100, "READY", true, "init", null);
        s.add(5000, "NOT_READY", false, "running", "p1");
        s.add(5011, "NOT_READY", false, "running", "p2");
        s.add(5023, "READY", true, "running", null);
        s.add(6000, "FAULT_BAD_READ", false, "running", "p3");
        s.add(6012, "READY", true, "running", null);
        assertEquals(3, s.notReady());
        assertEquals(7, s.readsAfterReady());
        assertEquals(5000, s.guardStopMs(), EPS);
        Map<String, Object> m = s.summary(7000);
        assertEquals(2L, m.get("streakCount"));
        assertEquals(2L, m.get("longestStreakReads"));
        assertEquals(23.0, (Double) m.get("longestStreakMs"), EPS);
        assertEquals("running", m.get("guardWouldStopPhase"));
        List<?> streaks = (List<?>) m.get("streaks");
        Map<?, ?> first = (Map<?, ?>) streaks.get(0);
        assertEquals("NOT_READY", first.get("status"));
        assertEquals(2L, first.get("reads"));
        assertEquals(23.0, (Double) first.get("readyAgainAfterMs"), EPS);
        assertEquals("p1", first.get("pose"));
        assertEquals(2L, ((Map<?, ?>) m.get("statusCounts")).get("NOT_READY"));
        assertEquals(2L, ((Map<?, ?>) m.get("statusCounts")).get("FAULT_BAD_READ"));
    }

    @Test
    public void aStreakStillOpenAtTheEndCountsToTheEnd() {
        PinpointMath.Streaks s = new PinpointMath.Streaks(10);
        s.add(0, "READY", true, "init", null);
        s.add(1000, "NOT_READY", false, "running", "p");
        Map<String, Object> m = s.summary(4000);
        assertEquals(true, m.get("openAtEnd"));
        assertEquals(3000.0, (Double) m.get("longestStreakMs"), EPS);
    }

    @Test
    public void noReadyMeansNothingCounted() {
        PinpointMath.Streaks s = new PinpointMath.Streaks(10);
        s.add(0, "NOT_READY", false, "init", "p");
        s.add(10, "CALIBRATING", false, "init", "p");
        assertEquals(0, s.readsAfterReady());
        assertEquals(0, s.notReady());
        assertTrue(Double.isNaN(s.guardStopMs()));
    }

    @Test
    public void driftReadsTheStepTheWindowAndTheSlope() {
        PinpointMath.Drift d = new PinpointMath.Drift();
        for (int i = 0; i <= 3000; i++) {
            double t = i * 0.1;
            double deg = t < 5 ? 0.04 * t : 0.2 + 0.01 * (t - 5);
            d.add(t, Math.toRadians(deg));
        }
        assertEquals(0.2, d.firstStepDeg(), 1e-6);
        assertEquals(1.15, d.windowDeg(), 1e-6);
        assertEquals(0.6, d.slopeDegPerMin(), 1e-6);
        assertEquals(0.2 + 0.01 * 295, d.maxAbsDeg(), 1e-6);
    }

    @Test
    public void driftUnwrapsAcrossZero() {
        PinpointMath.Drift d = new PinpointMath.Drift();
        for (int i = 0; i <= 1300; i++) {
            double t = i * 0.1;
            double deg = 359.9 + 0.002 * t;
            d.add(t, Math.toRadians(deg % 360));
        }
        assertEquals(0.01, d.firstStepDeg(), 1e-6);
        assertEquals(0.23, d.windowDeg(), 1e-6);
        assertEquals(0.12, d.slopeDegPerMin(), 1e-6);
    }

    @Test
    public void driftWithTooFewSamplesIsNaN() {
        PinpointMath.Drift d = new PinpointMath.Drift();
        d.add(0, 0);
        d.add(1, 0);
        assertTrue(Double.isNaN(d.firstStepDeg()));
        assertTrue(Double.isNaN(d.windowDeg()));
        assertTrue(Double.isNaN(d.slopeDegPerMin()));
    }
}
