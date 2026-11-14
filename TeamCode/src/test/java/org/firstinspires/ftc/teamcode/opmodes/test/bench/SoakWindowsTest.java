package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.util.List;
import java.util.Map;

public class SoakWindowsTest {
    private static final double EPS = 1e-9;

    @Test
    public void betweenTakesTheHalfOpenRange() {
        SoakWindows w = new SoakWindows();
        for (int i = 0; i < 100; i++) w.add(i * 10, i < 50 ? 20 : 40);
        Map<String, Object> before = w.between(0, 500);
        Map<String, Object> after = w.between(500, Double.POSITIVE_INFINITY);
        assertEquals(50, before.get("count"));
        assertEquals(50, after.get("count"));
        assertEquals(20.0, (double) before.get("p95"), EPS);
        assertEquals(40.0, (double) after.get("p50"), EPS);
    }

    @Test
    public void p95PicksTheNinetyFifthOfAHundred() {
        SoakWindows w = new SoakWindows();
        for (int i = 1; i <= 100; i++) w.add(i, i);
        Map<String, Object> all = w.between(0, 1000);
        assertEquals(95.0, (double) all.get("p95"), EPS);
        assertEquals(50.0, (double) all.get("p50"), EPS);
        assertEquals(100.0, (double) all.get("max"), EPS);
        assertEquals(50.5, (double) all.get("mean"), EPS);
    }

    @Test
    public void windowsKeepEmptyOnes() {
        SoakWindows w = new SoakWindows();
        w.add(100, 1);
        w.add(2500, 3);
        List<Map<String, Object>> windows = w.windows(1000);
        assertEquals(3, windows.size());
        assertEquals(1, windows.get(0).get("count"));
        assertEquals(0, windows.get(1).get("count"));
        assertEquals(1, windows.get(2).get("count"));
        assertEquals(2000.0, (double) windows.get(2).get("startMs"), EPS);
    }

    @Test
    public void emptyHasNoWindowsAndZeroCount() {
        SoakWindows w = new SoakWindows();
        assertEquals(0, w.windows(1000).size());
        assertEquals(0, w.between(0, 1).get("count"));
    }

    @Test
    public void growsPastItsFirstArray() {
        SoakWindows w = new SoakWindows();
        for (int i = 0; i < 5000; i++) w.add(i, 1);
        assertEquals(5000, w.count());
        assertEquals(5000, w.between(0, 5000).get("count"));
    }
}
