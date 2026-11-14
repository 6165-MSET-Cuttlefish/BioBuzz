package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

public class OctoQuadMathTest {
    private static final int[] DEFAULT_INTERVALS = {50, 50, 50, 50, 50, 50, 50, 50};

    @Test
    public void aLostMarkerShowsAsOneChangedChannel() {
        int[] set = DEFAULT_INTERVALS.clone();
        set[7] = 17;
        List<int[]> changes = OctoQuadMath.diff(set, DEFAULT_INTERVALS);
        assertEquals(1, changes.size());
        assertArrayEquals(new int[] {7, 17, 50}, changes.get(0));
        assertTrue(OctoQuadMath.kept(set, 7, 17));
        assertFalse(OctoQuadMath.kept(DEFAULT_INTERVALS, 7, 17));
    }

    @Test
    public void aKeptTableHasNoChanges() {
        assertTrue(OctoQuadMath.diff(DEFAULT_INTERVALS, DEFAULT_INTERVALS.clone()).isEmpty());
    }

    @Test
    public void everyChangedChannelIsListedInOrder() {
        int[] after = {25, 50, 50, 50, 50, 50, 50, 17};
        List<int[]> changes = OctoQuadMath.diff(DEFAULT_INTERVALS, after);
        assertEquals(2, changes.size());
        assertEquals(0, changes.get(0)[0]);
        assertEquals(7, changes.get(1)[0]);
    }

    @Test(expected = IllegalArgumentException.class)
    public void aShortTableIsRefused() {
        OctoQuadMath.diff(new int[7], DEFAULT_INTERVALS);
    }

    @Test
    public void directionBitsDiffLikeTheOtherTables() {
        boolean[] set = new boolean[8];
        set[0] = true;
        set[1] = true;
        int[] before = OctoQuadMath.directionBits(set);
        assertArrayEquals(new int[] {1, 1, 0, 0, 0, 0, 0, 0}, before);
        List<int[]> changes = OctoQuadMath.diff(before, OctoQuadMath.directionBits(new boolean[8]));
        assertEquals(2, changes.size());
        assertArrayEquals(new int[] {1, 1, 0}, changes.get(1));
    }

    @Test
    public void podCountsZeroedOrMovedOnlyOnTheListedChannels() {
        int[] before = {0, 0, 0, 0, 900, 0, 0, 0};
        int[] rolled = {2048, -3, 0, 0, 900, 0, 0, 0};
        int[] pods = {0, 1};
        assertTrue(OctoQuadMath.zeroed(before, pods, 0));
        assertFalse(OctoQuadMath.zeroed(rolled, pods, 0));
        assertTrue(OctoQuadMath.zeroed(new int[] {1, -1, 0, 0, 0, 0, 0, 0}, pods, 1));
        assertTrue(OctoQuadMath.moved(before, rolled, pods, 100));
        assertFalse(OctoQuadMath.moved(before, rolled, new int[] {1}, 100));
        assertFalse(OctoQuadMath.moved(new int[] {Integer.MAX_VALUE}, new int[] {Integer.MAX_VALUE}, new int[] {0}, 0));
        assertTrue(OctoQuadMath.moved(new int[] {Integer.MIN_VALUE}, new int[] {Integer.MAX_VALUE}, new int[] {0}, 0));
    }
}
