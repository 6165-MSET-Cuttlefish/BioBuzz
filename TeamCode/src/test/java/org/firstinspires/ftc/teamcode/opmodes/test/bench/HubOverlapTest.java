package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class HubOverlapTest {
    private static final List<String> CH_MOTORS = Arrays.asList("fl", "fr", "bl", "br");
    private static final List<String> CH_SERVOS = Arrays.asList("benchServo", "benchServo2");
    private static final List<String> EH_MOTORS = Arrays.asList("efl", "efr", "ebl", "ebr");
    private static final List<String> EH_SERVOS = Arrays.asList("eServo", "eServo2");

    private static Map<String, List<String>> sets() {
        return HubOverlap.sets(CH_MOTORS, CH_SERVOS, EH_MOTORS, EH_SERVOS);
    }

    private static boolean onControlHub(String channel) {
        return CH_MOTORS.contains(channel) || CH_SERVOS.contains(channel);
    }

    /** True when the set writes exactly as many channels on each hub as its "CH + EH" name says, none twice. */
    private static boolean matchesName(String name, List<String> writes) {
        String[] parts = name.split(" ");
        int ch = Integer.parseInt(parts[0]);
        int eh = Integer.parseInt(parts[2]);
        int onCh = 0;
        for (String w : writes) if (onControlHub(w)) onCh++;
        return onCh == ch && writes.size() - onCh == eh && new HashSet<>(writes).size() == writes.size();
    }

    private static boolean isPermutation(int[] order, int n) {
        if (order.length != n) return false;
        boolean[] seen = new boolean[n];
        for (int i : order) {
            if (i < 0 || i >= n || seen[i]) return false;
            seen[i] = true;
        }
        return true;
    }

    @Test
    public void theNameCheckRejectsAWrongSplitOrADuplicate() {
        assertFalse(matchesName("5 + 1", Arrays.asList("fl", "efl", "efr", "eServo", "ebl", "ebr")));
        assertFalse(matchesName("3 + 3", Arrays.asList("fl", "fl", "fr", "efl", "efr", "eServo")));
        assertFalse(matchesName("6 + 0", Arrays.asList("fl", "fr", "bl", "br", "benchServo")));
        assertTrue(matchesName("1 + 1", Arrays.asList("fl", "efl")));
    }

    @Test
    public void everySetWritesWhatItsNameSays() {
        Map<String, List<String>> sets = sets();
        assertEquals(Arrays.asList("6 + 0", "0 + 6", "6 + 6", "6 + 6 alternating", "3 + 3", "5 + 1", "1 + 5"),
                Arrays.asList(sets.keySet().toArray()));
        for (Map.Entry<String, List<String>> e : sets.entrySet()) {
            String name = e.getKey().replace(" alternating", "");
            assertTrue(e.getKey() + " writes " + e.getValue(), matchesName(name, e.getValue()));
        }
    }

    @Test
    public void twelveTogetherRunsHubByHubOrAlternating() {
        Map<String, List<String>> sets = sets();
        assertEquals(Arrays.asList("fl", "benchServo", "fr", "benchServo2", "bl", "br",
                "efl", "eServo", "efr", "eServo2", "ebl", "ebr"), sets.get("6 + 6"));
        List<String> alternating = sets.get("6 + 6 alternating");
        for (int i = 0; i < alternating.size(); i++) {
            assertEquals(alternating.get(i), i % 2 == 0, onControlHub(alternating.get(i)));
        }
        assertEquals(new HashSet<>(sets.get("6 + 6")), new HashSet<>(alternating));
        assertEquals(sets.get("6 + 0"), sets.get("6 + 6").subList(0, 6));
        assertEquals(sets.get("0 + 6"), sets.get("6 + 6").subList(6, 12));
    }

    @Test
    public void partialSetsMixMotorsAndServosAndMirrorEachOther() {
        Map<String, List<String>> sets = sets();
        assertEquals(Arrays.asList("fl", "benchServo", "fr", "efl", "eServo", "efr"), sets.get("3 + 3"));
        assertEquals(Arrays.asList("fl", "benchServo", "fr", "benchServo2", "bl", "efl"), sets.get("5 + 1"));
        assertEquals(Arrays.asList("fl", "efl", "eServo", "efr", "eServo2", "ebl"), sets.get("1 + 5"));
    }

    @Test
    public void mixedKeepsGoingWhenOneKindRunsOut() {
        assertEquals(Arrays.asList("m0", "s0", "m1", "m2"),
                HubOverlap.mixed(Arrays.asList("m0", "m1", "m2"), Collections.singletonList("s0"), 4));
        assertEquals(Arrays.asList("s0", "s1"),
                HubOverlap.mixed(Collections.<String>emptyList(), Arrays.asList("s0", "s1"), 5));
        assertEquals(Collections.singletonList("m0"),
                HubOverlap.mixed(Arrays.asList("m0", "m1"), Arrays.asList("s0", "s1"), 1));
    }

    @Test
    public void aHubWithoutSixBenchChannelsIsRefused() {
        try {
            HubOverlap.sets(CH_MOTORS, CH_SERVOS, EH_MOTORS, Collections.singletonList("eServo"));
            fail("five Expansion Hub channels must be refused");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("6 + 5"));
        }
    }

    @Test
    public void thePermutationCheckRejectsARepeatOrAGap() {
        assertFalse(isPermutation(new int[] {0, 1, 1, 3, 4, 5, 6}, 7));
        assertFalse(isPermutation(new int[] {0, 1, 2, 3, 4, 5}, 7));
        assertFalse(isPermutation(new int[] {0, 1, 2, 3, 4, 5, 7}, 7));
        assertTrue(isPermutation(new int[] {6, 5, 4, 3, 2, 1, 0}, 7));
    }

    @Test
    public void eachRoundRunsEverySetOnceAndNoSetKeepsOnePlace() {
        Random random = new Random(HubOverlap.SEED);
        int sets = 7;
        int[][] atPosition = new int[sets][sets];
        int[] previous = null;
        int unchanged = 0;
        for (int round = 0; round < HubOverlap.LOOPS; round++) {
            int[] order = HubOverlap.roundOrder(sets, random);
            assertTrue(Arrays.toString(order), isPermutation(order, sets));
            for (int pos = 0; pos < sets; pos++) atPosition[order[pos]][pos]++;
            if (previous != null && Arrays.equals(previous, order)) unchanged++;
            previous = order;
        }
        for (int set = 0; set < sets; set++) {
            for (int pos = 0; pos < sets; pos++) {
                assertTrue("set " + set + " ran at position " + pos + " in " + atPosition[set][pos] + " of "
                        + HubOverlap.LOOPS + " rounds", atPosition[set][pos] >= HubOverlap.LOOPS / sets / 2);
            }
        }
        assertTrue(unchanged < 3);
    }

    @Test
    public void theSameSeedGivesTheSameOrder() {
        Random a = new Random(HubOverlap.SEED);
        Random b = new Random(HubOverlap.SEED);
        for (int i = 0; i < 20; i++) assertArrayEquals(HubOverlap.roundOrder(7, a), HubOverlap.roundOrder(7, b));
    }
}
