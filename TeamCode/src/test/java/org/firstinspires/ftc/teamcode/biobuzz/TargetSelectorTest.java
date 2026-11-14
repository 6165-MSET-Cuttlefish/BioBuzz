package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField.Cell;
import org.junit.Test;

public class TargetSelectorTest {

    private static final double W = 141.5;

    @Test
    public void followsTheRobotsHalfForBothAlliances() {
        for (boolean blue : new boolean[] {false, true}) {
            TargetSelector s = new TargetSelector();
            s.followHalf(20, W, blue, 3);
            Cell audience = s.targetCell();
            s.followHalf(120, W, blue, 3);
            Cell rear = s.targetCell();
            assertEquals(audience.other(), rear);
            assertTrue(BioBuzzField.cellOpening(audience, blue, W).n[1] < 0);
            assertTrue(BioBuzzField.cellOpening(rear, blue, W).n[1] > 0);
            assertTrue(BioBuzzField.cellOpening(audience, blue, W).bottomCentreYIn < W / 2);
        }
    }

    @Test
    public void holdsItsChoiceNearTheMidline() {
        TargetSelector s = new TargetSelector();
        s.followHalf(20, W, false, 3);
        Cell audience = s.targetCell();
        s.followHalf(W / 2 + 2, W, false, 3);
        assertEquals(audience, s.targetCell());
        s.followHalf(W / 2 + 4, W, false, 3);
        assertEquals(audience.other(), s.targetCell());
        s.followHalf(W / 2 - 2, W, false, 3);
        assertEquals(audience.other(), s.targetCell());
    }

    @Test
    public void tipVerdictFlipsOnEachRisingEdge() {
        TargetSelector s = new TargetSelector();
        s.onTipVerdict(true);
        s.onTipVerdict(true);
        assertEquals(Cell.STARTS_DOWN, s.targetCell());
        s.onTipVerdict(false);
        s.onTipVerdict(true);
        assertEquals(Cell.STARTS_UP, s.targetCell());
    }
}
