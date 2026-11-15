package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TransferThreadCountTestTest {
    @Test
    public void aRangeThatWrapsThroughZeroHoldsBothEndsAndNothingBetween() {
        assertTrue(TransferThreadCountTest.hueInRange(350, 340, 20));
        assertTrue(TransferThreadCountTest.hueInRange(0, 340, 20));
        assertTrue(TransferThreadCountTest.hueInRange(360, 340, 20));
        assertTrue(TransferThreadCountTest.hueInRange(20, 340, 20));
        assertFalse(TransferThreadCountTest.hueInRange(21, 340, 20));
        assertFalse(TransferThreadCountTest.hueInRange(180, 340, 20));
        assertFalse(TransferThreadCountTest.hueInRange(339, 340, 20));
    }

    @Test
    public void anOrdinaryRangeHoldsOnlyItsInsideWithBothEnds() {
        assertTrue(TransferThreadCountTest.hueInRange(180, 180, 250));
        assertTrue(TransferThreadCountTest.hueInRange(250, 180, 250));
        assertFalse(TransferThreadCountTest.hueInRange(10, 180, 250));
        assertFalse(TransferThreadCountTest.hueInRange(300, 180, 250));
    }

    @Test
    public void equalBoundsHoldOnlyThatHueAndFullScaleStaysOutOfARangeFromZero() {
        assertTrue(TransferThreadCountTest.hueInRange(70, 70, 70));
        assertFalse(TransferThreadCountTest.hueInRange(71, 70, 70));
        assertFalse(TransferThreadCountTest.hueInRange(360, 0, 50));
    }

    @Test
    public void aDeadInputReadsNoSignalNotRed() {
        assertEquals(TransferThreadCountTest.NO_SIGNAL, TransferThreadCountTest.ballColour(0));
        assertEquals(TransferThreadCountTest.NO_SIGNAL, TransferThreadCountTest.ballColour(1.9));
        assertEquals(TransferThreadCountTest.NONE, TransferThreadCountTest.ballColour(2));
        assertEquals(TransferThreadCountTest.NONE, TransferThreadCountTest.ballColour(360));
    }

    @Test
    public void theDefaultRangesNameEachBallColour() {
        assertEquals(TransferThreadCountTest.NECTAR_RED, TransferThreadCountTest.ballColour(30));
        assertEquals(TransferThreadCountTest.NECTAR_BLUE, TransferThreadCountTest.ballColour(190));
        assertEquals(TransferThreadCountTest.POLLEN, TransferThreadCountTest.ballColour(86));
        assertEquals(TransferThreadCountTest.NONE, TransferThreadCountTest.ballColour(93));
    }
}
