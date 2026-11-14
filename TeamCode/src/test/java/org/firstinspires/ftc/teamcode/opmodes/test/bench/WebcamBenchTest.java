package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class WebcamBenchTest {

    @Test
    public void findsCellTipCamerasPrivateSession() {
        Field session = WebcamBench.sessionField();
        assertEquals(WebcamSession.class, session.getType());
        assertTrue(Modifier.isPrivate(session.getModifiers()));
    }

    @Test
    public void rejectFindTriesEveryValueOfASmallRange() {
        assertEquals(Arrays.asList(3, 4, 5, 6), WebcamBench.sweep(3, 6, 150));
        assertEquals(Collections.singletonList(0), WebcamBench.sweep(0, 0, 150));
        assertEquals(Collections.emptyList(), WebcamBench.sweep(5, 4, 150));
    }

    @Test
    public void rejectFindCapsALargeRangeFromItsMinAndEndsOnItsMax() {
        List<Integer> values = WebcamBench.sweep(2800, 6500, 150);
        assertEquals(150, values.size());
        assertEquals(2800, (int) values.get(0));
        assertEquals(2948, (int) values.get(148));
        assertEquals(6500, (int) values.get(149));
    }
}
