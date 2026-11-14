package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightSync;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class LimelightBenchTest {

    @Test
    public void findsTheSourcesPrivateSyncRequest() {
        Field sync = LimelightBench.syncField();
        assertEquals(LimelightSync.Request.class, sync.getType());
        assertTrue(Modifier.isPrivate(sync.getModifiers()));
    }
}
