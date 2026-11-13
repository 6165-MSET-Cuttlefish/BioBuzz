package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.dashboard.FtcDashboard;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

public class DashboardOffBenchTest {

    @Test
    public void findsThePinnedDashboardsPrivateDisable() {
        Method disable = DashboardOffBench.privateMethod(FtcDashboard.class, DashboardOffBench.DISABLE);
        assertTrue(Modifier.isPrivate(disable.getModifiers()));
        assertEquals(void.class, disable.getReturnType());
        assertEquals(0, disable.getParameterCount());
    }

    @Test
    public void aRenamedToggleFailsNamingIt() {
        IllegalStateException e = assertThrows(IllegalStateException.class,
                () -> DashboardOffBench.privateMethod(FtcDashboard.class, "disableDashboard"));
        assertTrue(e.getMessage(), e.getMessage().contains("disableDashboard()"));
    }
}
