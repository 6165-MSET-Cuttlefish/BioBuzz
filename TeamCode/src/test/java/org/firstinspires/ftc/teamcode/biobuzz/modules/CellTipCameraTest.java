package org.firstinspires.ftc.teamcode.biobuzz.modules;

import org.junit.Test;

public class CellTipCameraTest {

    @Test
    public void stopBeforeInitDoesNothing() {
        new CellTipCamera(null).stop();
    }
}
