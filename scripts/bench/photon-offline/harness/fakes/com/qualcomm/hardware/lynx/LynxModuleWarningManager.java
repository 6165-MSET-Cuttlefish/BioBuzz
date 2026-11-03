package com.qualcomm.hardware.lynx;

import java.util.concurrent.atomic.AtomicInteger;

public class LynxModuleWarningManager {
    private static final LynxModuleWarningManager instance = new LynxModuleWarningManager();
    public static final AtomicInteger UNRESPONSIVE = new AtomicInteger();

    public static LynxModuleWarningManager getInstance() { return instance; }

    public void reportModuleUnresponsive(LynxModule module) { UNRESPONSIVE.incrementAndGet(); }
    public void reportModuleReset(LynxModule module) {}
    public void reportModuleLowBattery(LynxModule module) {}
}
