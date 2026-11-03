package com.qualcomm.hardware.lynx;

import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class StockPhotonPerf {
    static void run() throws Exception {
        PhotonHarness.Rig srig = new PhotonHarness.Rig(174);
        srig.hub.ackDelayMs = 0;
        LynxModule orig = srig.originals.get(0);
        com.seattlesolvers.solverslib.photon.PhotonLynxModule stock =
                new com.seattlesolvers.solverslib.photon.PhotonLynxModule(srig.impl, 174, true, false);
        PhotonHarness.registerDeka(stock);
        srig.impl.knownModules.put(174, stock);
        Class<?> spc = com.seattlesolvers.solverslib.photon.PhotonCore.class;
        Object inst = PerfHarness.f(spc, "instance").get(null);
        ((AtomicBoolean) PerfHarness.f(spc, "enabled").get(inst)).set(true);
        @SuppressWarnings("unchecked")
        Map<LynxModule, RobotUsbDevice> map = (Map<LynxModule, RobotUsbDevice>) PerfHarness.f(spc, "usbDeviceMap").get(inst);
        map.put(stock, srig.impl.getRobotUsbDevice());
        PerfHarness.f(spc, "syncLock").set(inst, PerfHarness.f(LynxUsbDeviceImpl.class, "engageLock").get(srig.impl));
        PerfHarness.bench("stock SolversLib 0.3.6", i -> PerfHarness.send(stock, i), stock.unfinishedCommands, srig.hub);
        orig.getModuleAddress();
    }
}
