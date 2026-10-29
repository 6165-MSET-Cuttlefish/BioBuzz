package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.LastKnown;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonLynxModule;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Collections;

import dev.frozenmilk.sinister.loading.Pinned;

final class PhotonProbe {
    private PhotonProbe() {}

    static Field field(Class<?> owner, String name) {
        for (Class<?> c = owner; c != null; c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                if (!f.getName().equals(name)) continue;
                f.setAccessible(true);
                return f;
            }
        }
        throw new IllegalStateException(owner.getName() + "." + name + " is gone: the bench needs updating for this SDK or Photon");
    }

    static Object get(Field f, Object holder) {
        try {
            return f.get(holder);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("can't read " + f, e);
        }
    }

    private static final Field MODULE_USB = field(LynxModule.class, "lynxUsbDevice");
    private static final Field KNOWN_MODULES = field(LynxUsbDeviceImpl.class, "knownModules");
    private static final Field ENGAGE_LOCK = field(LynxUsbDeviceImpl.class, "engageLock");
    private static final Field CONTROLLER_MODULE = field(LynxController.class, "module");
    private static final Field ORPHAN = field(PhotonLynxModule.class, "orphan");
    private static final Field PARALLEL_WRITES = field(PhotonLynxModule.class, "parallelWrites");
    private static final Field ROUTES = field(PhotonLynxModule.class, "routes");
    private static final Field UNFINISHED = field(LynxModule.class, "unfinishedCommands");
    private static final Field ATTENTION = field(LynxModule.class, "attentionRequiredPreviously");
    private static final String STAND_IN = PhotonLynxModule.class.getName() + "$ParallelWrite";
    private static final Field CORE_INSTANCE = field(PhotonCore.class, "INSTANCE");
    private static final Field CORE_HUBS = field(PhotonCore.class, "hubs");
    private static final Field GATE = field(OpModeManagerImpl.class, "preventDangerousHardwareAccess");

    static LynxUsbDeviceImpl impl(LynxModule hub) {
        return ((LynxUsbDevice) get(MODULE_USB, hub)).getDelegationTarget();
    }

    static Object engageLock(LynxUsbDeviceImpl impl) {
        return get(ENGAGE_LOCK, impl);
    }

    @SuppressWarnings("unchecked")
    static Map<Integer, LynxModule> knownModules(LynxUsbDeviceImpl impl) {
        return (Map<Integer, LynxModule>) get(KNOWN_MODULES, impl);
    }

    static int inFlight(LynxModule hub) {
        return PhotonCore.isSwapped(hub) ? PhotonCore.inFlight(hub) : 0;
    }

    static boolean gateUp() {
        return OpModeManagerImpl.shouldPreventDangerousHardwareAccess();
    }

    // Only ever raised: the next OpMode's INIT lowers it, and lowering it mid-STOP would re-open the gate.
    static void raiseGate() {
        try {
            GATE.setBoolean(null, true);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("can't raise the STOP gate", e);
        }
    }

    static List<Integer> swapOrder() {
        List<Integer> out = new ArrayList<>();
        for (Object hub : (Collection<?>) get(CORE_HUBS, get(CORE_INSTANCE, null))) {
            out.add(((LynxModule) hub).getModuleAddress());
        }
        return out;
    }

    static double oldestPendingMs(LynxModule hub) {
        if (!PhotonCore.isSwapped(hub)) return 0;
        long now = System.nanoTime();
        long oldest = 0;
        for (Object w : ((Map<?, ?>) get(PARALLEL_WRITES, hub)).values()) {
            long sent = (Long) get(field(w.getClass(), "firstSentNanos"), w);
            if (sent != 0) oldest = Math.max(oldest, now - sent);
        }
        return oldest / 1e6;
    }

    static int pendingEntries(LynxModule hub) {
        return PhotonCore.isSwapped(hub) ? ((Map<?, ?>) get(PARALLEL_WRITES, hub)).size() : 0;
    }

    // Counted in the SDK's own map, not Photon's bookkeeping: a stand-in left there holds a message number for good.
    static int standIns(LynxModule hub) {
        int n = 0;
        for (Object command : ((Map<?, ?>) get(UNFINISHED, hub)).values()) {
            if (command.getClass().getName().equals(STAND_IN)) n++;
        }
        return n;
    }

    static int unfinished(LynxModule hub) {
        return ((Map<?, ?>) get(UNFINISHED, hub)).size();
    }

    static int routes(LynxModule hub) {
        return PhotonCore.isSwapped(hub) ? ((Map<?, ?>) get(ROUTES, hub)).size() : 0;
    }

    static boolean attentionRequired(LynxModule hub) {
        return (Boolean) get(ATTENTION, hub);
    }

    /** What Photon could have left behind on a hub; every value is 0 on a clean hub. */
    static Map<String, Object> leftovers(LynxModule hub) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("inFlight", inFlight(hub));
        m.put("pendingEntries", pendingEntries(hub));
        m.put("standIns", standIns(hub));
        m.put("routes", routes(hub));
        return m;
    }

    static boolean motorCacheValid(DcMotorEx motor) {
        Object controller = ((DcMotorImpl) motor).getController();
        Object[] motors = (Object[]) get(field(controller.getClass(), "motors"), controller);
        Object props = motors[motor.getPortNumber()];
        return ((LastKnown<?>) get(field(props.getClass(), "lastKnownPower"), props)).isValid();
    }

    static boolean servoCacheValid(ServoImplEx servo) {
        Object controller = servo.getController();
        Object[] positions = (Object[]) get(field(controller.getClass(), "lastKnownCommandedPosition"), controller);
        return ((LastKnown<?>) positions[servo.getPortNumber()]).isValid();
    }

    static List<String> routingProblems(HardwareMap map) {
        List<String> problems = new ArrayList<>();
        Map<LynxUsbDeviceImpl, List<LynxModule>> byImpl = new IdentityHashMap<>();
        Set<Object> orphans = Collections.newSetFromMap(new IdentityHashMap<Object, Boolean>());
        for (LynxModule hub : hubs(map)) {
            if (!PhotonCore.isSwapped(hub)) {
                problems.add("REV Hub #" + hub.getModuleAddress() + " is a " + hub.getClass().getName() + ", not Photon's");
                continue;
            }
            orphans.add(get(ORPHAN, hub));
            LynxUsbDeviceImpl impl = impl(hub);
            List<LynxModule> list = byImpl.get(impl);
            if (list == null) byImpl.put(impl, list = new ArrayList<>());
            list.add(hub);
        }
        for (Map.Entry<LynxUsbDeviceImpl, List<LynxModule>> e : byImpl.entrySet()) {
            Map<Integer, LynxModule> known = knownModules(e.getKey());
            String usb = e.getKey().getSerialNumber().toString();
            for (LynxModule hub : e.getValue()) {
                LynxModule routed = known.get(hub.getModuleAddress());
                if (routed != hub) {
                    problems.add("USB device " + usb + " routes REV Hub #" + hub.getModuleAddress() + "'s replies to "
                            + (routed == null ? "nothing" : routed.getClass().getName() + "@" + System.identityHashCode(routed)));
                }
            }
            for (Map.Entry<Integer, LynxModule> k : known.entrySet()) {
                if (orphans.contains(k.getValue())) {
                    problems.add("USB device " + usb + " still routes address " + k.getKey() + " to a replaced module");
                }
                boolean ours = false;
                for (LynxModule hub : e.getValue()) ours |= hub.getModuleAddress() == k.getKey();
                if (!ours) problems.add("USB device " + usb + " routes address " + k.getKey() + ", which isn't one of its hubs in the map");
            }
        }
        for (HardwareDevice device : map.unsafeIterable()) {
            Object controller = controllerOf(device);
            if (!(controller instanceof LynxController)) continue;
            Object module = get(CONTROLLER_MODULE, controller);
            if (orphans.contains(module) || !(module instanceof PhotonLynxModule)) {
                problems.add("device \"" + firstName(map, device) + "\"'s controller talks to "
                        + module.getClass().getName() + "@" + System.identityHashCode(module));
            }
        }
        if (orphans.contains(EmbeddedControlHubModule.get())) problems.add("EmbeddedControlHubModule is the replaced module");
        return problems;
    }

    private static Object controllerOf(HardwareDevice device) {
        if (device instanceof DcMotorImpl) return ((DcMotorImpl) device).getController();
        if (device instanceof ServoImplEx) return ((ServoImplEx) device).getController();
        return null;
    }

    private static String firstName(HardwareMap map, HardwareDevice device) {
        Set<String> names = map.getNamesOf(device);
        return names.isEmpty() ? device.getClass().getSimpleName() : names.iterator().next();
    }

    static List<LynxModule> hubs(HardwareMap map) {
        List<LynxModule> out = new ArrayList<>();
        for (HardwareDevice d : map.unsafeIterable()) if (d instanceof LynxModule) out.add((LynxModule) d);
        return out;
    }

    static Map<String, Object> counters() {
        PhotonCore.Counters c = PhotonCore.counters();
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("parallelWrites", c.parallelWrites);
        m.put("retransmits", c.retransmits);
        m.put("abandonedWrites", c.abandonedWrites);
        m.put("supersededWrites", c.supersededWrites);
        m.put("gateFallbacks", c.gateFallbacks);
        m.put("postStopFailSafes", c.postStopFailSafes);
        m.put("slotTimeouts", c.slotTimeouts);
        m.put("notRespondingFallbacks", c.notRespondingFallbacks);
        m.put("deviceNotReadyFallbacks", c.deviceNotReadyFallbacks);
        m.put("writeFailures", c.writeFailures);
        m.put("clearedAtInit", c.clearedAtInit);
        m.put("orphanDatagrams", c.orphanDatagrams);
        m.put("stockFallbacks", c.stockFallbacks());
        m.put("nacks", new LinkedHashMap<>(c.nacks));
        return m;
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> delta(Map<String, Object> before, Map<String, Object> after) {
        Map<String, Object> d = new LinkedHashMap<>();
        for (Map.Entry<String, Object> e : after.entrySet()) {
            if (e.getValue() instanceof Number) {
                d.put(e.getKey(), ((Number) e.getValue()).longValue() - ((Number) before.get(e.getKey())).longValue());
            }
        }
        Map<String, Long> nacks = new LinkedHashMap<>();
        Map<String, Long> was = (Map<String, Long>) before.get("nacks");
        for (Map.Entry<String, Long> e : ((Map<String, Long>) after.get("nacks")).entrySet()) {
            long n = e.getValue() - (was.containsKey(e.getKey()) ? was.get(e.getKey()) : 0);
            if (n != 0) nacks.put(e.getKey(), n);
        }
        d.put("nacks", nacks);
        return d;
    }

    static Map<String, Object> loaders(Object bench, HardwareMap map) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("photonCoreLoader", describe(PhotonCore.class.getClassLoader()));
        m.put("photonModuleLoader", describe(PhotonLynxModule.class.getClassLoader()));
        m.put("benchLoader", describe(bench.getClass().getClassLoader()));
        m.put("photonCorePinned", PhotonCore.class.isAnnotationPresent(Pinned.class));
        m.put("photonModulePinned", PhotonLynxModule.class.isAnnotationPresent(Pinned.class));
        List<Map<String, Object>> hubs = new ArrayList<>();
        for (LynxModule hub : hubs(map)) {
            Map<String, Object> h = new LinkedHashMap<>();
            h.put("address", hub.getModuleAddress());
            h.put("identity", System.identityHashCode(hub));
            h.put("classLoader", describe(hub.getClass().getClassLoader()));
            hubs.add(h);
        }
        m.put("hubs", hubs);
        m.put("pid", android.os.Process.myPid());
        return m;
    }

    private static String describe(ClassLoader loader) {
        return loader == null ? "boot" : loader.getClass().getName() + "@" + Integer.toHexString(System.identityHashCode(loader));
    }
}
