package org.firstinspires.ftc.teamcode.architecture.hardware.photon;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicLong;

import dev.frozenmilk.sinister.loading.Pinned;

/**
 * Swaps every hub in the hardware map for a {@link PhotonLynxModule} at the INIT of an OpMode annotated {@link Photon};
 * {@link #enable()} then lets those hubs send motor-power and servo-pulse writes without waiting for each ACK. Only a
 * {@link Photon} INIT swaps hubs, but OpModes after it keep the swapped hubs, disabled, until the hardware map is rebuilt
 * (config change or Restart Robot).
 */
@Pinned
public final class PhotonCore implements Runnable, OpModeManagerNotifier.Notifications {
    static final String TAG = "Photon";
    private static final long SWEEP_MS = 5;
    private static final Set<String> SYSTEM_OPMODES = new HashSet<>(Arrays.asList(
            OpModeManagerImpl.DEFAULT_OP_MODE_NAME, "$Manual$Control$", "$Process$Load$Event$"));
    private static final PhotonCore INSTANCE = new PhotonCore();
    private static volatile int maxParallelWrites = 8;

    final CopyOnWriteArrayList<PhotonLynxModule> hubs = new CopyOnWriteArrayList<>();
    final AtomicLong parallelWrites = new AtomicLong();
    final AtomicLong retransmits = new AtomicLong();
    final AtomicLong abandonedWrites = new AtomicLong();
    final AtomicLong supersededWrites = new AtomicLong();
    final AtomicLong gateFallbacks = new AtomicLong();
    final AtomicLong postStopFailSafes = new AtomicLong();
    final AtomicLong slotTimeouts = new AtomicLong();
    final AtomicLong notRespondingFallbacks = new AtomicLong();
    final AtomicLong deviceNotReadyFallbacks = new AtomicLong();
    final AtomicLong writeFailures = new AtomicLong();
    final AtomicLong clearedAtInit = new AtomicLong();
    final AtomicLong orphanDatagrams = new AtomicLong();
    private final ConcurrentHashMap<String, AtomicLong> nacks = new ConcurrentHashMap<>();

    private volatile boolean enabled;
    private volatile OpModeManagerImpl opModeManager;
    private volatile OpMode photonOpMode;
    private volatile HardwareMap photonMap;
    private volatile List<String> swapFailures = Collections.emptyList();
    private volatile Throwable sweeperFault;
    private Thread sweeper;
    private Fields fields;

    private PhotonCore() {
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        OpModeManagerImpl manager = eventLoop.getOpModeManager();
        manager.registerListener(INSTANCE);
        INSTANCE.opModeManager = manager;
    }

    /** Throws {@link IllegalStateException} unless the running OpMode is {@link Photon} and its hubs are Photon's. */
    public static void enable() {
        INSTANCE.checkCanEnable();
        INSTANCE.enabled = true;
    }

    public static void disable() {
        INSTANCE.enabled = false;
    }

    public static boolean isEnabled() {
        return INSTANCE.enabled;
    }

    /** Per hub. */
    public static void setMaxParallelWrites(int writes) {
        if (writes < 1) throw new IllegalArgumentException("Photon needs at least 1 parallel write per hub, got " + writes);
        maxParallelWrites = writes;
    }

    public static int maxParallelWrites() {
        return maxParallelWrites;
    }

    public static boolean isSwapped(LynxModule hub) {
        return INSTANCE.owns(hub);
    }

    /** Why the last {@link Photon} INIT couldn't take over the hubs; empty when it could. */
    public static List<String> swapFailures() {
        return INSTANCE.swapFailures;
    }

    /** Parallel writes sent to this hub whose real ACK hasn't arrived yet. */
    public static int inFlight(LynxModule hub) {
        if (!INSTANCE.owns(hub)) throw new IllegalArgumentException("hub #" + hub.getModuleAddress() + " isn't Photon's");
        return ((PhotonLynxModule) hub).inFlight();
    }

    public static Counters counters() {
        return new Counters(INSTANCE);
    }

    public static void resetCounters() {
        PhotonCore c = INSTANCE;
        for (AtomicLong counter : new AtomicLong[] {c.parallelWrites, c.retransmits, c.abandonedWrites,
                c.supersededWrites, c.gateFallbacks, c.postStopFailSafes, c.slotTimeouts, c.notRespondingFallbacks,
                c.deviceNotReadyFallbacks, c.writeFailures, c.clearedAtInit, c.orphanDatagrams}) {
            counter.set(0);
        }
        c.nacks.clear();
    }

    boolean parallelAllowed() {
        return enabled && sweeperFault == null;
    }

    void countNack(String reason) {
        AtomicLong counter = nacks.get(reason);
        if (counter == null) {
            AtomicLong fresh = new AtomicLong();
            counter = nacks.putIfAbsent(reason, fresh);
            if (counter == null) counter = fresh;
        }
        counter.incrementAndGet();
    }

    private boolean owns(LynxModule hub) {
        return hub instanceof PhotonLynxModule && ((PhotonLynxModule) hub).core == this;
    }

    private void checkCanEnable() {
        OpModeManagerImpl manager = opModeManager;
        OpMode active = manager == null ? null : manager.getActiveOpMode();
        if (active == null || active != photonOpMode) {
            throw new IllegalStateException("PhotonCore.enable() needs an OpMode annotated @Photon: Photon didn't take "
                    + "over this OpMode's hubs at INIT");
        }
        List<String> failures = swapFailures;
        if (!failures.isEmpty()) {
            StringBuilder why = new StringBuilder("Photon couldn't take over the hubs:");
            for (String failure : failures) why.append("\n- ").append(failure);
            throw new IllegalStateException(why.toString());
        }
        Throwable fault = sweeperFault;
        if (fault != null) throw new IllegalStateException("Photon's sweeper stopped", fault);
        for (HardwareDevice device : photonMap.unsafeIterable()) {
            if (device instanceof LynxModule && !owns((LynxModule) device)) {
                throw new IllegalStateException("REV Hub #" + ((LynxModule) device).getModuleAddress()
                        + " isn't Photon's (" + device.getClass().getName() + ")");
            }
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        enabled = false;
        photonOpMode = null;
        photonMap = null;
        swapFailures = Collections.emptyList();
        for (PhotonLynxModule hub : hubs) {
            if (hub.isOpen()) hub.clearParallelWrites();
            else hubs.remove(hub);
        }
        OpModeManagerImpl manager = opModeManager;
        if (opMode == null || manager == null || SYSTEM_OPMODES.contains(manager.getActiveOpModeName())
                || !opMode.getClass().isAnnotationPresent(Photon.class)) {
            return;
        }
        List<String> problems = new ArrayList<>();
        try {
            takeOver(opMode.hardwareMap, problems);
        } catch (ReflectiveOperationException | RuntimeException e) {
            // Throwing here would put the whole robot in EMERGENCY_STOP; enable() throws on the OpMode thread instead.
            RobotLog.ee(TAG, e, "Photon couldn't take over the hubs");
            problems.add("taking over the hubs threw " + e);
        }
        for (String problem : problems) RobotLog.ee(TAG, "%s", problem);
        if (!problems.isEmpty()) RobotLog.addGlobalWarningMessage("Photon is off for this OpMode: " + problems.get(0));
        swapFailures = Collections.unmodifiableList(problems);
        photonMap = opMode.hardwareMap;
        photonOpMode = opMode;
        startSweeperIfStopped();
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        enabled = false;
    }

    @Override
    public void run() {
        try {
            while (true) {
                long now = System.nanoTime();
                for (PhotonLynxModule hub : hubs) {
                    hub.sweep(now);
                    hub.disarmOrphanPing();
                }
                Thread.sleep(SWEEP_MS);
            }
        } catch (Throwable t) {
            sweeperFault = t;
            RobotLog.ee(TAG, t, "Photon's sweeper stopped; parallel writes are off until the next @Photon INIT");
            RobotLog.addGlobalWarningMessage("Photon's sweeper stopped: " + t);
        }
    }

    private void startSweeperIfStopped() {
        if (hubs.isEmpty()) return;
        Thread running = sweeper;
        if (running != null && running.isAlive()) return;
        sweeperFault = null;
        Thread thread = new Thread(this, "Photon sweeper");
        thread.setDaemon(true);
        sweeper = thread;
        thread.start();
    }

    private void takeOver(HardwareMap map, List<String> problems) throws ReflectiveOperationException {
        if (fields == null) fields = new Fields();
        Fields f = fields;
        String unhandled = PhotonLynxModule.unhandledFields();
        if (unhandled != null) {
            problems.add(unhandled);
            return;
        }

        List<LynxModule> originals = new ArrayList<>();
        for (HardwareDevice device : map.unsafeIterable()) {
            if (!(device instanceof LynxModule)) continue;
            LynxModule hub = (LynxModule) device;
            if (hub instanceof PhotonLynxModule) {
                if (!owns(hub)) problems.add("REV Hub #" + hub.getModuleAddress() + " belongs to another PhotonCore");
                continue;
            }
            if (hub.getClass() != LynxModule.class) {
                problems.add("REV Hub #" + hub.getModuleAddress() + " is a " + hub.getClass().getName()
                        + ", which Photon can't take over");
                continue;
            }
            if (map.getNamesOf(hub).isEmpty()) problems.add("REV Hub #" + hub.getModuleAddress() + " has no name");
            if (f.knownModules(usbImpl(f, hub)).get(hub.getModuleAddress()) != hub) {
                problems.add("replies for REV Hub #" + hub.getModuleAddress() + " already go to another module");
            }
            originals.add(hub);
        }
        Set<LynxModule> targets = identitySet();
        targets.addAll(originals);
        List<Ref> refs = new ArrayList<>();
        findReferences(map, targets, refs);
        for (Ref ref : refs) {
            if (Modifier.isFinal(ref.field.getModifiers())) problems.add(ref.describe(map) + " is final");
        }
        if (!problems.isEmpty()) return;

        Map<LynxModule, PhotonLynxModule> swapped = new IdentityHashMap<>();
        for (LynxModule hub : originals) {
            swapped.put(hub, new PhotonLynxModule(this, hub, (LynxUsbDevice) f.moduleUsbDevice.get(hub), f));
        }
        List<PhotonLynxModule> routed = new ArrayList<>();
        for (Map.Entry<LynxModule, PhotonLynxModule> e : swapped.entrySet()) {
            ConcurrentHashMap<Integer, LynxModule> known = f.knownModules(e.getValue().usbImpl());
            boolean replaced;
            synchronized (known) {
                replaced = known.replace(e.getKey().getModuleAddress(), e.getKey(), e.getValue());
            }
            if (!replaced) {
                problems.add("replies for REV Hub #" + e.getKey().getModuleAddress() + " changed owner during INIT");
                break;
            }
            routed.add(e.getValue());
        }
        if (!problems.isEmpty()) {
            for (PhotonLynxModule hub : routed) {
                ConcurrentHashMap<Integer, LynxModule> known = f.knownModules(hub.usbImpl());
                synchronized (known) {
                    known.replace(hub.getModuleAddress(), hub, hub.orphan);
                }
            }
            return;
        }

        for (Map.Entry<LynxModule, PhotonLynxModule> e : swapped.entrySet()) {
            LynxModule hub = e.getKey();
            PhotonLynxModule photon = e.getValue();
            for (String name : new ArrayList<>(map.getNamesOf(hub))) {
                map.remove(hub.getModuleSerialNumber(), name, hub);
                map.put(photon.getModuleSerialNumber(), name, photon);
            }
            if (EmbeddedControlHubModule.get() == hub) EmbeddedControlHubModule.set(photon);
        }
        for (Ref ref : refs) ref.field.set(ref.holder, swapped.get(ref.target));
        for (Map.Entry<LynxModule, PhotonLynxModule> e : swapped.entrySet()) {
            f.stopPingTimer.invoke(e.getKey(), false);
            e.getValue().restartPingTimer();
            hubs.add(e.getValue());
        }
        verify(map, f, problems);
    }

    private void verify(HardwareMap map, Fields f, List<String> problems) throws IllegalAccessException {
        Set<LynxModule> orphans = identitySet();
        Set<LynxUsbDeviceImpl> impls = Collections.newSetFromMap(new IdentityHashMap<LynxUsbDeviceImpl, Boolean>());
        for (HardwareDevice device : map.unsafeIterable()) {
            if (!(device instanceof LynxModule)) continue;
            if (!owns((LynxModule) device)) {
                problems.add("REV Hub #" + ((LynxModule) device).getModuleAddress() + " is still the SDK's LynxModule");
                continue;
            }
            PhotonLynxModule hub = (PhotonLynxModule) device;
            orphans.add(hub.orphan);
            impls.add(hub.usbImpl());
            if (f.knownModules(hub.usbImpl()).get(hub.getModuleAddress()) != hub) {
                problems.add("replies for REV Hub #" + hub.getModuleAddress() + " don't reach Photon's module");
            }
        }
        for (LynxUsbDeviceImpl impl : impls) {
            for (LynxModule known : f.knownModules(impl).values()) {
                if (orphans.contains(known)) {
                    problems.add("USB device " + impl.getSerialNumber() + " still routes REV Hub #"
                            + known.getModuleAddress() + " to the replaced module");
                }
            }
        }
        if (orphans.contains(EmbeddedControlHubModule.get())) {
            problems.add("the SDK's embedded Control Hub module is still the replaced one");
        }
        List<Ref> left = new ArrayList<>();
        findReferences(map, orphans, left);
        for (Ref ref : left) {
            problems.add(ref.describe(map) + " still points at the replaced REV Hub #" + ref.target.getModuleAddress());
        }
    }

    private static LynxUsbDeviceImpl usbImpl(Fields f, LynxModule hub) throws IllegalAccessException {
        return ((LynxUsbDevice) f.moduleUsbDevice.get(hub)).getDelegationTarget();
    }

    private static Set<LynxModule> identitySet() {
        return Collections.newSetFromMap(new IdentityHashMap<LynxModule, Boolean>());
    }

    private static void findReferences(HardwareMap map, Set<LynxModule> targets, List<Ref> refs)
            throws IllegalAccessException {
        Set<Object> seen = Collections.newSetFromMap(new IdentityHashMap<Object, Boolean>());
        for (HardwareDevice root : map.unsafeIterable()) {
            if (root instanceof LynxModuleIntf || root instanceof LynxUsbDevice) continue;
            collectReferences(root, root, targets, refs, seen);
        }
    }

    private static void collectReferences(HardwareDevice root, Object holder, Set<LynxModule> targets, List<Ref> refs,
                                          Set<Object> seen) throws IllegalAccessException {
        if (!seen.add(holder)) return;
        for (Class<?> c = holder.getClass(); c != null && !isPlatformClass(c); c = c.getSuperclass()) {
            for (Field field : c.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) || field.getType().isPrimitive()) continue;
                field.setAccessible(true);
                Object value = field.get(holder);
                if (value instanceof LynxModule) {
                    if (targets.contains(value)) refs.add(new Ref(root, holder, field, (LynxModule) value));
                } else if (value instanceof HardwareDevice && !(value instanceof LynxModuleIntf)
                        && !(value instanceof LynxUsbDevice)) {
                    collectReferences(root, value, targets, refs, seen);
                }
            }
        }
    }

    private static boolean isPlatformClass(Class<?> c) {
        String name = c.getName();
        return name.startsWith("java.") || name.startsWith("javax.") || name.startsWith("android.")
                || name.startsWith("androidx.") || name.startsWith("kotlin.") || name.startsWith("dalvik.")
                || name.startsWith("libcore.");
    }

    /** A snapshot of Photon's counters since app start or the last {@link #resetCounters()}, over all hubs. */
    @Pinned
    public static final class Counters {
        public final long parallelWrites;
        public final long retransmits;
        /** Parallel writes with no ACK by the SDK's ack timeout: handled like the SDK's own timeout. */
        public final long abandonedWrites;
        /** Unacknowledged parallel writes a newer write or a fail-safe made moot, while the hub kept answering. */
        public final long supersededWrites;
        /** Writes sent the SDK's way because STOP had begun, so the SDK refuses them. */
        public final long gateFallbacks;
        /** Fail-safes Photon sent because STOP began while a parallel write was going out. */
        public final long postStopFailSafes;
        public final long slotTimeouts;
        public final long notRespondingFallbacks;
        public final long deviceNotReadyFallbacks;
        public final long writeFailures;
        /** Parallel writes still unacknowledged at the next INIT, dropped there. */
        public final long clearedAtInit;
        /** Replies routed to a replaced hub object because a command was still sent through it. */
        public final long orphanDatagrams;
        /** NACKs of parallel writes, by reason. */
        public final Map<String, Long> nacks;

        Counters(PhotonCore c) {
            parallelWrites = c.parallelWrites.get();
            retransmits = c.retransmits.get();
            abandonedWrites = c.abandonedWrites.get();
            supersededWrites = c.supersededWrites.get();
            gateFallbacks = c.gateFallbacks.get();
            postStopFailSafes = c.postStopFailSafes.get();
            slotTimeouts = c.slotTimeouts.get();
            notRespondingFallbacks = c.notRespondingFallbacks.get();
            deviceNotReadyFallbacks = c.deviceNotReadyFallbacks.get();
            writeFailures = c.writeFailures.get();
            clearedAtInit = c.clearedAtInit.get();
            orphanDatagrams = c.orphanDatagrams.get();
            Map<String, Long> byReason = new LinkedHashMap<>();
            for (Map.Entry<String, AtomicLong> e : c.nacks.entrySet()) byReason.put(e.getKey(), e.getValue().get());
            nacks = Collections.unmodifiableMap(byReason);
        }

        /** Writes Photon would have parallelized but sent the SDK's way, other than at STOP. */
        public long stockFallbacks() {
            return slotTimeouts + notRespondingFallbacks + deviceNotReadyFallbacks + writeFailures;
        }
    }

    @Pinned
    static final class Fields {
        final Field moduleUsbDevice;
        final Field moduleUnfinished;
        final Method stopPingTimer;
        final Field knownModules;
        final Field engageLock;
        final Field implEngaged;
        final Field notResponding;

        Fields() throws NoSuchFieldException, NoSuchMethodException {
            moduleUsbDevice = accessible(LynxModule.class, "lynxUsbDevice", LynxUsbDevice.class);
            moduleUnfinished = accessible(LynxModule.class, "unfinishedCommands", ConcurrentHashMap.class);
            stopPingTimer = LynxModule.class.getDeclaredMethod("stopPingTimer", boolean.class);
            stopPingTimer.setAccessible(true);
            knownModules = accessible(LynxUsbDeviceImpl.class, "knownModules", ConcurrentHashMap.class);
            engageLock = accessible(LynxUsbDeviceImpl.class, "engageLock", Object.class);
            implEngaged = accessible(LynxUsbDeviceImpl.class, "isEngaged", boolean.class);
            notResponding = accessible(LynxModule.class, "isNotResponding", boolean.class);
        }

        private static Field accessible(Class<?> owner, String name, Class<?> type) throws NoSuchFieldException {
            Field field = owner.getDeclaredField(name);
            if (field.getType() != type) {
                throw new NoSuchFieldException(owner.getSimpleName() + "." + name + " is a " + field.getType().getName()
                        + ", not a " + type.getName());
            }
            field.setAccessible(true);
            return field;
        }

        @SuppressWarnings("unchecked")
        ConcurrentHashMap<Integer, LynxModule> knownModules(LynxUsbDeviceImpl impl) throws IllegalAccessException {
            return (ConcurrentHashMap<Integer, LynxModule>) knownModules.get(impl);
        }
    }

    @Pinned
    static final class Ref {
        final HardwareDevice root;
        final Object holder;
        final Field field;
        final LynxModule target;

        Ref(HardwareDevice root, Object holder, Field field, LynxModule target) {
            this.root = root;
            this.holder = holder;
            this.field = field;
            this.target = target;
        }

        String describe(HardwareMap map) {
            Set<String> names = map.getNamesOf(root);
            String name = names.isEmpty() ? root.getClass().getSimpleName() : names.iterator().next();
            return "device \"" + name + "\" (" + field.getDeclaringClass().getSimpleName() + "." + field.getName() + ")";
        }
    }
}
