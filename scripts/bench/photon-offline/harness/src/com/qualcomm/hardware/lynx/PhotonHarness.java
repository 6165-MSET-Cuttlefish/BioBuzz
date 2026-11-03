package com.qualcomm.hardware.lynx;

import android.util.Log;

import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxInterface;
import com.qualcomm.hardware.lynx.commands.LynxInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxFailSafeCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxKeepAliveCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.hardware.RobotCoreLynxModule;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.WeakReferenceSet;

import org.firstinspires.ftc.robotcore.internal.hardware.usb.ArmableUsbDevice;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbUnspecifiedException;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonLynxModule;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.LockSupport;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

public class PhotonHarness {
    static int failures = 0;
    static final AtomicInteger RIGS = new AtomicInteger();
    static int passes = 0;
    static sun.misc.Unsafe UNSAFE;

    static void check(boolean ok, String what) {
        System.out.println((ok ? "PASS " : "FAIL ") + what);
        if (ok) passes++; else failures++;
    }

    static Field field(Class<?> c, String name) throws Exception {
        Field f = c.getDeclaredField(name);
        f.setAccessible(true);
        return f;
    }

    static void set(Object o, Class<?> c, String name, Object v) throws Exception { field(c, name).set(o, v); }

    static Object get(Object o, Class<?> c, String name) throws Exception { return field(c, name).get(o); }

    static void setStatic(Class<?> c, String name, Object v) throws Exception { field(c, name).set(null, v); }

    static Object call(Object target, Class<?> c, String name, Class<?>[] types, Object... args) throws Exception {
        Method m = c.getDeclaredMethod(name, types);
        m.setAccessible(true);
        return m.invoke(target, args);
    }

    static Object core() throws Exception { return get(null, PhotonCore.class, "INSTANCE"); }

    static void enablePhoton(boolean on) throws Exception { set(core(), PhotonCore.class, "enabled", on); }

    static void prevent(boolean on) throws Exception { setStatic(OpModeManagerImpl.class, "preventDangerousHardwareAccess", on); }

    static void sleep(long ms) throws InterruptedException { Thread.sleep(ms); }

    static final class FakeHub implements InvocationHandler {
        final List<LynxDatagram> written = new CopyOnWriteArrayList<>();
        final ScheduledExecutorService poller = Executors.newSingleThreadScheduledExecutor(r -> { Thread t = new Thread(r, "fake poller"); t.setDaemon(true); return t; });
        volatile LynxUsbDeviceImpl impl;
        volatile int ackDelayMs = 2;
        volatile Predicate<LynxDatagram> dropAck = d -> false;
        volatile Function<LynxDatagram, Integer> nack = d -> null;
        volatile boolean attention = false;
        volatile Throwable throwNext;
        volatile Consumer<LynxDatagram> onWrite = d -> {};
        final AtomicInteger hubMessage = new AtomicInteger(1);
        volatile int i2cWriteRead = -1;
        volatile int i2cReadStatus = -1;
        final Map<Integer, int[]> i2cPending = new ConcurrentHashMap<>();
        final AtomicInteger i2cTransactions = new AtomicInteger();
        final AtomicInteger i2cOverlaps = new AtomicInteger();
        SerialNumber serial;

        @Override
        public Object invoke(Object proxy, Method m, Object[] args) throws Throwable {
            switch (m.getName()) {
                case "write": {
                    Throwable t = throwNext;
                    if (t != null) { throwNext = null; throw t; }
                    LynxDatagram d = new LynxDatagram();
                    d.fromByteArray((byte[]) args[0]);
                    written.add(d);
                    onWrite.accept(d);
                    if (!dropAck.test(d)) poller.schedule(() -> respond(d), ackDelayMs, TimeUnit.MILLISECONDS);
                    return null;
                }
                case "isOpen": case "isAttached": return true;
                case "getSerialNumber": return serial;
                case "toString": return "FakeHub";
                case "hashCode": return System.identityHashCode(proxy);
                case "equals": return proxy == args[0];
                default:
                    Class<?> r = m.getReturnType();
                    if (r == boolean.class) return false;
                    if (r == int.class) return 0;
                    if (r == long.class) return 0L;
                    return null;
            }
        }

        void respond(LynxDatagram cmd) {
            LynxDatagram r = new LynxDatagram();
            r.setSourceModuleAddress(cmd.getDestModuleAddress());
            r.setMessageNumber(1 + (hubMessage.getAndIncrement() % 254));
            r.setReferenceNumber(cmd.getMessageNumber());
            Integer reason = nack.apply(cmd);
            int command = cmd.getCommandNumber();
            int[] result = null;
            if (reason == null && command == i2cWriteRead) {
                byte[] p = cmd.getPayloadData();
                i2cTransactions.incrementAndGet();
                // One I2C engine per hub, all buses: a second transaction begun before the first is read back overlaps it.
                if (i2cPending.put(cmd.getDestModuleAddress(), new int[] {p[3] & 0xFF, p[2] & 0xFF, 1}) != null) {
                    i2cOverlaps.incrementAndGet();
                }
            } else if (reason == null && command == i2cReadStatus) {
                int[] pending = i2cPending.get(cmd.getDestModuleAddress());
                if (pending == null) {
                    reason = LynxNack.StandardReasonCode.I2C_NO_RESULTS_PENDING.getValue();
                } else if (pending[2] > 0) {
                    pending[2]--;
                    reason = LynxNack.StandardReasonCode.I2C_OPERATION_IN_PROGRESS.getValue();
                } else {
                    i2cPending.remove(cmd.getDestModuleAddress());
                    result = pending;
                }
            }
            if (reason != null) {
                r.setPacketId(LynxNack.getStandardCommandNumber());
                r.setPayloadData(new byte[] {(byte) (int) reason});
            } else if (result != null) {
                byte[] data = i2cBytes(result[0], result[1]);
                byte[] payload = new byte[2 + data.length];
                payload[1] = (byte) data.length;
                System.arraycopy(data, 0, payload, 2, data.length);
                r.setPacketId(command | 0x8000);
                r.setPayloadData(payload);
            } else if (command == LynxGetModuleStatusCommand.getStandardCommandNumber()) {
                r.setPacketId(cmd.getCommandNumber() | 0x8000);
                r.setPayloadData(new byte[] {0, 0});
            } else {
                r.setPacketId(LynxAck.getStandardCommandNumber());
                r.setPayloadData(new byte[] {(byte) (attention ? 1 : 0)});
            }
            LynxModule module = impl.findKnownModule(cmd.getDestModuleAddress());
            if (module != null) module.onIncomingDatagramReceived(r);
        }

        long count(int messageNumber, int commandNumber) {
            return written.stream().filter(d -> d.getMessageNumber() == messageNumber && d.getCommandNumber() == commandNumber).count();
        }
    }

    static class Wrapper implements HardwareDevice {
        LynxModule module;
        Wrapper(LynxModule m) { module = m; }
        public Manufacturer getManufacturer() { return Manufacturer.Other; }
        public String getDeviceName() { return "wrapper"; }
        public String getConnectionInfo() { return ""; }
        public int getVersion() { return 1; }
        public void resetDeviceConfigurationForOpMode() {}
        public void close() {}
    }

    static final class TestDevice extends Wrapper {
        private LynxModule hub;
        RobotCoreLynxModule asInterface;
        Wrapper inner;
        TestDevice(LynxModule m) { super(m); hub = m; asInterface = m; inner = new Wrapper(m); }
        LynxModule hub() { return hub; }
    }

    public static class PlainOpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
        @Override public void init() {}
        @Override public void loop() {}
    }

    @org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon
    public static class AnnotatedOpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
        @Override public void init() {}
        @Override public void loop() {}
    }

    static final class FinalDevice extends Wrapper {
        final LynxModule pinnedHub;
        FinalDevice(LynxModule m) { super(m); pinnedHub = m; }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    static void registerDeka(LynxModule module) throws Exception {
        LynxInterface deka = LynxDekaInterfaceCommand.createDekaInterface();
        deka.setWasNacked(false);
        deka.setBaseCommandNumber(0x1000);
        int i = 0;
        for (Class<? extends LynxInterfaceCommand> c : deka.getCommandClasses()) {
            if (c != null) {
                LynxModule.MessageClassAndCtor pair = new LynxModule.MessageClassAndCtor();
                pair.clazz = c;
                pair.assignCtor();
                module.commandClasses.put(0x1000 + i, pair);
                module.supportedCommands.add((Class) c);
            }
            i++;
        }
        module.interfacesQueried.put("DEKA", deka);
    }

    static final class Rig {
        final FakeHub hub = new FakeHub();
        final LynxUsbDeviceImpl impl;
        final List<LynxModule> originals = new ArrayList<>();
        final HardwareMap map = new HardwareMap(null, null);
        final List<TestDevice> devices = new ArrayList<>();

        Rig(int[] addresses, boolean[] parents) throws Exception {
            hub.serial = SerialNumber.fromString("DQTEST" + RIGS.incrementAndGet());
            RobotUsbDevice usb = (RobotUsbDevice) Proxy.newProxyInstance(PhotonHarness.class.getClassLoader(),
                    new Class<?>[] {RobotUsbDevice.class}, hub);
            impl = (LynxUsbDeviceImpl) UNSAFE.allocateInstance(LynxUsbDeviceImpl.class);
            set(impl, ArmableUsbDevice.class, "armingState", RobotArmingStateNotifier.ARMINGSTATE.ARMED);
            set(impl, ArmableUsbDevice.class, "robotUsbDevice", usb);
            set(impl, ArmableUsbDevice.class, "registeredCallbacks", new WeakReferenceSet<>());
            set(impl, ArmableUsbDevice.class, "serialNumber", hub.serial);
            set(impl, ArmableUsbDevice.class, "armingLock", new Object());
            set(impl, ArmableUsbDevice.class, "warningMessageLock", new Object());
            set(impl, ArmableUsbDevice.class, "warningMessage", "");
            set(impl, LynxUsbDeviceImpl.class, "knownModules", new ConcurrentHashMap<>());
            set(impl, LynxUsbDeviceImpl.class, "knownModulesChanging", new ConcurrentHashMap<>());
            set(impl, LynxUsbDeviceImpl.class, "discoveredModules", new ConcurrentHashMap<>());
            set(impl, LynxUsbDeviceImpl.class, "missingModules", new ConcurrentHashMap<>());
            set(impl, LynxUsbDeviceImpl.class, "networkTransmissionLock", new MessageKeyedLock("lynx xmit lock", 500));
            set(impl, LynxUsbDeviceImpl.class, "engageLock", new Object());
            set(impl, LynxUsbDeviceImpl.class, "isEngaged", true);
            set(impl, LynxUsbDeviceImpl.class, "sysOpStartStopLock", new Object());
            set(impl, LynxUsbDeviceImpl.class, "runningSysOpTrackers", new HashSet<>());
            hub.impl = impl;
            for (int i = 0; i < addresses.length; i++) {
                LynxModule m = new LynxModule(impl, addresses[i], parents[i], false);
                registerDeka(m);
                impl.knownModules.put(addresses[i], m);
                originals.add(m);
                map.put(m.getModuleSerialNumber(), "Hub " + addresses[i], m);
                TestDevice d = new TestDevice(m);
                devices.add(d);
                map.put("device " + addresses[i], d);
            }
        }

        Rig(int address) throws Exception { this(new int[] {address}, new boolean[] {true}); }

        List<String> takeOver() throws Exception {
            List<String> problems = new ArrayList<>();
            call(core(), PhotonCore.class, "takeOver", new Class<?>[] {HardwareMap.class, List.class}, map, problems);
            call(core(), PhotonCore.class, "startSweeperIfStopped", new Class<?>[0]);
            return problems;
        }

        PhotonLynxModule photon(int address) {
            return (PhotonLynxModule) hubAt(address);
        }

        LynxModule hubAt(int address) {
            for (HardwareDevice d : map.unsafeIterable()) {
                if (d instanceof LynxModule && ((LynxModule) d).getModuleAddress() == address) return (LynxModule) d;
            }
            return null;
        }
    }

    static Map<?, ?> routes(PhotonLynxModule p) throws Exception { return (Map<?, ?>) get(p, PhotonLynxModule.class, "routes"); }

    static List<String> photonLines() {
        List<String> out = new ArrayList<>();
        for (String l : Log.LINES) if (l.contains("/Photon:")) out.add(l);
        return out;
    }

    static long clashes(PhotonCore.Counters c) {
        try {
            return c.getClass().getField("numberClashes").getLong(c);
        } catch (NoSuchFieldException e) {
            return -1;
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }

    @SuppressWarnings("unchecked")
    static Map<String, Map<String, Double>> profileOf(LynxModule hub) {
        try {
            return (Map<String, Map<String, Double>>) PhotonCore.class.getMethod("writeProfile", LynxModule.class).invoke(null, hub);
        } catch (NoSuchMethodException e) {
            return null;
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException(e);
        }
    }

    static void profiling(boolean on) {
        try {
            PhotonCore.class.getMethod("setWriteProfiling", boolean.class).invoke(null, on);
        } catch (NoSuchMethodException e) {
            return;
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException(e);
        }
    }

    static double profileCount(Map<String, Map<String, Double>> profile, String step) {
        if (profile == null || profile.get(step) == null) return -1;
        return profile.get(step).get("count");
    }

    static boolean logged(String fragment) {
        for (String l : Log.LINES) if (l.contains(fragment)) return true;
        return false;
    }

    static void power(LynxModule m, int channel, int power) throws Exception {
        new LynxSetMotorConstantPowerCommand(m, channel, power).send();
    }

    static byte[] i2cBytes(int start, int length) {
        byte[] b = new byte[length];
        for (int i = 0; i < length; i++) b[i] = (byte) (start + i + 1);
        return b;
    }

    static final class I2cHolder extends Wrapper {
        I2cDeviceSynchSimple deviceClient;
        I2cHolder(I2cDeviceSynchSimple client) { super(null); deviceClient = client; }
    }

    static LynxI2cDeviceSynchV2 i2cClient(LynxModule module, int bus) {
        LynxI2cDeviceSynchV2 client = new LynxI2cDeviceSynchV2(null, module, bus);
        client.setI2cAddress(I2cAddr.create7bit(0x52));
        return client;
    }

    static final class I2cReader extends Thread {
        final LynxI2cDeviceSynchV2 client;
        final int register;
        final int reads;
        final AtomicInteger wrong = new AtomicInteger();
        final AtomicInteger zeros = new AtomicInteger();
        volatile Throwable failure;

        I2cReader(LynxI2cDeviceSynchV2 client, int register, int reads) {
            super("i2c reader 0x" + Integer.toHexString(register));
            this.client = client;
            this.register = register;
            this.reads = reads;
        }

        @Override public void run() {
            try {
                byte[] expected = i2cBytes(register, 4);
                for (int i = 0; i < reads; i++) {
                    byte[] got = client.read(register, 4);
                    if (Arrays.equals(got, new byte[4])) zeros.incrementAndGet();
                    else if (!Arrays.equals(got, expected)) wrong.incrementAndGet();
                }
            } catch (Throwable t) {
                failure = t;
            }
        }
    }

    static LynxDatagram reply(int address, int reference, int packetId, byte[] payload) {
        LynxDatagram r = new LynxDatagram();
        r.setSourceModuleAddress(address);
        r.setMessageNumber(1);
        r.setReferenceNumber(reference);
        r.setPacketId(packetId);
        r.setPayloadData(payload);
        return r;
    }

    // True when second's I2C lock waits while first holds its own, then gets in once first lets go.
    static boolean excludes(LynxModuleIntf first, LynxModuleIntf second) throws Exception {
        CountDownLatch held = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicBoolean entered = new AtomicBoolean();
        List<Throwable> errors = new CopyOnWriteArrayList<>();
        Thread holder = new Thread(() -> {
            try {
                first.acquireI2cLockWhile(() -> { held.countDown(); release.await(); return null; });
            } catch (Throwable t) {
                errors.add(t);
            }
        });
        holder.start();
        if (!held.await(1, TimeUnit.SECONDS)) return false;
        Thread contender = new Thread(() -> {
            try {
                second.acquireI2cLockWhile(() -> { entered.set(true); return null; });
            } catch (Throwable t) {
                errors.add(t);
            }
        });
        contender.start();
        contender.join(100);
        boolean shutOut = !entered.get();
        release.countDown();
        holder.join(1000);
        contender.join(1000);
        return shutOut && entered.get() && !holder.isAlive() && !contender.isAlive() && errors.isEmpty();
    }

    public static void main(String[] args) {
        try {
            run();
        } catch (Throwable t) {
            t.printStackTrace(System.out);
            failures++;
        }
        System.out.println(passes + " passed, " + failures + " failed");
        System.exit(failures == 0 ? 0 : 1);
    }

    static void run() throws Exception {
        Field u = sun.misc.Unsafe.class.getDeclaredField("theUnsafe");
        u.setAccessible(true);
        UNSAFE = (sun.misc.Unsafe) u.get(null);
        prevent(false);

        check(PhotonLynxModule.class.getDeclaredMethods().length > 0, "PhotonLynxModule loads against SDK 12 classes");
        check(call(null, PhotonLynxModule.class, "unhandledFields", new Class<?>[0]) == null,
                "SDK 12 LynxModule fields are exactly the ones the transplant handles");

        // Take-over
        Rig rig = new Rig(2);
        LynxModule original = rig.originals.get(0);
        EmbeddedControlHubModule.set(original);
        List<String> problems = rig.takeOver();
        check(problems.isEmpty(), "take-over reports no problems " + problems);
        PhotonLynxModule photon = rig.photon(2);
        check(photon != null && PhotonCore.isSwapped(photon), "hardware map hub is Photon's");
        check(rig.impl.knownModules.get(2) == photon, "replies route to Photon's module");
        TestDevice dev = rig.devices.get(0);
        check(dev.hub() == photon && dev.module == photon && dev.asInterface == photon && dev.inner.module == photon,
                "device fields re-pointed, including through a wrapper and an interface-typed field");
        check(EmbeddedControlHubModule.get() == photon, "embedded Control Hub module re-pointed");
        check(map(rig).get(photon.getModuleSerialNumber()) == photon, "serial-number lookup returns Photon's module");
        check(photon.commandClasses.size() == original.commandClasses.size() && photon.getInterface("DEKA") != null,
                "DEKA interface carried over");
        check(rig.takeOver().isEmpty() && rig.photon(2) == photon, "second INIT on the same map keeps the same module");

        PhotonCore.resetCounters();

        // Disabled: stock path
        enablePhoton(false);
        long t0 = System.nanoTime();
        for (int i = 0; i < 5; i++) power(photon, 0, 1000 + i);
        check(PhotonCore.counters().parallelWrites == 0 && rig.hub.written.size() >= 5, "disabled: every write takes the SDK path");
        check(routes(photon).isEmpty(), "disabled: no routes left behind");

        // Enabled: parallel
        enablePhoton(true);
        rig.hub.ackDelayMs = 20;
        int before = rig.hub.written.size();
        t0 = System.nanoTime();
        for (int i = 0; i < 6; i++) power(photon, i % 4, 2000 + i);
        double ms = (System.nanoTime() - t0) / 1e6;
        int inflight = PhotonCore.inFlight(photon);
        check(ms < 15, String.format("enabled: 6 writes return before their 20 ms ACKs (%.1f ms)", ms));
        check(inflight == 6, "enabled: 6 writes in flight right after (" + inflight + ")");
        sleep(80);
        check(PhotonCore.inFlight(photon) == 0 && photon.unfinishedCommands.isEmpty(), "enabled: all in-flight writes drained by their real ACKs");
        check(PhotonCore.counters().parallelWrites == 6 && PhotonCore.counters().retransmits == 0, "enabled: 6 parallel writes, no retransmits");
        check(rig.hub.written.size() - before == 6, "enabled: each write sent once");
        check(routes(photon).isEmpty(), "enabled: no routes left behind");

        // Slot limit and late lock
        PhotonCore.resetCounters();
        PhotonCore.setMaxParallelWrites(2);
        rig.hub.ackDelayMs = 40;
        AtomicInteger maxSeen = new AtomicInteger();
        rig.hub.onWrite = d -> maxSeen.accumulateAndGet(PhotonCore.inFlight(photon), Math::max);
        t0 = System.nanoTime();
        power(photon, 0, 100);
        power(photon, 1, 100);
        power(photon, 2, 100);
        ms = (System.nanoTime() - t0) / 1e6;
        check(PhotonCore.counters().slotTimeouts == 1, "slot limit: third write falls back after the bounded wait");
        check(maxSeen.get() <= 2, "slot limit: never more than 2 in flight (" + maxSeen.get() + ")");
        check(ms >= 35 && ms < 150, String.format("slot limit: the fallback waits for its own ACK the SDK way (%.1f ms)", ms));
        sleep(60);
        rig.hub.ackDelayMs = 2;
        enablePhoton(false);
        t0 = System.nanoTime();
        power(photon, 3, 100);
        ms = (System.nanoTime() - t0) / 1e6;
        check(ms < 100 && !logged("abandoning lock"), String.format("slot limit: the late-locked fallback released the network lock (%.1f ms)", ms));
        enablePhoton(true);
        rig.hub.onWrite = d -> {};
        PhotonCore.setMaxParallelWrites(8);

        // Lost ACK: retransmit then abandon
        PhotonCore.resetCounters();
        int motorCmd = new LynxSetMotorConstantPowerCommand(photon, 3, 1).getCommandNumber();
        rig.hub.dropAck = d -> d.getCommandNumber() == motorCmd && d.getPayloadData()[0] == 3;
        int unresponsiveBefore = LynxModuleWarningManager.UNRESPONSIVE.get();
        power(photon, 3, 3000);
        int msg = rig.hub.written.get(rig.hub.written.size() - 1).getMessageNumber();
        sleep(400);
        check(rig.hub.count(msg, motorCmd) == 3, "lost ACK: write sent 3 times (initial + 2 retransmits), got " + rig.hub.count(msg, motorCmd));
        check(PhotonCore.counters().retransmits == 2 && PhotonCore.counters().abandonedWrites == 1, "lost ACK: counted 2 retransmits and 1 abandoned");
        check(photon.isNotResponding(), "lost ACK: hub marked not responding like the SDK timeout");
        check(logged("timeout: abandoning waiting 250ms for ack: cmd=LynxSetMotorConstantPowerCommand mod=2 msg#=" + msg)
               ,
                "lost ACK: logged exactly like the SDK timeout");
        check(LynxModuleWarningManager.UNRESPONSIVE.get() == unresponsiveBefore + 1, "lost ACK: reported to the warning manager once, like the SDK timeout");
        check(PhotonCore.inFlight(photon) == 0 && !photon.unfinishedCommands.containsKey(msg), "lost ACK: entry removed");
        rig.hub.dropAck = d -> false;
        boolean nacked = false;
        try {
            power(photon, 0, 500);
        } catch (LynxNackException e) {
            nacked = true;
        }
        check(nacked && PhotonCore.counters().notRespondingFallbacks == 1, "not responding: next write takes the SDK path, which NACKs it at once");
        sleep(30);
        check(!photon.isNotResponding(), "not responding: cleared by the hub's next reply to Photon's module");

        // Superseded write is never retransmitted
        PhotonCore.resetCounters();
        AtomicInteger firstSeen = new AtomicInteger(-1);
        rig.hub.dropAck = d -> d.getCommandNumber() == motorCmd && d.getPayloadData()[0] == 1 && firstSeen.compareAndSet(-1, d.getMessageNumber())
                || d.getMessageNumber() == firstSeen.get();
        power(photon, 1, 1111);
        int stale = firstSeen.get();
        sleep(10);
        power(photon, 1, 2222);
        sleep(320);
        check(rig.hub.count(stale, motorCmd) == 1, "superseded: the older write is never retransmitted over the newer one");
        check(PhotonCore.counters().supersededWrites == 1 && PhotonCore.counters().abandonedWrites == 0 && !photon.isNotResponding(),
                "superseded: finished quietly while the hub kept answering");
        rig.hub.dropAck = d -> false;

        // Attention bit from the real ACK, not the fake one
        photon.attentionRequiredPreviously = true;
        rig.hub.ackDelayMs = 30;
        rig.hub.attention = false;
        power(photon, 0, 10);
        check(photon.attentionRequiredPreviously, "attention: the fake ACK leaves the module's attention state alone");
        sleep(80);
        check(!photon.attentionRequiredPreviously, "attention: the real ACK's bit is applied");
        rig.hub.attention = true;
        power(photon, 0, 11);
        sleep(80);
        check(photon.attentionRequiredPreviously, "attention: a real ACK with attention set reaches the module");
        rig.hub.attention = false;
        power(photon, 0, 12);
        sleep(150);
        rig.hub.ackDelayMs = 2;

        // NACK of a parallel write
        PhotonCore.resetCounters();
        RobotLog.clearGlobalWarningMsg();
        rig.hub.nack = d -> d.getCommandNumber() == motorCmd && d.getPayloadData()[0] == 2 ? 52 : null;
        power(photon, 2, 4000);
        sleep(30);
        Map<String, Long> nacks = PhotonCore.counters().nacks;
        check(Long.valueOf(1).equals(nacks.get("BATTERY_TOO_LOW_TO_RUN_MOTOR")), "NACK: counted by reason " + nacks);
        check(RobotLog.hasGlobalWarningMsg(), "NACK: battery-too-low raises a global warning");
        check(PhotonCore.inFlight(photon) == 0, "NACK: entry finished");
        rig.hub.nack = d -> null;
        RobotLog.clearGlobalWarningMsg();

        // STOP gate
        PhotonCore.resetCounters();
        prevent(true);
        before = rig.hub.written.size();
        boolean cancelled = false;
        try {
            power(photon, 0, 5000);
        } catch (LynxNackException e) {
            cancelled = e.getNack().getNackReasonCodeAsEnum() == LynxNack.StandardReasonCode.CANCELLED_FOR_SAFETY;
        }
        check(cancelled && rig.hub.written.size() == before && PhotonCore.counters().gateFallbacks == 1,
                "STOP gate: a dangerous write takes the SDK path and is refused, nothing sent");
        power(photon, 0, 0);
        check(PhotonCore.counters().parallelWrites == 1, "STOP gate: a zero-power write still goes parallel");
        prevent(false);

        // STOP landing during a parallel write
        PhotonCore.resetCounters();
        sleep(20);
        rig.hub.onWrite = d -> {
            if (d.getCommandNumber() == motorCmd && d.getPayloadData()[1] != 0) {
                try { prevent(true); } catch (Exception e) { throw new RuntimeException(e); }
            }
        };
        before = rig.hub.written.size();
        boolean raceNacked = false;
        try {
            power(photon, 1, 6000);
        } catch (LynxNackException e) {
            raceNacked = e.getNack().getNackReasonCodeAsEnum() == LynxNack.StandardReasonCode.CANCELLED_FOR_SAFETY;
        }
        sleep(20);
        boolean failSafeAfter = false;
        for (int i = before; i < rig.hub.written.size(); i++) {
            if (rig.hub.written.get(i).getCommandNumber() == LynxFailSafeCommand.getStandardCommandNumber()) failSafeAfter = true;
        }
        check(failSafeAfter && PhotonCore.counters().postStopFailSafes == 1, "STOP race: the hub is fail-safed right after the write");
        check(raceNacked, "STOP race: send() ends in the SDK gate's CANCELLED_FOR_SAFETY NACK, so the motor enable after it is skipped");
        check(PhotonCore.inFlight(photon) == 0 && photon.unfinishedCommands.isEmpty(), "STOP race: the stand-in still finishes on the real ACK");
        rig.hub.onWrite = d -> {};
        prevent(false);

        // STOP landing between the route decision and the write: the check under the device's write lock catches it
        PhotonCore.resetCounters();
        Object engageLock = get(rig.impl, LynxUsbDeviceImpl.class, "engageLock");
        Thread stopper = new Thread(() -> {
            synchronized (engageLock) {
                try { sleep(30); prevent(true); } catch (Exception e) { throw new RuntimeException(e); }
            }
        });
        before = rig.hub.written.size();
        stopper.start();
        sleep(5);
        cancelled = false;
        try {
            power(photon, 2, 6100);
        } catch (LynxNackException e) {
            cancelled = e.getNack().getNackReasonCodeAsEnum() == LynxNack.StandardReasonCode.CANCELLED_FOR_SAFETY;
        }
        stopper.join();
        check(cancelled && rig.hub.written.size() == before && PhotonCore.counters().gateFallbacks == 1
                        && PhotonCore.counters().parallelWrites == 0 && PhotonCore.inFlight(photon) == 0,
                "STOP between decision and write: refused the SDK way, nothing sent");
        prevent(false);

        // A dangerous write is never retransmitted once STOP has begun
        PhotonCore.resetCounters();
        rig.hub.dropAck = d -> d.getCommandNumber() == motorCmd && d.getPayloadData()[0] == 0;
        power(photon, 0, 6200);
        int gated = rig.hub.written.get(rig.hub.written.size() - 1).getMessageNumber();
        sleep(40);
        prevent(true);
        photon.failSafe();
        sleep(300);
        check(rig.hub.count(gated, motorCmd) == 1 && PhotonCore.counters().retransmits == 0,
                "after STOP: a pending dangerous write is never retransmitted");
        check(PhotonCore.counters().supersededWrites == 1 && PhotonCore.counters().abandonedWrites == 0 && PhotonCore.inFlight(photon) == 0,
                "after STOP: it finishes quietly once the fail-safe is acknowledged");
        rig.hub.dropAck = d -> false;
        prevent(false);

        // Device not ready
        PhotonCore.resetCounters();
        set(rig.impl, ArmableUsbDevice.class, "armingState", RobotArmingStateNotifier.ARMINGSTATE.DISARMED);
        before = rig.hub.written.size();
        power(photon, 0, 700);
        check(PhotonCore.counters().deviceNotReadyFallbacks == 1 && rig.hub.written.size() == before,
                "device not ready: SDK path pretends, nothing written to the stale device");
        set(rig.impl, ArmableUsbDevice.class, "armingState", RobotArmingStateNotifier.ARMINGSTATE.ARMED);
        Object liveUsb = get(rig.impl, ArmableUsbDevice.class, "robotUsbDevice");
        Object[][] notReady = {
                {"shut down abnormally", LynxUsbDeviceImpl.class, "hasShutdownAbnormally", true, false},
                {"disengaged", LynxUsbDeviceImpl.class, "isEngaged", false, true},
                {"no USB device", ArmableUsbDevice.class, "robotUsbDevice", null, liveUsb}};
        for (Object[] c : notReady) {
            sleep(20);
            PhotonCore.resetCounters();
            set(rig.impl, (Class<?>) c[1], (String) c[2], c[3]);
            before = rig.hub.written.size();
            Throwable notReadyError = null;
            try {
                power(photon, 0, 710);
            } catch (Throwable t) {
                notReadyError = t;
            } finally {
                set(rig.impl, (Class<?>) c[1], (String) c[2], c[4]);
            }
            check(notReadyError == null && PhotonCore.counters().deviceNotReadyFallbacks == 1 && PhotonCore.counters().parallelWrites == 0
                            && rig.hub.written.size() == before && PhotonCore.inFlight(photon) == 0,
                    "device not ready: " + c[0] + ", the SDK path pretends and nothing is written ("
                            + PhotonCore.counters().deviceNotReadyFallbacks + " fallbacks, " + (rig.hub.written.size() - before)
                            + " written) " + (notReadyError == null ? "" : notReadyError));
        }

        // Write failure
        PhotonCore.resetCounters();
        rig.hub.throwNext = new RobotUsbUnspecifiedException("injected write failure");
        before = rig.hub.written.size();
        int linesBefore = photonLines().size();
        List<Boolean> heldWhileLogging = new CopyOnWriteArrayList<>();
        Object failEngageLock = get(rig.impl, LynxUsbDeviceImpl.class, "engageLock");
        Log.hook = l -> { if (l.contains("/Photon:")) heldWhileLogging.add(Thread.holdsLock(failEngageLock)); };
        power(photon, 0, 800);
        Log.hook = null;
        check(PhotonCore.counters().writeFailures == 1 && rig.hub.written.size() == before + 1 && PhotonCore.inFlight(photon) == 0,
                "write failure: entry removed, then sent once the SDK way");
        List<String> failLines = photonLines().subList(linesBefore, photonLines().size());
        check(failLines.size() == 1 && failLines.get(0).split("\n").length == 1
                        && failLines.get(0).contains("RobotUsbUnspecifiedException") && failLines.get(0).contains("injected write failure"),
                "write failure: logged as one line naming the exception, no stack trace (" + failLines.size() + " Photon lines, "
                        + (failLines.isEmpty() ? 0 : failLines.get(0).split("\n").length) + " text lines)");
        check(!heldWhileLogging.isEmpty() && !heldWhileLogging.contains(Boolean.TRUE),
                "write failure: logged after the USB device's write lock is released " + heldWhileLogging);

        // Leftovers cleared at INIT
        PhotonCore.resetCounters();
        rig.hub.dropAck = d -> true;
        power(photon, 0, 1);
        power(photon, 1, 1);
        power(photon, 2, 1);
        check(PhotonCore.inFlight(photon) == 3, "INIT clear: 3 unacknowledged writes in flight");
        call(core(), PhotonCore.class, "onOpModePreInit", new Class<?>[] {com.qualcomm.robotcore.eventloop.opmode.OpMode.class}, (Object) null);
        check(PhotonCore.inFlight(photon) == 0 && PhotonCore.counters().clearedAtInit == 3 && photon.unfinishedCommands.isEmpty(),
                "INIT clear: all dropped at the next INIT");
        check(!PhotonCore.isEnabled(), "INIT clear: INIT disables Photon");
        sleep(300);
        check(PhotonCore.counters().abandonedWrites == 0, "INIT clear: cleared writes are never abandoned later");
        rig.hub.dropAck = d -> false;
        enablePhoton(true);

        // Interrupt while waiting for a slot
        PhotonCore.setMaxParallelWrites(1);
        rig.hub.dropAck = d -> true;
        power(photon, 0, 2);
        Thread.currentThread().interrupt();
        boolean interrupted = false;
        try {
            power(photon, 1, 2);
        } catch (InterruptedException e) {
            interrupted = true;
        }
        check(interrupted && routes(photon).isEmpty(), "interrupt: the slot wait honors it and leaves no route");
        PhotonCore.setMaxParallelWrites(8);
        rig.hub.dropAck = d -> false;
        call(core(), PhotonCore.class, "onOpModePreInit", new Class<?>[] {com.qualcomm.robotcore.eventloop.opmode.OpMode.class}, (Object) null);
        enablePhoton(true);

        // Orphan traffic still gets its replies
        PhotonCore.resetCounters();
        t0 = System.nanoTime();
        power(original, 0, 900);
        ms = (System.nanoTime() - t0) / 1e6;
        check(ms < 100 && PhotonCore.counters().orphanDatagrams == 1, String.format("orphan: a command sent through the replaced module gets its reply (%.1f ms)", ms));

        // The replaced module's not-responding flag follows Photon's, for SDK objects that kept the replaced one
        photon.noteNotResponding();
        check(original.isNotResponding(), "orphan flag: noteNotResponding on Photon's module marks the replaced one too");
        boolean flaggedNack = false;
        try {
            power(photon, 0, 901);
        } catch (LynxNackException e) {
            flaggedNack = true;
        }
        sleep(20);
        check(flaggedNack && !photon.isNotResponding() && !original.isNotResponding(),
                "orphan flag: the next reply to Photon's module clears both");
        original.noteNotResponding();
        power(photon, 0, 902);
        sleep(20);
        check(!original.isNotResponding(), "orphan flag: a flag set on the replaced module alone clears on Photon's next reply");

        // A keep-alive restarted on the replaced module is stopped by the sweeper
        boolean armed;
        // Under the ping lock, or the sweeper can cancel it before it's seen armed.
        synchronized (get(original, LynxModule.class, "pingFutureLock")) {
            call(original, LynxModule.class, "startPingTimer", new Class<?>[0]);
            armed = get(original, LynxModule.class, "pingFuture") != null;
        }
        sleep(30);
        Object stillArmed = get(original, LynxModule.class, "pingFuture");
        check(armed && stillArmed == null,
                "orphan ping: a re-armed keep-alive on the replaced module is cancelled within a sweep (armed " + armed
                        + ", still armed after 30 ms " + (stillArmed != null) + ")");
        check(get(photon, LynxModule.class, "pingFuture") != null, "orphan ping: Photon's own keep-alive stays armed");

        // Many threads
        PhotonCore.resetCounters();
        List<Thread> threads = new ArrayList<>();
        List<Throwable> errors = new CopyOnWriteArrayList<>();
        rig.hub.ackDelayMs = 1;
        for (int t = 0; t < 4; t++) {
            final int channel = t;
            Thread th = new Thread(() -> {
                try {
                    for (int i = 0; i < 200; i++) {
                        if (channel < 2) power(photon, channel, 100 + i);
                        else new LynxSetServoPulseWidthCommand(photon, channel, 1000 + i).send();
                    }
                } catch (Throwable e) {
                    errors.add(e);
                }
            });
            threads.add(th);
            th.start();
        }
        for (Thread th : threads) th.join();
        sleep(300);
        PhotonCore.Counters c = PhotonCore.counters();
        check(errors.isEmpty(), "threads: no errors " + errors);
        check(c.parallelWrites + c.stockFallbacks() + c.gateFallbacks == 800, "threads: every write accounted for (" + c.parallelWrites + " parallel, " + c.stockFallbacks() + " fallbacks)");
        check(PhotonCore.inFlight(photon) == 0 && photon.unfinishedCommands.isEmpty() && routes(photon).isEmpty() && c.abandonedWrites == 0,
                "threads: nothing leaked");

        // Keep-alive: Photon resets the ping timer at most every 100 ms, and the hub still never goes 1950 ms unheard
        Field executorField = field(LynxModule.class, "executor");
        ScheduledExecutorService realExecutor = (ScheduledExecutorService) executorField.get(photon);
        AtomicInteger schedules = new AtomicInteger();
        executorField.set(photon, Proxy.newProxyInstance(PhotonHarness.class.getClassLoader(), new Class<?>[] {ScheduledExecutorService.class},
                (proxy, m, a) -> {
                    if (m.getName().equals("schedule")) schedules.incrementAndGet();
                    try {
                        return m.invoke(realExecutor, a);
                    } catch (InvocationTargetException e) {
                        throw e.getCause();
                    }
                }));
        enablePhoton(true);
        rig.hub.ackDelayMs = 1;
        int keepAliveCmd = new LynxKeepAliveCommand(photon, false).getCommandNumber();
        List<Long> keepAlives = new CopyOnWriteArrayList<>();
        rig.hub.onWrite = d -> { if (d.getCommandNumber() == keepAliveCmd) keepAlives.add(System.nanoTime()); };
        sleep(20);
        PhotonCore.resetCounters();
        long pingT0 = System.nanoTime();
        for (int i = 0; i < 200; i++) {
            power(photon, i % 4, 3000 + i);
            LockSupport.parkNanos(2_000_000L);
        }
        double pingMs = (System.nanoTime() - pingT0) / 1e6;
        int scheduled = schedules.get();
        check(PhotonCore.counters().parallelWrites == 200 && scheduled <= pingMs / 100 + 2,
                String.format("keep-alive: 200 parallel writes over %.0f ms reschedule the ping %d times (at most %d)", pingMs,
                        scheduled, (int) (pingMs / 100 + 2)));
        long steadyStart = System.nanoTime();
        long lastWrite = 0;
        while (System.nanoTime() - steadyStart < 2_400_000_000L) {
            power(photon, 0, 3300 + (int) (lastWrite % 7));
            lastWrite = System.nanoTime();
            sleep(50);
        }
        final long writesEnded = lastWrite;
        long kaDuring = keepAlives.stream().filter(t -> t > steadyStart && t <= writesEnded).count();
        sleep(2100);
        long firstAfter = keepAlives.stream().filter(t -> t > writesEnded).findFirst().orElse(0L);
        double silenceMs = firstAfter == 0 ? Double.NaN : (firstAfter - writesEnded) / 1e6;
        check(kaDuring == 0, "keep-alive: none sent while writes come every 50 ms for 2.4 s (" + kaDuring + ")");
        check(silenceMs >= 1800 && silenceMs <= 1990,
                String.format("keep-alive: the hub hears one 1800-1990 ms after the last write (the SDK's own interval is 1950 ms) (%.0f ms)", silenceMs));
        rig.hub.onWrite = d -> {};
        executorField.set(photon, realExecutor);

        // Per-step write timing: off unless switched on, and every parallel write lands in every step while on
        Map<String, Map<String, Double>> idle = profileOf(photon);
        boolean nothingYet = idle != null;
        if (idle != null) for (Map<String, Double> step : idle.values()) nothingYet &= step.get("count") == 0;
        check(nothingYet, "write profile: off by default, nothing recorded");
        profiling(true);
        for (int i = 0; i < 50; i++) power(photon, i % 4, 3500 + i);
        sleep(40);
        profiling(false);
        Map<String, Map<String, Double>> prof = profileOf(photon);
        boolean everyStep = prof != null;
        List<String> short50 = new ArrayList<>();
        if (prof != null) {
            for (Map.Entry<String, Map<String, Double>> e : prof.entrySet()) {
                if (e.getValue().get("count") != 50) short50.add(e.getKey() + "=" + e.getValue().get("count"));
            }
            everyStep = short50.isEmpty() && prof.size() == 15;
        }
        check(everyStep, "write profile: 50 parallel writes counted once in each of the 15 steps " + short50);
        for (int i = 0; i < 20; i++) power(photon, i % 4, 3600 + i);
        sleep(20);
        check(profileCount(profileOf(photon), "sendParallel") == 50, "write profile: switched off again, nothing more recorded");

        // I2C: one lock per physical hub, whichever module object a client goes through
        Rig i2cRig = new Rig(7);
        LynxModule i2cOriginal = i2cRig.originals.get(0);
        LynxI2cDeviceSynchV2 mapped = i2cClient(i2cOriginal, 2);
        i2cRig.map.put("i2c device", new I2cHolder(mapped));
        check(i2cRig.takeOver().isEmpty(), "I2C: take-over with an I2C device in the map");
        PhotonLynxModule i2cPhoton = i2cRig.photon(7);
        check(i2cPhoton != null && get(mapped, LynxController.class, "module") == i2cPhoton && mapped.getModule() == i2cPhoton,
                "I2C: the mapped device's I2C client is re-pointed at Photon's module");
        Object orphanI2cLock = get(i2cOriginal, LynxModule.class, "i2cLock");
        check(Boolean.TRUE.equals(i2cPhoton.acquireI2cLockWhile(() -> Thread.holdsLock(orphanI2cLock))),
                "I2C lock: a transaction through Photon's module holds the replaced module's lock");
        check(excludes(i2cPhoton, i2cOriginal), "I2C lock: a transaction through Photon's module shuts out one through the replaced module");
        check(excludes(i2cOriginal, i2cPhoton), "I2C lock: a transaction through the replaced module shuts out one through Photon's module");
        AtomicBoolean nestedDone = new AtomicBoolean();
        Thread nester = new Thread(() -> {
            try {
                Object v = i2cPhoton.acquireI2cLockWhile(() -> i2cOriginal.acquireI2cLockWhile(() -> i2cPhoton.acquireI2cLockWhile(() -> "in")));
                nestedDone.set("in".equals(v));
            } catch (Throwable t) {
                t.printStackTrace(System.out);
            }
        });
        nester.setDaemon(true);
        nester.start();
        nester.join(1000);
        check(nestedDone.get(), "I2C lock: nesting it through both module objects on one thread doesn't deadlock");

        PhotonCore.resetCounters();
        enablePhoton(true);
        i2cRig.hub.ackDelayMs = 1;
        i2cRig.hub.i2cWriteRead = new LynxI2cWriteReadMultipleBytesCommand(i2cPhoton, 0, I2cAddr.create7bit(0x52), 0x32, 4).getCommandNumber();
        i2cRig.hub.i2cReadStatus = new LynxI2cReadStatusQueryCommand(i2cPhoton, 0, 4).getCommandNumber();
        LynxI2cDeviceSynchV2 onOrphan = i2cClient(i2cOriginal, 0);
        LynxI2cDeviceSynchV2 onPhoton = i2cClient(i2cPhoton, 0);
        LynxI2cDeviceSynchV2 otherBus = i2cClient(i2cPhoton, 1);
        int i2cReads = 40;
        List<I2cReader> readers = Arrays.asList(new I2cReader(onOrphan, 0x32, i2cReads), new I2cReader(onPhoton, 0x37, i2cReads),
                new I2cReader(otherBus, 0x10, i2cReads));
        List<Throwable> writeErrors = new CopyOnWriteArrayList<>();
        Thread writer = new Thread(() -> {
            try {
                for (int i = 0; i < 300; i++) power(i2cPhoton, i % 4, 300 + i);
            } catch (Throwable e) {
                writeErrors.add(e);
            }
        });
        t0 = System.nanoTime();
        for (I2cReader reader : readers) reader.start();
        writer.start();
        for (I2cReader reader : readers) reader.join(20_000);
        writer.join(20_000);
        ms = (System.nanoTime() - t0) / 1e6;
        sleep(50);
        int wrong = 0, zeros = 0;
        List<Throwable> readErrors = new ArrayList<>();
        for (I2cReader reader : readers) {
            wrong += reader.wrong.get();
            zeros += reader.zeros.get();
            if (reader.failure != null) readErrors.add(reader.failure);
            if (reader.isAlive()) readErrors.add(new IllegalStateException(reader.getName() + " still running"));
        }
        check(readErrors.isEmpty() && i2cRig.hub.i2cTransactions.get() == 3 * i2cReads,
                String.format("I2C clients: %d reads through the replaced module and Photon's (two buses) all reached the hub (%d, %.0f ms) %s",
                        3 * i2cReads, i2cRig.hub.i2cTransactions.get(), ms, readErrors));
        check(i2cRig.hub.i2cOverlaps.get() == 0,
                "I2C clients: no transaction on the hub began before the previous one was read back (" + i2cRig.hub.i2cOverlaps.get() + " overlaps)");
        check(wrong == 0 && zeros == 0, "I2C clients: every read returned its own register's bytes (" + wrong + " wrong, " + zeros + " placeholders)");
        check(PhotonCore.counters().orphanDatagrams == 3 * i2cReads && clashes(PhotonCore.counters()) == 0
                        && i2cOriginal.unfinishedCommands.isEmpty(),
                "I2C clients: every reply to the replaced module's transactions reached it (" + PhotonCore.counters().orphanDatagrams
                        + " of " + 3 * i2cReads + ": ACK, in-progress NACK, data)");
        c = PhotonCore.counters();
        check(writeErrors.isEmpty() && c.parallelWrites > 0 && c.parallelWrites + c.stockFallbacks() + c.gateFallbacks == 300,
                "I2C clients: parallel writes on the same link alongside are all accounted for (" + c.parallelWrites + " parallel, "
                        + c.stockFallbacks() + " fallbacks) " + writeErrors);
        check(PhotonCore.inFlight(i2cPhoton) == 0 && i2cPhoton.unfinishedCommands.isEmpty() && routes(i2cPhoton).isEmpty()
                        && c.abandonedWrites == 0 && i2cRig.hub.i2cPending.isEmpty(),
                "I2C clients: nothing leaked");
        for (I2cDeviceSynchSimple client : new I2cDeviceSynchSimple[] {mapped, onOrphan, onPhoton, otherBus}) {
            I2cWarningManager.removeProblemI2cDevice(client);
        }
        i2cPhoton.close();
        int closedWarnings = 0;
        for (String l : Log.LINES) if (l.contains("Attempted use of a closed LynxModule instance")) closedWarnings++;
        Object afterClose = i2cPhoton.acquireI2cLockWhile(() -> "ran");
        int closedWarningsAfter = 0;
        for (String l : Log.LINES) if (l.contains("Attempted use of a closed LynxModule instance")) closedWarningsAfter++;
        check("ran".equals(afterClose) && closedWarningsAfter == closedWarnings + 1 && i2cOriginal.isOpen(),
                "I2C lock: Photon's module, once closed, still warns like the SDK's and still runs the transaction");

        // Message numbers: the replaced module shares the counter, so Photon must skip the numbers it still waits on
        Rig numRig = new Rig(8);
        LynxModule numOriginal = numRig.originals.get(0);
        check(numRig.takeOver().isEmpty(), "message numbers: take-over");
        PhotonLynxModule numPhoton = numRig.photon(8);
        int[] planted = {5, 77, 200};
        for (int n : planted) {
            LynxSetMotorConstantPowerCommand held = new LynxSetMotorConstantPowerCommand(numOriginal, 0, 1);
            held.setMessageNumber(n);
            numOriginal.unfinishedCommands.put(n, held);
        }
        PhotonCore.resetCounters();
        numRig.hub.ackDelayMs = 1;
        int numStart = numRig.hub.written.size();
        enablePhoton(true);
        for (int i = 0; i < 300; i++) power(numPhoton, i % 4, 400 + i);
        enablePhoton(false);
        int numNacks = 0;
        for (int i = 0; i < 300; i++) {
            try {
                power(numPhoton, i % 4, 700 + i);
            } catch (LynxNackException e) {
                numNacks++;
            }
        }
        sleep(30);
        Set<Integer> usedNumbers = new HashSet<>();
        int reused = 0;
        for (int i = numStart; i < numRig.hub.written.size(); i++) {
            int n = numRig.hub.written.get(i).getMessageNumber();
            usedNumbers.add(n);
            for (int k : planted) if (n == k) reused++;
        }
        PhotonCore.Counters nc = PhotonCore.counters();
        check(reused == 0 && numNacks == 0 && usedNumbers.size() == 255 - planted.length && nc.parallelWrites > 0
                        && nc.parallelWrites + nc.stockFallbacks() == 300,
                "message numbers: 300 parallel and 300 SDK-path writes through Photon's module skip every number pending on "
                        + "the replaced one (" + reused + " reused, " + numNacks + " abandoned, " + usedNumbers.size()
                        + " distinct numbers)");
        for (int n : planted) numOriginal.unfinishedCommands.remove(n);

        int servoCmd = new LynxSetServoPulseWidthCommand(numPhoton, 5, 1).getCommandNumber();
        numRig.hub.dropAck = d -> d.getCommandNumber() == servoCmd && d.getPayloadData()[0] == 5;
        AtomicInteger heldWrites = new AtomicInteger();
        AtomicInteger heldClashes = new AtomicInteger();
        List<Long> heldSends = new CopyOnWriteArrayList<>();
        numRig.hub.onWrite = d -> {
            if (d.getCommandNumber() == servoCmd && d.getPayloadData()[0] == 5) heldSends.add(System.nanoTime());
            if (d.getCommandNumber() == servoCmd || numOriginal.unfinishedCommands.isEmpty()) return;
            heldWrites.incrementAndGet();
            if (numOriginal.unfinishedCommands.containsKey(d.getMessageNumber())) heldClashes.incrementAndGet();
        };
        enablePhoton(true);
        List<Throwable> heldErrors = new CopyOnWriteArrayList<>();
        AtomicBoolean heldAbandoned = new AtomicBoolean();
        AtomicLong heldAbandonedAt = new AtomicLong();
        Thread heldSender = new Thread(() -> {
            try {
                new LynxSetServoPulseWidthCommand(numOriginal, 5, 1500).send();
            } catch (LynxNackException e) {
                heldAbandonedAt.set(System.nanoTime());
                heldAbandoned.set(e.getNack().getNackReasonCodeAsEnum() == LynxNack.StandardReasonCode.ABANDONED_WAITING_FOR_ACK);
            } catch (Throwable t) {
                heldErrors.add(t);
            }
        });
        int heldStart = numRig.hub.written.size();
        heldSender.start();
        for (int i = 0; i < 100 && numOriginal.unfinishedCommands.isEmpty(); i++) sleep(1);
        Thread heldWriter = new Thread(() -> {
            try {
                for (int i = 0; heldSender.isAlive(); i++) power(numPhoton, i % 4, 1000 + (i % 500));
            } catch (Throwable t) {
                heldErrors.add(t);
            }
        });
        heldWriter.start();
        heldSender.join(2000);
        heldWriter.join(2000);
        sleep(30);
        int heldNumber = -1;
        for (int i = heldStart; i < numRig.hub.written.size(); i++) {
            LynxDatagram d = numRig.hub.written.get(i);
            if (d.getCommandNumber() == servoCmd) { heldNumber = d.getMessageNumber(); break; }
        }
        List<Double> heldAt = new ArrayList<>();
        for (long t : heldSends) heldAt.add((t - heldSends.get(0)) / 1e6);
        double heldAbandonMs = heldSends.isEmpty() ? Double.NaN : (heldAbandonedAt.get() - heldSends.get(0)) / 1e6;
        boolean heldTimed = heldAt.size() >= 4 && heldAt.get(1) >= 99 && heldAt.get(1) < 150
                && heldAt.get(2) - heldAt.get(1) >= 99 && heldAt.get(2) - heldAt.get(1) < 150 && heldAbandonMs >= 250 && heldAbandonMs < 300;
        for (int i = 3; i < heldAt.size(); i++) heldTimed &= heldAt.get(i) >= 240 && heldAt.get(i) <= heldAbandonMs;
        check(heldErrors.isEmpty() && heldAbandoned.get() && heldTimed && numRig.hub.count(heldNumber, servoCmd) == heldSends.size(),
                "message numbers: a write through the replaced module whose ACK never comes is held the SDK's 250 ms under "
                        + "one number (sent at 0, ~100 and ~200 ms and in the deadline's last ms, abandoned at 250 ms): sent at "
                        + String.format("%s ms, abandoned at %.1f ms, %d of %d sends under its number ",
                        heldAt.stream().map(x -> String.format("%.1f", x)).collect(java.util.stream.Collectors.toList()), heldAbandonMs,
                        numRig.hub.count(heldNumber, servoCmd), heldSends.size()) + heldErrors);
        check(heldWrites.get() >= 300 && heldClashes.get() == 0,
                "message numbers: Photon's parallel writes meanwhile wrap the counter and never take its number ("
                        + heldWrites.get() + " writes, " + heldClashes.get() + " clashes)");
        numRig.hub.onWrite = d -> {};

        // Every number pending on the replaced module: Photon refuses the write instead of spinning
        for (int n = 1; n <= 255; n++) {
            LynxSetMotorConstantPowerCommand held = new LynxSetMotorConstantPowerCommand(numOriginal, 0, 1);
            held.setMessageNumber(n);
            numOriginal.unfinishedCommands.put(n, held);
        }
        PhotonCore.resetCounters();
        AtomicReference<Throwable> fullParallel = new AtomicReference<>();
        AtomicReference<Throwable> fullStock = new AtomicReference<>();
        Thread fullWriter = new Thread(() -> {
            try {
                enablePhoton(true);
                power(numPhoton, 0, 4321);
            } catch (Throwable t) {
                fullParallel.set(t);
            }
        });
        fullWriter.setDaemon(true);
        fullWriter.start();
        fullWriter.join(1000);
        boolean parallelReturned = !fullWriter.isAlive();
        if (parallelReturned) {
            Thread stockWriter = new Thread(() -> {
                try {
                    enablePhoton(false);
                    power(numPhoton, 0, 4322);
                } catch (Throwable t) {
                    fullStock.set(t);
                }
            });
            stockWriter.setDaemon(true);
            stockWriter.start();
            stockWriter.join(1000);
            parallelReturned = !stockWriter.isAlive();
        }
        for (int n = 1; n <= 255; n++) numOriginal.unfinishedCommands.remove(n);
        check(parallelReturned && refusedForNumbers(fullParallel.get()) && refusedForNumbers(fullStock.get())
                        && PhotonCore.counters().notRespondingFallbacks == 0 && PhotonCore.inFlight(numPhoton) == 0 && routes(numPhoton).isEmpty() && numPhoton.unfinishedCommands.isEmpty(),
                "message numbers: with all 255 pending on the replaced module, a write throws at once on both paths and leaves "
                        + "nothing behind (parallel: " + fullParallel.get() + ", SDK path: " + fullStock.get() + ")");

        // A reply to a number both module objects hold is dropped and counted; each command then times out the SDK's way
        PhotonCore.resetCounters();
        enablePhoton(true);
        sleep(30);
        numRig.hub.dropAck = d -> d.getCommandNumber() == motorCmd && d.getPayloadData()[0] == 2;
        int clashStart = numRig.hub.written.size();
        long clashT0 = System.nanoTime();
        power(numPhoton, 2, 2500);
        int shared = numRig.hub.written.get(clashStart).getMessageNumber();
        LynxI2cReadStatusQueryCommand orphanQuery = new LynxI2cReadStatusQueryCommand(numOriginal, 0, 4);
        orphanQuery.setMessageNumber(shared);
        numOriginal.unfinishedCommands.put(shared, orphanQuery);
        numPhoton.noteNotResponding();
        boolean bothFlagged = numPhoton.isNotResponding() && numOriginal.isNotResponding();
        long heardBefore = (long) get(numPhoton, PhotonLynxModule.class, "lastDatagramNanos");
        numPhoton.onIncomingDatagramReceived(reply(8, shared, LynxNack.getStandardCommandNumber(), new byte[] {41}));
        check(bothFlagged && !numPhoton.isNotResponding() && !numOriginal.isNotResponding()
                        && (long) get(numPhoton, PhotonLynxModule.class, "lastDatagramNanos") - heardBefore > 0,
                "shared number: a dropped reply still counts as hearing from the hub, on both module objects");
        numPhoton.onIncomingDatagramReceived(reply(8, shared, LynxAck.getStandardCommandNumber(), new byte[] {0}));
        numPhoton.onIncomingDatagramReceived(reply(8, shared, orphanQuery.getCommandNumber() | 0x8000, new byte[] {0, 4, 1, 2, 3, 4}));
        PhotonCore.Counters cc = PhotonCore.counters();
        check(!orphanQuery.hasBeenAcknowledged() && PhotonCore.inFlight(numPhoton) == 1
                        && numOriginal.unfinishedCommands.get(shared) == orphanQuery && clashes(cc) == 3 && cc.orphanDatagrams == 0,
                "shared number: a NACK, an ACK and a response to a number both modules hold complete neither command and are "
                        + "counted (" + clashes(cc) + " counted)");
        while (PhotonCore.counters().abandonedWrites == 0 && System.nanoTime() - clashT0 < 1_000_000_000L) sleep(1);
        double clashAbandonMs = (System.nanoTime() - clashT0) / 1e6;
        cc = PhotonCore.counters();
        check(cc.abandonedWrites == 1 && clashAbandonMs >= 250 && clashAbandonMs < 300,
                String.format("shared number: Photon's write is then abandoned like the SDK's own timeout (%.0f ms)", clashAbandonMs));
        numPhoton.onIncomingDatagramReceived(reply(8, shared, orphanQuery.getCommandNumber() | 0x8000, new byte[] {0, 4, 1, 2, 3, 4}));
        cc = PhotonCore.counters();
        check(orphanQuery.isAckOrResponseReceived() && !numOriginal.unfinishedCommands.containsKey(shared)
                        && cc.orphanDatagrams == 1 && clashes(cc) == 3,
                "shared number: the replaced module's query takes the next reply once the number is only its own");
        numRig.hub.dropAck = d -> false;
        enablePhoton(false);
        power(numPhoton, 0, 1);
        sleep(30);
        check(numPhoton.unfinishedCommands.isEmpty() && numOriginal.unfinishedCommands.isEmpty() && routes(numPhoton).isEmpty()
                        && PhotonCore.inFlight(numPhoton) == 0 && PhotonCore.counters().abandonedWrites == 1 && !numPhoton.isNotResponding(),
                "message numbers: nothing leaked, and the one abandoned write is the clashed one");

        // RS485 child stays stock
        Rig rs485 = new Rig(new int[] {1, 2}, new boolean[] {true, false});
        check(rs485.takeOver().isEmpty(), "RS485: take-over of parent and child");
        PhotonLynxModule child = rs485.photon(2);
        check(child != null && PhotonCore.isSwapped(child) && rs485.impl.knownModules.get(2) == child, "RS485: child swapped and routed");
        PhotonCore.resetCounters();
        enablePhoton(true);
        rs485.hub.ackDelayMs = 20;
        t0 = System.nanoTime();
        power(child, 0, 100);
        ms = (System.nanoTime() - t0) / 1e6;
        check(PhotonCore.counters().parallelWrites == 0 && ms >= 15, String.format("RS485: child write waits for its ACK the SDK way (%.1f ms)", ms));
        power(rs485.photon(1), 0, 100);
        check(PhotonCore.counters().parallelWrites == 1, "RS485: the parent on the same link still goes parallel");
        sleep(50);

        // A final LynxModule field blocks the take-over before anything changes
        Rig blocked = new Rig(4);
        blocked.map.put("pinned", new FinalDevice(blocked.originals.get(0)));
        List<String> blockedProblems = blocked.takeOver();
        check(!blockedProblems.isEmpty() && blocked.hubAt(4) == blocked.originals.get(0)
                        && blocked.impl.knownModules.get(4) == blocked.originals.get(0),
                "final field: take-over refused with nothing changed " + blockedProblems);

        // INIT hook: only an @Photon OpMode's hubs are taken over, and only it may enable
        OpModeManagerImpl manager = (OpModeManagerImpl) UNSAFE.allocateInstance(OpModeManagerImpl.class);
        set(core(), PhotonCore.class, "opModeManager", manager);
        Rig plainRig = new Rig(5);
        PlainOpMode plain = (PlainOpMode) UNSAFE.allocateInstance(PlainOpMode.class);
        plain.hardwareMap = plainRig.map;
        set(manager, OpModeManagerImpl.class, "activeOpModeName", "Plain");
        set(manager, OpModeManagerImpl.class, "activeOpMode", plain);
        manager.getActiveOpMode();
        PhotonCore.class.getMethod("onOpModePreInit", com.qualcomm.robotcore.eventloop.opmode.OpMode.class).invoke(core(), plain);
        boolean plainThrew = false;
        try { PhotonCore.enable(); } catch (IllegalStateException e) { plainThrew = true; }
        check(plainRig.hubAt(5) == plainRig.originals.get(0) && plainRig.impl.knownModules.get(5) == plainRig.originals.get(0) && plainThrew,
                "INIT hook: an OpMode without @Photon keeps the SDK's hubs and can't enable");
        Rig annotatedRig = new Rig(6);
        AnnotatedOpMode annotated = (AnnotatedOpMode) UNSAFE.allocateInstance(AnnotatedOpMode.class);
        annotated.hardwareMap = annotatedRig.map;
        set(manager, OpModeManagerImpl.class, "activeOpModeName", OpModeManagerImpl.DEFAULT_OP_MODE_NAME);
        set(manager, OpModeManagerImpl.class, "activeOpMode", annotated);
        PhotonCore.class.getMethod("onOpModePreInit", com.qualcomm.robotcore.eventloop.opmode.OpMode.class).invoke(core(), annotated);
        check(annotatedRig.hubAt(6) == annotatedRig.originals.get(0), "INIT hook: a system OpMode name is never touched, even annotated");
        set(manager, OpModeManagerImpl.class, "activeOpModeName", "Annotated");
        PhotonCore.class.getMethod("onOpModePreInit", com.qualcomm.robotcore.eventloop.opmode.OpMode.class).invoke(core(), annotated);
        boolean enabled = false;
        try { PhotonCore.enable(); enabled = PhotonCore.isEnabled(); } catch (IllegalStateException e) { System.out.println("    " + e); }
        check(PhotonCore.isSwapped(annotatedRig.hubAt(6)) && PhotonCore.swapFailures().isEmpty() && enabled,
                "INIT hook: an @Photon OpMode's hubs are taken over and enable() works");
        PhotonCore.class.getMethod("onOpModePostStop", com.qualcomm.robotcore.eventloop.opmode.OpMode.class).invoke(core(), annotated);
        check(!PhotonCore.isEnabled(), "STOP disables Photon");
        set(core(), PhotonCore.class, "opModeManager", null);

        // API guards
        boolean threw = false;
        try { PhotonCore.enable(); } catch (IllegalStateException e) { threw = true; }
        check(threw, "enable() throws outside a @Photon OpMode");
        threw = false;
        try { PhotonCore.setMaxParallelWrites(0); } catch (IllegalArgumentException e) { threw = true; }
        check(threw, "setMaxParallelWrites(0) throws");
        threw = false;
        try { PhotonCore.setMaxParallelWrites(65); } catch (IllegalArgumentException e) { threw = true; }
        check(threw && PhotonCore.maxParallelWrites() <= 64, "setMaxParallelWrites(65) throws, so every hub keeps message numbers free");

    }

    static boolean refusedForNumbers(Throwable t) {
        return t instanceof IllegalStateException && String.valueOf(t.getMessage()).contains("no free message number");
    }

    @SuppressWarnings("unchecked")
    static Map<SerialNumber, HardwareDevice> map(Rig rig) throws Exception {
        return (Map<SerialNumber, HardwareDevice>) get(rig.map, HardwareMap.class, "serialNumberMap");
    }
}
