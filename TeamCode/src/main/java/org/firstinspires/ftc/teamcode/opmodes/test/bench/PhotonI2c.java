package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;
import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

final class PhotonI2c {
    static final long PERIOD_NS = 10_000_000L;
    static final int PINPOINT_ID_REGISTER = 1;
    static final int PINPOINT_VERSION_REGISTER = 2;
    static final int OCTOQUAD_CHIP_ID_REGISTER = 0;
    static final int OCTOQUAD_FIRMWARE_REGISTER = 1;
    private static final int RAW_RETRIES = 50;

    final List<Device> devices = new ArrayList<>();
    private final PhotonRig rig;

    PhotonI2c(HardwareMap map, PhotonRig rig) {
        this.rig = rig;
        GoBildaPinpointDriver pinpoint = BenchIO.require(map, GoBildaPinpointDriver.class, BettaConstants.pinpoint.name,
                BenchIO.RUNNER_HINT);
        OctoQuad octoQuad = BenchIO.require(map, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
        LynxModule pinpointHub = rig.expansionHub != null ? rig.expansionHub : rig.controlHub;
        LynxModule octoQuadHub = rig.controlHub;
        int id = pinpoint.getDeviceID();
        int version = pinpoint.getDeviceVersion();
        Device p = new Device("pinpoint", (LynxI2cDeviceSynch) pinpoint.getDeviceClient(), pinpointHub,
                PINPOINT_ID_REGISTER, le(id), PINPOINT_VERSION_REGISTER, le(version), map);
        if (id == 0 || version == 0) {
            throw new IllegalStateException("the Pinpoint on " + p.where() + " read device ID " + id + ", version " + version
                    + ": it isn't answering. Plug it in, or rerun without --photon-i2c");
        }
        byte chipId = octoQuad.getChipId();
        if (chipId != OctoQuad.OCTOQUAD_CHIP_ID) {
            throw new IllegalStateException(String.format(Locale.US, "the OctoQuad on %s I2C bus %d read chip ID 0x%02x, not "
                            + "0x%02x: it isn't answering. Plug it in there, or rerun without --photon-i2c", rig.key(octoQuadHub),
                    PhotonProbe.i2cBus(clientOf(octoQuad)), chipId & 0xFF, OctoQuad.OCTOQUAD_CHIP_ID));
        }
        OctoQuad.FirmwareVersion fw = octoQuad.getFirmwareVersion();
        byte[] firmware = {(byte) fw.maj, (byte) fw.min, (byte) fw.eng};
        if (Arrays.equals(firmware, new byte[3])) {
            throw new IllegalStateException("the OctoQuad's firmware version reads 0.0.0, so a failed read couldn't be told apart");
        }
        byte[] idAndFirmware = {OctoQuad.OCTOQUAD_CHIP_ID, firmware[0], firmware[1], firmware[2]};
        Device o = new Device("octoquad", clientOf(octoQuad), octoQuadHub, OCTOQUAD_CHIP_ID_REGISTER, idAndFirmware,
                OCTOQUAD_FIRMWARE_REGISTER, firmware, map);
        devices.add(p);
        devices.add(o);
    }

    // By USB link and address, so a client still on a replaced hub object names the hub it really talks to.
    String hubOf(LynxI2cDeviceSynch client) {
        LynxModule m = PhotonProbe.moduleOf(client);
        for (LynxModule h : rig.hubs) {
            if (PhotonProbe.impl(h) == PhotonProbe.impl(m) && h.getModuleAddress() == m.getModuleAddress()) return rig.key(h);
        }
        return "REV Hub #" + m.getModuleAddress();
    }

    private static LynxI2cDeviceSynch clientOf(OctoQuad octoQuad) {
        return (LynxI2cDeviceSynch) ((I2cDeviceSynchDevice<?>) octoQuad).getDeviceClient();
    }

    private static byte[] le(int value) {
        return TypeConversion.intToByteArray(value, ByteOrder.LITTLE_ENDIAN);
    }

    static String hex(byte[] bytes) {
        StringBuilder b = new StringBuilder();
        for (byte x : bytes) b.append(String.format(Locale.US, "%02x", x & 0xFF));
        return b.toString();
    }

    Map<String, Object> info() {
        Map<String, Object> m = new LinkedHashMap<>();
        List<Map<String, Object>> list = new ArrayList<>();
        for (Device d : devices) list.add(d.info());
        m.put("devices", list);
        m.put("periodMs", PERIOD_NS / 1e6);
        m.put("photonSwapFailures", new ArrayList<>(PhotonCore.swapFailures()));
        return m;
    }

    Readers readers(boolean replaced, long periodNs) {
        Readers r = new Readers();
        Thread opModeThread = Thread.currentThread();
        for (Device d : devices) {
            r.add(new Reader(d, "driver", d.driverClient, d.registerA, d.expectedA, opModeThread, periodNs));
            LynxI2cDeviceSynch second = replaced ? d.replacedClient() : d.second;
            r.add(new Reader(d, replaced ? "replaced" : "second", second, d.registerB, d.expectedB, opModeThread, periodNs));
        }
        return r;
    }

    // Skips the hub's I2C lock on purpose, beside a locked reader at full speed: can the byte compares see an overlap at all?
    Map<String, Object> control(int attempts) throws InterruptedException {
        Map<String, Object> out = new LinkedHashMap<>();
        for (Device d : devices) {
            Readers locked = new Readers();
            locked.add(new Reader(d, "driver", d.driverClient, d.registerA, d.expectedA, Thread.currentThread(), 0));
            Map<String, Long> refused = new LinkedHashMap<>();
            long ok = 0;
            long wrong = 0;
            String firstWrong = null;
            Samples rawMs = new Samples();
            locked.start();
            boolean finished = false;
            try {
                for (int i = 0; i < attempts; i++) {
                    long t = System.nanoTime();
                    Object got = rawRead(d.module(), d.bus, d.address, d.registerB, d.expectedB.length);
                    rawMs.addNanos(System.nanoTime() - t);
                    if (got instanceof byte[]) {
                        if (Arrays.equals((byte[]) got, d.expectedB)) ok++;
                        else {
                            wrong++;
                            if (firstWrong == null) firstWrong = hex((byte[]) got);
                        }
                    } else {
                        Long n = refused.get(got.toString());
                        refused.put(got.toString(), n == null ? 1 : n + 1);
                    }
                    locked.check();
                }
                finished = true;
            } finally {
                locked.halt(finished);
            }
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("hub", rig.key(d.hub));
            m.put("attempted", attempts);
            m.put("unlockedRight", ok);
            m.put("unlockedWrong", wrong);
            m.put("unlockedFirstWrong", firstWrong);
            m.put("unlockedRefused", refused);
            m.put("unlockedReadMs", rawMs.summary());
            m.put("lockedReader", locked.summary().get(0));
            out.put(d.name, m);
        }
        return out;
    }

    // Mirrors LynxI2cDeviceSynchV2.readTimeStamped minus acquireI2cLockWhile: the hub's own busy NACKs are retried.
    private static Object rawRead(LynxModule hub, int bus, I2cAddr address, int register, int length) throws InterruptedException {
        for (int i = 0; ; i++) {
            try {
                new LynxI2cWriteReadMultipleBytesCommand(hub, bus, address, register, length).send();
                break;
            } catch (LynxNackException e) {
                if (!retryable(e) || i >= RAW_RETRIES) return "write-read " + e.getNack().getNackReasonCodeAsEnum();
            }
        }
        for (int i = 0; ; i++) {
            try {
                LynxI2cReadStatusQueryResponse response = new LynxI2cReadStatusQueryCommand(hub, bus, length).sendReceive();
                return response.getBytes();
            } catch (LynxNackException e) {
                if (!retryable(e) || i >= RAW_RETRIES) return "status " + e.getNack().getNackReasonCodeAsEnum();
            }
        }
    }

    private static boolean retryable(LynxNackException e) {
        LynxNack.StandardReasonCode code = e.getNack().getNackReasonCodeAsEnum();
        return code == LynxNack.StandardReasonCode.I2C_MASTER_BUSY || code == LynxNack.StandardReasonCode.I2C_OPERATION_IN_PROGRESS;
    }

    final class Device {
        final String name;
        final LynxI2cDeviceSynch driverClient;
        final LynxI2cDeviceSynch second;
        final LynxModule hub;
        final int bus;
        final I2cAddr address;
        final int registerA;
        final byte[] expectedA;
        final int registerB;
        final byte[] expectedB;
        private final HardwareMap map;
        private LynxI2cDeviceSynch replaced;

        Device(String name, LynxI2cDeviceSynch driverClient, LynxModule hub, int registerA, byte[] expectedA, int registerB,
               byte[] expectedB, HardwareMap map) {
            this.name = name;
            this.driverClient = driverClient;
            this.hub = hub;
            this.registerA = registerA;
            this.expectedA = expectedA;
            this.registerB = registerB;
            this.expectedB = expectedB;
            this.map = map;
            bus = PhotonProbe.i2cBus(driverClient);
            address = driverClient.getI2cAddress();
            LynxModule on = module();
            if (on.getModuleAddress() != hub.getModuleAddress()) {
                throw new IllegalStateException("the " + name + " is on REV Hub #" + on.getModuleAddress() + ", but the Photon "
                        + "block wants it on the " + rig.key(hub) + " (#" + hub.getModuleAddress() + "); " + BenchIO.RUNNER_HINT);
            }
            second = client(on);
        }

        LynxModule module() {
            return PhotonProbe.moduleOf(driverClient);
        }

        LynxI2cDeviceSynch replacedClient() {
            if (replaced == null) replaced = client(PhotonProbe.replaced(module()));
            return replaced;
        }

        private LynxI2cDeviceSynch client(LynxModule on) {
            LynxI2cDeviceSynch c = LynxFirmwareVersionManager.createLynxI2cDeviceSynch(map.appContext, on, bus);
            c.setI2cAddress(address);
            return c;
        }

        String where() {
            return hubKey() + " I2C bus " + bus;
        }

        String hubKey() {
            return rig.key(hub);
        }

        Map<String, Object> info() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("device", name);
            m.put("hub", rig.key(hub));
            m.put("hubAddress", hub.getModuleAddress());
            m.put("bus", bus);
            m.put("i2cAddress", address.get7Bit());
            m.put("clientClass", driverClient.getClass().getSimpleName());
            m.put("clientHub", hubOf(driverClient));
            m.put("clientModuleSwapped", PhotonCore.isSwapped(module()));
            m.put("secondHub", hubOf(second));
            m.put("secondBus", PhotonProbe.i2cBus(second));
            m.put("secondI2cAddress", second.getI2cAddress().get7Bit());
            m.put("secondModuleSwapped", PhotonCore.isSwapped(PhotonProbe.moduleOf(second)));
            m.put("registerA", registerA);
            m.put("expectedA", hex(expectedA));
            m.put("registerB", registerB);
            m.put("expectedB", hex(expectedB));
            m.put("readersDistinguishable", !Arrays.equals(expectedA, expectedB));
            return m;
        }
    }

    static final class Readers {
        private final List<Reader> list = new ArrayList<>();
        private final List<String> haltProblems = new ArrayList<>();

        void add(Reader r) {
            list.add(r);
        }

        void start() {
            for (Reader r : list) r.start();
        }

        void check() {
            for (Reader r : list) r.check();
        }

        // Interrupt status is cleared for the joins and restored: a LinearOpMode is interrupted at STOP, and readers are never.
        void halt(boolean throwProblems) {
            for (Reader r : list) r.running = false;
            boolean interrupted = Thread.interrupted();
            try {
                for (Reader r : list) {
                    try {
                        r.join(1000);
                    } catch (InterruptedException e) {
                        interrupted = true;
                    }
                    if (r.isAlive()) haltProblems.add(r.getName() + " was still running 1 s after it was told to stop");
                    if (r.failure != null) haltProblems.add(r.getName() + " failed: " + r.failure);
                }
            } finally {
                if (interrupted) Thread.currentThread().interrupt();
            }
            if (!throwProblems || haltProblems.isEmpty()) return;
            for (Reader r : list) r.check();
            throw new IllegalStateException(haltProblems.get(0));
        }

        List<Map<String, Object>> summary() {
            List<Map<String, Object>> out = new ArrayList<>();
            for (Reader r : list) {
                Map<String, Object> m = r.summary();
                if (!haltProblems.isEmpty()) m.put("haltProblems", new ArrayList<>(haltProblems));
                out.add(m);
            }
            return out;
        }

        long reads() {
            long n = 0;
            for (Reader r : list) n += r.reads.get();
            return n;
        }
    }

    final class Reader extends Thread {
        private final Device device;
        private final String client;
        private final LynxI2cDeviceSynch i2c;
        private final int register;
        private final byte[] expected;
        private final Thread opModeThread;
        private final long periodNs;
        private final ReadTimes times = new ReadTimes();
        final AtomicLong reads = new AtomicLong();
        final AtomicLong wrong = new AtomicLong();
        final AtomicLong placeholders = new AtomicLong();
        private volatile String firstWrong;
        volatile boolean running = true;
        volatile Throwable failure;

        Reader(Device device, String client, LynxI2cDeviceSynch i2c, int register, byte[] expected, Thread opModeThread,
               long periodNs) {
            super("PhotonBenchI2c-" + device.name + "-" + client);
            this.device = device;
            this.client = client;
            this.i2c = i2c;
            this.register = register;
            this.expected = expected.clone();
            this.opModeThread = opModeThread;
            this.periodNs = periodNs;
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                byte[] zeros = new byte[expected.length];
                long next = System.nanoTime();
                while (running && opModeThread.isAlive()) {
                    long t = System.nanoTime();
                    byte[] got = i2c.read(register, expected.length);
                    times.add(System.nanoTime() - t);
                    reads.incrementAndGet();
                    if (Arrays.equals(got, zeros)) placeholders.incrementAndGet();
                    else if (!Arrays.equals(got, expected)) {
                        wrong.incrementAndGet();
                        if (firstWrong == null) firstWrong = hex(got);
                    }
                    if (periodNs == 0) continue;
                    next += periodNs;
                    long wait = next - System.nanoTime();
                    if (wait > 0) Thread.sleep(wait / 1_000_000, (int) (wait % 1_000_000));
                    else next = System.nanoTime();
                }
            } catch (Throwable t) {
                failure = t;
            }
        }

        void check() {
            Throwable t = failure;
            if (t != null) throw new IllegalStateException(getName() + " failed", t);
        }

        Map<String, Object> summary() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("device", device.name);
            m.put("client", client);
            m.put("hub", hubOf(i2c));
            m.put("hubAddress", PhotonProbe.moduleOf(i2c).getModuleAddress());
            m.put("bus", PhotonProbe.i2cBus(i2c));
            m.put("i2cAddress", i2c.getI2cAddress().get7Bit());
            m.put("register", register);
            m.put("expected", hex(expected));
            m.put("moduleSwapped", PhotonCore.isSwapped(PhotonProbe.moduleOf(i2c)));
            m.put("periodMs", periodNs / 1e6);
            m.put("reads", reads.get());
            m.put("wrong", wrong.get());
            m.put("placeholders", placeholders.get());
            m.put("firstWrong", firstWrong);
            m.put("failure", failure == null ? null : failure.toString());
            m.put("readMs", times.summary());
            return m;
        }
    }

    // Fixed buckets, so a 4-hour soak's reads cost a constant few KB per reader.
    static final class ReadTimes {
        private static final double BUCKET_MS = 0.1;
        private static final int BUCKETS = 1000;
        private final long[] counts = new long[BUCKETS + 1];
        private long n;
        private double sumMs;
        private double maxMs;

        void add(long nanos) {
            double ms = nanos / 1e6;
            counts[(int) Math.min(BUCKETS, ms / BUCKET_MS)]++;
            n++;
            sumMs += ms;
            if (ms > maxMs) maxMs = ms;
        }

        Map<String, Object> summary() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("count", n);
            if (n == 0) return m;
            m.put("mean", sumMs / n);
            m.put("p50", percentile(0.50));
            m.put("p95", percentile(0.95));
            m.put("p99", percentile(0.99));
            m.put("max", maxMs);
            return m;
        }

        private double percentile(double p) {
            long rank = (long) Math.ceil(p * n);
            long seen = 0;
            for (int b = 0; b <= BUCKETS; b++) {
                seen += counts[b];
                if (seen >= rank) return b == BUCKETS ? maxMs : (b + 1) * BUCKET_MS;
            }
            return maxMs;
        }
    }
}
