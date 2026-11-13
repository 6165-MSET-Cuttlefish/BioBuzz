package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.locks.LockSupport;

@Photon
@TeleOp(name = "Bench: Photon", group = "Test")
public class PhotonBench extends PhotonWorkload {
    @Override
    public void runOpMode() {
        setUp("Photon", "photon_" + BenchIO.param(BenchIO.params(), "runId", "1"));
        Map<String, Object> info = rig.info();
        info.put("photonEnabledAtInit", PhotonCore.isEnabled());
        info.put("inFlightAtInit", rig.inFlight());
        Map<String, Object> left = new LinkedHashMap<>();
        for (LynxModule h : rig.hubs) left.put(rig.key(h), PhotonProbe.leftovers(h));
        info.put("leftoversAtInit", left);
        info.put("maxParallelWritesAtInit", PhotonCore.maxParallelWrites());
        info.put("routingProblems", PhotonProbe.routingProblems(hardwareMap));
        report.put("info", info);
        if (i2c != null) report.put("i2cInfo", i2c.info());
        report.save();
        for (LynxModule h : rig.hubs) {
            if (!PhotonCore.isSwapped(h)) {
                throw new IllegalStateException("Photon didn't take over REV Hub #" + h.getModuleAddress() + ": "
                        + PhotonCore.swapFailures());
            }
        }
        if (!report.holdUntilStart(this)) return;
        try {
            runPhases();
        } finally {
            PhotonCore.disable();
            rig.zeroMotors();
            rig.disableServos();
        }
        report.done(String.format("write block p50 %s ms off, %s ms on8", p50("off"), p50("on8")));
        report.holdUntilStop(this);
    }

    private Object p50(String mode) {
        Object t = report.root.get("timing");
        if (!(t instanceof Map)) return null;
        Object m = ((Map<?, ?>) t).get(mode);
        if (!(m instanceof Map)) return null;
        Object s = ((Map<?, ?>) m).get("writeBlockMs");
        return s instanceof Map ? ((Map<?, ?>) s).get("p50") : null;
    }

    private void runPhases() {
        Map<String, Object> timing = new LinkedHashMap<>();
        report.put("timing", timing);
        int[][] modes = {{0, 0}, {8, 0}, {4, 0}, {2, 0}, {0, 1}, {8, 1}};
        for (int[] m : modes) {
            if (!opModeIsActive()) return;
            String name = modeName(m[0]) + (m[1] == 1 ? "+poller" : "");
            timing.put(name, timing("timing " + name, m[0], m[1] == 1));
            report.save();
        }
        if (!opModeIsActive()) return;
        timing.put("on8 profiled", timing("timing on8 profiled", 8, false, true));
        report.save();
        Map<String, Object> burst = new LinkedHashMap<>();
        report.put("burst", burst);
        for (int parallel : new int[] {0, 8, 2, 1}) {
            if (!opModeIsActive()) return;
            burst.put(modeName(parallel), burst("burst " + modeName(parallel), parallel, Math.max(50, loops / 2)));
            report.save();
        }
        Map<String, Object> last = new LinkedHashMap<>();
        report.put("lastWriterWins", last);
        for (int parallel : new int[] {8, 1}) {
            if (!opModeIsActive()) return;
            last.put(modeName(parallel), lastWriterWins("last writer wins " + modeName(parallel), parallel,
                    Math.max(50, loops / 2)));
            report.save();
        }
        Map<String, Object> interleave = new LinkedHashMap<>();
        report.put("interleave", interleave);
        for (int parallel : new int[] {0, 8}) {
            if (!opModeIsActive()) return;
            interleave.put(modeName(parallel), interleave("interleave " + modeName(parallel), parallel,
                    Math.max(30, loops / 5)));
            report.save();
        }
        Map<String, Object> reads = new LinkedHashMap<>();
        report.put("reads", reads);
        for (int parallel : new int[] {0, 8}) {
            if (!opModeIsActive()) return;
            reads.put(modeName(parallel), reads("reads " + modeName(parallel), parallel, Math.max(100, loops / 2)));
            report.save();
        }
        if (i2c != null) {
            Map<String, Object> i2cPhases = new LinkedHashMap<>();
            report.put("i2c", i2cPhases);
            for (int parallel : new int[] {0, 2, 8}) {
                if (!opModeIsActive()) return;
                i2cPhases.put(modeName(parallel), i2cReads("i2c " + modeName(parallel), parallel, parallel == 0, loops));
                report.save();
            }
        }
        if (rig.topology == PhotonRig.RS485) {
            Map<String, Object> rs485 = new LinkedHashMap<>();
            report.put("rs485SharedLink", rs485);
            for (int parallel : new int[] {0, 8}) {
                if (!opModeIsActive()) return;
                rs485.put(modeName(parallel), rs485SharedLink("rs485 shared link " + modeName(parallel), parallel));
                report.save();
            }
        }
        if (!opModeIsActive()) return;
        report.put("twoWriters", twoWriters(BenchIO.param(report.params, "twoWriterSeconds", 8)));
        report.save();
        if (!opModeIsActive()) return;
        report.put("storm", storm((int) BenchIO.param(report.params, "stormWrites", 3000)));
        report.save();
        if (i2c != null) {
            if (!opModeIsActive()) return;
            report.put("i2cControl", i2cControl((int) BenchIO.param(report.params, "i2cControlAttempts", 200)));
            report.save();
            if (!opModeIsActive()) return;
            report.put("i2cAfterControl", i2cReads("i2c after control", 0, false, Math.max(50, loops / 5)));
            report.save();
        }
        if (!opModeIsActive()) return;
        report.put("soak", soak(BenchIO.param(report.params, "soakSeconds", 60),
                BenchIO.param(report.params, "soakWritesPerHub", 0)));
        report.save();
        if (rig.topology == PhotonRig.USB) {
            Map<String, Object> overlap = new LinkedHashMap<>();
            report.put("hubOverlap", overlap);
            int rounds = (int) BenchIO.param(report.params, "overlapLoops", HubOverlap.LOOPS);
            for (int parallel : new int[] {0, 8, 4, 2}) {
                if (!opModeIsActive()) return;
                overlap.put(modeName(parallel), hubOverlap("hub overlap " + modeName(parallel), parallel, rounds));
                report.save();
            }
        }
    }

    // Each loop drains before the next, so every set starts with nothing in flight and its drain is its own.
    private Map<String, Object> hubOverlap(String name, int parallel, int rounds) {
        Phase p = new Phase(name, parallel);
        Map<String, List<PhotonChannel>> sets = HubOverlap.sets(kind(rig.controlHub, true), kind(rig.controlHub, false),
                kind(rig.expansionHub, true), kind(rig.expansionHub, false));
        List<String> names = new ArrayList<>(sets.keySet());
        Map<String, Samples> returned = new LinkedHashMap<>();
        Map<String, Samples> drained = new LinkedHashMap<>();
        for (String set : names) {
            returned.put(set, new Samples());
            drained.put(set, new Samples());
        }
        Random random = new Random(HubOverlap.SEED);
        int drainTimeouts = 0;
        long loop = 0;
        for (int r = 0; r < rounds && opModeIsActive(); r++) {
            for (int k : HubOverlap.roundOrder(names.size(), random)) {
                String set = names.get(k);
                List<PhotonChannel> writes = sets.get(set);
                bulkRead();
                for (PhotonChannel c : writes) c.prepare();
                long callNanos = 0;
                long start = System.nanoTime();
                for (PhotonChannel c : writes) {
                    long t = System.nanoTime();
                    c.send();
                    callNanos += System.nanoTime() - t;
                    p.sampleInFlight(c.hub);
                }
                double drain = rig.drain(DRAIN_TIMEOUT_MS);
                long end = System.nanoTime();
                returned.get(set).addNanos(callNanos);
                if (Double.isNaN(drain)) drainTimeouts++;
                else drained.get(set).addNanos(end - start);
                for (PhotonChannel c : writes) p.countWrite(c);
                p.step = loop;
                if (loop % VERIFY_EVERY == VERIFY_EVERY - 1) p.verifyAll(false);
                loop++;
                report.status("%s %d/%d", name, r + 1, rounds);
                report.sendIfDue();
            }
        }
        Map<String, Object> out = new LinkedHashMap<>();
        for (String set : names) {
            Map<String, Object> s = new LinkedHashMap<>();
            List<String> channels = new ArrayList<>();
            for (PhotonChannel c : sets.get(set)) channels.add(c.name);
            s.put("channels", channels);
            s.put("returnMs", returned.get(set).summary());
            s.put("drainedMs", drained.get(set).summary());
            out.put(set, s);
        }
        p.result.put("rounds", rounds);
        p.result.put("seed", HubOverlap.SEED);
        p.result.put("drainTimeouts", drainTimeouts);
        p.result.put("sets", out);
        return p.end();
    }

    private List<PhotonChannel> kind(LynxModule hub, boolean motors) {
        List<PhotonChannel> out = new ArrayList<>();
        for (PhotonChannel c : rig.on(hub)) if (c.isMotor() == motors) out.add(c);
        return out;
    }

    // The Control Hub's parallel writes skip the RS485 link's lock that the child's writes and the poller's reads take.
    private Map<String, Object> rs485SharedLink(String name, int parallel) {
        Phase p = new Phase(name, parallel);
        Poller poller = new Poller(Thread.currentThread());
        poller.start();
        Map<LynxModule, Samples> perHub = new LinkedHashMap<>();
        for (LynxModule h : rig.hubs) perHub.put(h, new Samples());
        try {
            for (int i = 0; i < loops && opModeIsActive(); i++) {
                for (LynxModule h : rig.hubs) {
                    long t = System.nanoTime();
                    for (PhotonChannel c : rig.on(h)) p.write(c);
                    perHub.get(h).addNanos(System.nanoTime() - t);
                }
                p.step = i;
                if (i % VERIFY_EVERY == VERIFY_EVERY - 1) p.verifyAll(false);
                poller.check();
                report.status("%s %d/%d", name, i + 1, loops);
                report.sendIfDue();
            }
        } finally {
            poller.halt();
        }
        Map<String, Object> blocks = new LinkedHashMap<>();
        for (Map.Entry<LynxModule, Samples> e : perHub.entrySet()) blocks.put(rig.key(e.getKey()), e.getValue().summary());
        p.result.put("writeBlockPerHubMs", blocks);
        p.result.put("pollerCommandMs", poller.summary());
        return p.end();
    }

    private Map<String, Object> twoWriters(double seconds) {
        Phase p = new Phase("two writers", 8);
        p.result.put("countExact", false);
        List<PhotonChannel> servos = new ArrayList<>();
        List<PhotonChannel> motors = new ArrayList<>();
        for (PhotonChannel c : rig.channels) (c.isMotor() ? motors : servos).add(c);
        Thread opModeThread = Thread.currentThread();
        ServoWriter servoWriter = new ServoWriter(p, servos, opModeThread);
        Toggler toggler = new Toggler(opModeThread, 50);
        long motorWrites = 0;
        long iteration = 0;
        long end = System.nanoTime() + (long) (seconds * 1e9);
        servoWriter.start();
        toggler.start();
        try {
            while (System.nanoTime() < end && opModeIsActive()) {
                for (PhotonChannel c : motors) {
                    p.write(c);
                    motorWrites++;
                }
                p.step = iteration;
                if (iteration % VERIFY_EVERY == VERIFY_EVERY - 1) {
                    for (PhotonChannel c : motors) p.readback.check(p.name, iteration, c, false);
                }
                iteration++;
                servoWriter.check();
                toggler.check();
                report.status("two writers %d motor writes", motorWrites);
                report.sendIfDue();
            }
        } finally {
            servoWriter.halt();
            toggler.halt();
            PhotonCore.setMaxParallelWrites(8);
            PhotonCore.enable();
        }
        p.result.put("seconds", seconds);
        p.result.put("motorWrites", motorWrites);
        p.result.put("servoWrites", servoWriter.writes);
        p.result.put("toggles", toggler.toggles);
        return p.end();
    }

    private Map<String, Object> storm(int writes) {
        Phase p = new Phase("storm", 1);
        p.result.put("burstPerHub", writes);
        Watchdog watchdog = new Watchdog(Thread.currentThread());
        watchdog.start();
        Samples writeMs = new Samples();
        try {
            for (int i = 0; i < writes && opModeIsActive(); i++) {
                PhotonChannel c = rig.channels.get(i % rig.channels.size());
                long t = System.nanoTime();
                p.write(c);
                writeMs.addNanos(System.nanoTime() - t);
                watchdog.beat();
                if (i % 50 == 0) {
                    report.status("storm %d/%d", i, writes);
                    report.sendIfDue();
                }
            }
        } finally {
            watchdog.halt();
        }
        p.result.put("writes", writes);
        p.result.put("writeMs", writeMs.summary());
        p.result.put("maxStallMs", watchdog.maxGapMs);
        return p.end();
    }

    private Map<String, Object> i2cControl(int attempts) {
        report.phase("i2c control");
        PhotonCore.disable();
        try {
            return i2c.control(attempts);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted in the unlocked-I2C negative control", e);
        }
    }

    private Map<String, Object> soak(double seconds, double writesPerHub) {
        Phase p = new Phase("soak", 8);
        Poller poller = new Poller(Thread.currentThread());
        poller.start();
        PhotonI2c.Readers readers = i2c == null ? null : i2c.readers(false, PhotonI2c.PERIOD_NS);
        if (readers != null) readers.start();
        long t0 = System.nanoTime();
        long deadline = t0 + (long) (seconds * 1e9);
        long loop = 0;
        long lastSave = t0;
        boolean finished = false;
        try {
            while (opModeIsActive()) {
                long now = System.nanoTime();
                if (now > deadline || (writesPerHub > 0 && minIssued(p) >= writesPerHub)) break;
                for (PhotonChannel c : rig.channels) p.write(c);
                p.step = loop;
                if (loop % 50 == 49) p.verifyAll(false);
                if (loop % 200 == 0) {
                    poller.check();
                    if (readers != null) readers.check();
                }
                loop++;
                if (now - lastSave > 30_000_000_000L) {
                    lastSave = now;
                    Map<String, Object> progress = progress(p, loop, (now - t0) / 1e9);
                    if (readers != null) progress.put("i2cReads", readers.reads());
                    report.put("soakProgress", progress);
                    report.save();
                }
                report.status("soak %.0f s, %d loops", (now - t0) / 1e9, loop);
                report.sendIfDue();
            }
            finished = true;
        } finally {
            poller.halt();
            if (readers != null) readers.halt(finished);
        }
        p.result.put("seconds", (System.nanoTime() - t0) / 1e9);
        p.result.put("loops", loop);
        p.result.put("targetWritesPerHub", writesPerHub);
        p.result.put("pollerCommandMs", poller.summary());
        if (readers != null) p.result.put("i2cReaders", readers.summary());
        return p.end();
    }

    private long minIssued(Phase p) {
        long min = Long.MAX_VALUE;
        for (LynxModule h : rig.hubs) {
            if (PhotonRig.parallelCapable(h)) min = Math.min(min, p.issued(h));
        }
        return min;
    }

    private Map<String, Object> progress(Phase p, long loop, double seconds) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("seconds", seconds);
        m.put("loops", loop);
        for (LynxModule h : rig.hubs) m.put(rig.key(h) + "Writes", p.issued(h));
        m.put("counters", PhotonProbe.delta(p.countersBefore, PhotonProbe.counters()));
        m.put("readbackMismatches", p.readback.count);
        return m;
    }

    private static final class ServoWriter extends Thread {
        private final Phase phase;
        private final List<PhotonChannel> servos;
        private final Thread opModeThread;
        private volatile boolean running = true;
        volatile Throwable failure;
        volatile long writes;

        ServoWriter(Phase phase, List<PhotonChannel> servos, Thread opModeThread) {
            super("PhotonServoWriter");
            this.phase = phase;
            this.servos = servos;
            this.opModeThread = opModeThread;
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                while (running && opModeThread.isAlive()) {
                    for (PhotonChannel c : servos) phase.write(c);
                    writes += servos.size();
                    LockSupport.parkNanos(2_000_000L);
                }
            } catch (Throwable t) {
                failure = t;
            }
        }

        void check() {
            if (failure != null) throw new IllegalStateException("the servo writer thread failed", failure);
        }

        void halt() {
            running = false;
            joinOrThrow(this, "servo writer");
            check();
        }
    }

    private static final class Toggler extends Thread {
        private final Thread opModeThread;
        private final int count;
        private final Random random = new Random(6165);
        private volatile boolean running = true;
        volatile Throwable failure;
        volatile int toggles;

        Toggler(Thread opModeThread, int count) {
            super("PhotonToggler");
            this.opModeThread = opModeThread;
            this.count = count;
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                while (running && opModeThread.isAlive() && toggles < count) {
                    LockSupport.parkNanos((20 + random.nextInt(60)) * 1_000_000L);
                    PhotonCore.disable();
                    LockSupport.parkNanos((5 + random.nextInt(20)) * 1_000_000L);
                    PhotonCore.enable();
                    toggles++;
                }
            } catch (Throwable t) {
                failure = t;
            }
        }

        void check() {
            if (failure != null) throw new IllegalStateException("the Photon toggler thread failed", failure);
        }

        void halt() {
            running = false;
            joinOrThrow(this, "Photon toggler");
            check();
        }
    }

    private static final class Watchdog extends Thread {
        private final Thread opModeThread;
        private volatile long lastBeat = System.nanoTime();
        private volatile boolean running = true;
        volatile double maxGapMs;

        Watchdog(Thread opModeThread) {
            super("PhotonStallWatchdog");
            this.opModeThread = opModeThread;
            setDaemon(true);
        }

        void beat() {
            lastBeat = System.nanoTime();
        }

        @Override
        public void run() {
            while (running && opModeThread.isAlive()) {
                double gap = (System.nanoTime() - lastBeat) / 1e6;
                if (gap > maxGapMs) maxGapMs = gap;
                LockSupport.parkNanos(1_000_000L);
            }
        }

        void halt() {
            running = false;
            joinOrThrow(this, "stall watchdog");
        }
    }

    static void joinOrThrow(Thread t, String what) {
        try {
            t.join(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted stopping the " + what, e);
        }
        if (t.isAlive()) throw new IllegalStateException("the " + what + " was still running 1 s after it was told to stop");
    }
}
