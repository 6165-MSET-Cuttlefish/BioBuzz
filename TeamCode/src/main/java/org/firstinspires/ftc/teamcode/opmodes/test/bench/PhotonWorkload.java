package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

abstract class PhotonWorkload extends LinearOpMode {
    static final int VERIFY_EVERY = 10;
    static final double DRAIN_TIMEOUT_MS = 500;
    static final long POLL_NS = 10_000_000L;

    protected BenchReport report;
    protected PhotonRig rig;
    protected PhotonI2c i2c;
    protected int loops;

    protected void setUp(String benchName, String fileName) {
        report = new BenchReport(benchName, fileName);
        loops = (int) BenchIO.param(report.params, "loops", 500);
        rig = new PhotonRig(hardwareMap, report.params, benchName);
        if (BenchIO.param(report.params, "i2c", 0) != 0) i2c = new PhotonI2c(hardwareMap, rig);
    }

    static void setMode(int parallel) {
        if (parallel < 0) return;
        if (parallel == 0) {
            PhotonCore.disable();
            return;
        }
        PhotonCore.setMaxParallelWrites(parallel);
        PhotonCore.enable();
    }

    static String modeName(int parallel) {
        return parallel < 0 ? "stock" : parallel == 0 ? "off" : "on" + parallel;
    }

    final class Phase {
        final String name;
        final int parallel;
        final Map<String, Object> countersBefore;
        final Map<LynxModule, AtomicLong> issued = new LinkedHashMap<>();
        final Map<LynxModule, AtomicInteger> maxInFlight = new LinkedHashMap<>();
        final PhotonChannel.Mismatches readback = new PhotonChannel.Mismatches();
        final PendingMonitor pending;
        final Map<String, Object> result = new LinkedHashMap<>();
        long step;

        Phase(String name, int parallel) {
            this.name = name;
            this.parallel = parallel;
            report.phase(name);
            for (LynxModule h : rig.hubs) {
                issued.put(h, new AtomicLong());
                maxInFlight.put(h, new AtomicInteger());
            }
            result.put("parallel", parallel);
            result.put("countExact", true);
            int perHub = 0;
            for (LynxModule h : rig.hubs) perHub = Math.max(perHub, rig.on(h).size());
            result.put("burstPerHub", perHub);
            result.put("drainBeforeMs", rig.drain(DRAIN_TIMEOUT_MS));
            countersBefore = PhotonProbe.counters();
            setMode(parallel);
            result.put("photonEnabled", PhotonCore.isEnabled());
            pending = new PendingMonitor(rig.hubs, 20, Thread.currentThread());
            pending.start();
        }

        void write(PhotonChannel c) {
            c.write();
            noteWrite(c);
        }

        void noteWrite(PhotonChannel c) {
            countWrite(c);
            sampleInFlight(c.hub);
        }

        void countWrite(PhotonChannel c) {
            issued.get(c.hub).incrementAndGet();
        }

        void sampleInFlight(LynxModule hub) {
            int f = PhotonProbe.inFlight(hub);
            AtomicInteger max = maxInFlight.get(hub);
            int was = max.get();
            while (f > was && !max.compareAndSet(was, f)) was = max.get();
        }

        long issued(LynxModule hub) {
            return issued.get(hub).get();
        }

        boolean verifyAll(boolean wantEnabled) {
            boolean ok = true;
            for (PhotonChannel c : rig.channels) ok &= readback.check(name, step, c, wantEnabled);
            return ok;
        }

        Map<String, Object> end() {
            double drain = rig.drain(DRAIN_TIMEOUT_MS);
            result.put("drainAfterMs", drain);
            result.put("inFlightAfterDrain", rig.inFlight());
            Map<String, Object> left = new LinkedHashMap<>();
            for (LynxModule h : rig.hubs) left.put(rig.key(h), PhotonProbe.leftovers(h));
            result.put("leftoversAfterDrain", left);
            PhotonChannel.Mismatches atEnd = new PhotonChannel.Mismatches();
            boolean all = true;
            for (PhotonChannel c : rig.channels) all &= atEnd.check(name + " end", step, c, true);
            result.put("allAtEnd", all);
            result.put("endReadback", atEnd.toMap());
            pending.halt();
            result.put("counters", PhotonProbe.delta(countersBefore, PhotonProbe.counters()));
            Map<String, Object> iss = new LinkedHashMap<>();
            Map<String, Object> mif = new LinkedHashMap<>();
            long onParallelHubs = 0;
            for (LynxModule h : rig.hubs) {
                iss.put(rig.key(h), issued(h));
                mif.put(rig.key(h), maxInFlight.get(h).get());
                if (PhotonRig.parallelCapable(h)) onParallelHubs += issued(h);
            }
            result.put("issuedWrites", iss);
            result.put("issuedOnParallelHubs", onParallelHubs);
            result.put("maxInFlight", mif);
            result.put("maxPendingMs", pending.maxMs(rig));
            result.put("maxPendingEntries", pending.maxEntries(rig));
            result.put("maxStandIns", pending.maxStandIns(rig));
            result.put("pendingAgeSamples", pending.ageHistogram());
            result.put("notResponding", rig.notResponding());
            result.put("readback", readback.toMap());
            setMode(parallel < 0 ? -1 : 0);
            rig.zeroMotors();
            return result;
        }
    }

    protected Map<String, Object> timing(String name, int parallel, boolean withPoller) {
        return timing(name, parallel, withPoller, false);
    }

    // Bookkeeping stays off the clock so the write block measures only what the robot's loop pays.
    protected Map<String, Object> timing(String name, int parallel, boolean withPoller, boolean profiled) {
        Phase p = new Phase(name, parallel);
        Poller poller = withPoller ? new Poller(Thread.currentThread()) : null;
        if (poller != null) poller.start();
        Samples period = new Samples();
        Samples bulk = new Samples();
        Samples motorWrite = new Samples();
        Samples servoWrite = new Samples();
        Samples writeBlock = new Samples();
        Samples prepare = new Samples();
        Samples note = new Samples();
        Map<LynxModule, Samples> perHub = new LinkedHashMap<>();
        Map<LynxModule, List<PhotonChannel>> byHub = new LinkedHashMap<>();
        for (LynxModule h : rig.hubs) {
            perHub.put(h, new Samples());
            byHub.put(h, rig.on(h));
        }
        long lastStart = 0;
        boolean lastVerified = false;
        if (profiled) {
            PhotonCore.resetWriteProfile();
            PhotonCore.setWriteProfiling(true);
        }
        try {
            for (int i = 0; i < loops && opModeIsActive(); i++) {
                long start = System.nanoTime();
                if (lastStart != 0 && !lastVerified) period.addNanos(start - lastStart);
                lastStart = start;
                long t = System.nanoTime();
                bulkRead();
                bulk.addNanos(System.nanoTime() - t);
                t = System.nanoTime();
                for (PhotonChannel c : rig.channels) c.prepare();
                prepare.addNanos(System.nanoTime() - t);
                long blockNanos = 0;
                for (Map.Entry<LynxModule, List<PhotonChannel>> e : byHub.entrySet()) {
                    long hubNanos = 0;
                    for (PhotonChannel c : e.getValue()) {
                        t = System.nanoTime();
                        c.send();
                        long sent = System.nanoTime() - t;
                        hubNanos += sent;
                        (c.isMotor() ? motorWrite : servoWrite).addNanos(sent);
                        p.sampleInFlight(c.hub);
                    }
                    perHub.get(e.getKey()).addNanos(hubNanos);
                    blockNanos += hubNanos;
                }
                writeBlock.addNanos(blockNanos);
                t = System.nanoTime();
                for (PhotonChannel c : rig.channels) p.countWrite(c);
                note.addNanos(System.nanoTime() - t);
                p.step = i;
                lastVerified = i % VERIFY_EVERY == VERIFY_EVERY - 1;
                if (lastVerified) p.verifyAll(false);
                if (poller != null) poller.check();
                report.status("%s %d/%d", name, i + 1, loops);
                report.sendIfDue();
            }
        } finally {
            if (profiled) PhotonCore.setWriteProfiling(false);
            if (poller != null) poller.halt();
        }
        p.result.put("loops", loops);
        p.result.put("loopPeriodMs", period.summary());
        p.result.put("bulkReadMs", bulk.summary());
        p.result.put("motorWriteMs", motorWrite.summary());
        p.result.put("servoWriteMs", servoWrite.summary());
        p.result.put("writeBlockMs", writeBlock.summary());
        p.result.put("prepareMs", prepare.summary());
        p.result.put("noteWritesMs", note.summary());
        Map<String, Object> hubBlocks = new LinkedHashMap<>();
        for (Map.Entry<LynxModule, Samples> e : perHub.entrySet()) hubBlocks.put(rig.key(e.getKey()), e.getValue().summary());
        p.result.put("writeBlockPerHubMs", hubBlocks);
        if (poller != null) p.result.put("pollerCommandMs", poller.summary());
        if (profiled) {
            Map<String, Object> profile = new LinkedHashMap<>();
            for (LynxModule h : rig.hubs) {
                if (PhotonCore.isSwapped(h)) profile.put(rig.key(h), PhotonCore.writeProfile(h));
            }
            p.result.put("writeProfile", profile);
        }
        return p.end();
    }

    protected Map<String, Object> burst(String name, int parallel, int iterations) {
        Phase p = new Phase(name, parallel);
        Samples burstMs = new Samples();
        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            long t = System.nanoTime();
            for (PhotonChannel c : rig.channels) p.write(c);
            burstMs.addNanos(System.nanoTime() - t);
            p.step = i;
            p.verifyAll(false);
            report.status("%s %d/%d", name, i + 1, iterations);
            report.sendIfDue();
        }
        p.result.put("iterations", iterations);
        p.result.put("burstMs", burstMs.summary());
        return p.end();
    }

    protected Map<String, Object> lastWriterWins(String name, int parallel, int iterations) {
        Phase p = new Phase(name, parallel);
        Samples sequenceMs = new Samples();
        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            boolean up = i % 2 == 0;
            long t = System.nanoTime();
            for (PhotonChannel c : rig.channels) {
                double[] values = c.isMotor() ? (up ? new double[] {0.2, 0.25, 0.3} : new double[] {0.25, 0.2, 0.15})
                        : (up ? new double[] {0.45, 0.5, 0.55} : new double[] {0.5, 0.45, 0.35});
                for (double v : values) {
                    c.set(v);
                    p.noteWrite(c);
                }
            }
            sequenceMs.addNanos(System.nanoTime() - t);
            p.step = i;
            p.verifyAll(false);
            report.status("%s %d/%d", name, i + 1, iterations);
            report.sendIfDue();
        }
        p.result.put("iterations", iterations);
        p.result.put("writesPerChannelPerIteration", 3);
        p.result.put("burstPerHub", 3 * (int) p.result.get("burstPerHub"));
        p.result.put("sequenceMs", sequenceMs.summary());
        return p.end();
    }

    protected Map<String, Object> interleave(String name, int parallel, int iterations) {
        Phase p = new Phase(name, parallel);
        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            for (PhotonChannel c : rig.channels) {
                p.write(c);
                if (c.isMotor()) {
                    c.motor.setMotorDisable();
                    c.motor.setMotorEnable();
                } else {
                    c.servo.setPwmDisable();
                    c.servo.setPwmEnable();
                }
            }
            p.step = i;
            p.verifyAll(true);
            report.status("%s %d/%d", name, i + 1, iterations);
            report.sendIfDue();
        }
        p.result.put("iterations", iterations);
        return p.end();
    }

    protected Map<String, Object> reads(String name, int parallel, int iterations) {
        Phase p = new Phase(name, parallel);
        Samples bulk = new Samples();
        Samples voltage = new Samples();
        int lowVoltage = 0;
        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            for (PhotonChannel c : rig.channels) p.write(c);
            long t = System.nanoTime();
            bulkRead();
            bulk.addNanos(System.nanoTime() - t);
            for (LynxModule h : rig.hubs) {
                t = System.nanoTime();
                if (h.getInputVoltage(VoltageUnit.VOLTS) < 9) lowVoltage++;
                voltage.addNanos(System.nanoTime() - t);
            }
            p.step = i;
            if (i % VERIFY_EVERY == VERIFY_EVERY - 1) p.verifyAll(false);
            report.status("%s %d/%d", name, i + 1, iterations);
            report.sendIfDue();
        }
        p.result.put("iterations", iterations);
        p.result.put("bulkReadMs", bulk.summary());
        p.result.put("inputVoltageMs", voltage.summary());
        p.result.put("voltageReadsUnder9V", lowVoltage);
        return p.end();
    }

    protected Map<String, Object> i2cReads(String name, int parallel, boolean replaced, int iterations) {
        Phase p = new Phase(name, parallel);
        Samples period = new Samples();
        PhotonI2c.Readers readers = i2c.readers(replaced, PhotonI2c.PERIOD_NS);
        readers.start();
        boolean finished = false;
        try {
            for (int i = 0; i < iterations && opModeIsActive(); i++) {
                period.tick(System.nanoTime());
                bulkRead();
                for (PhotonChannel c : rig.channels) p.write(c);
                readers.check();
                p.step = i;
                if (i % VERIFY_EVERY == VERIFY_EVERY - 1) p.verifyAll(false);
                report.status("%s %d/%d", name, i + 1, iterations);
                report.sendIfDue();
            }
            finished = true;
        } finally {
            readers.halt(finished);
        }
        p.result.put("iterations", iterations);
        p.result.put("loopPeriodMs", period.summary());
        p.result.put("i2cReplacedReader", replaced);
        p.result.put("i2cReaders", readers.summary());
        Map<String, Object> replacedLeft = new LinkedHashMap<>();
        if (replaced) {
            for (PhotonI2c.Device d : i2c.devices) replacedLeft.put(d.name, PhotonProbe.unfinished(PhotonProbe.replaced(d.module())));
        }
        p.result.put("replacedPendingAfter", replacedLeft);
        return p.end();
    }

    protected void bulkRead() {
        for (LynxModule h : rig.hubs) {
            h.clearBulkCache();
            h.getBulkData();
        }
    }

    static final class PendingMonitor extends Thread {
        static final double[] AGE_BUCKETS_MS = {10, 25, 50, 100, 250};

        private final List<LynxModule> hubs;
        private final long periodMs;
        private final Thread opModeThread;
        private final double[] maxMs;
        private final int[] maxEntries;
        private final int[] maxStandIns;
        private final long[] ages = new long[AGE_BUCKETS_MS.length + 1];
        private volatile boolean running = true;
        private volatile Throwable failure;

        PendingMonitor(List<LynxModule> hubs, long periodMs, Thread opModeThread) {
            super("PhotonPendingMonitor");
            this.hubs = hubs;
            this.periodMs = periodMs;
            this.opModeThread = opModeThread;
            maxMs = new double[hubs.size()];
            maxEntries = new int[hubs.size()];
            maxStandIns = new int[hubs.size()];
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                while (running && opModeThread.isAlive()) {
                    for (int i = 0; i < hubs.size(); i++) {
                        double oldest = PhotonProbe.oldestPendingMs(hubs.get(i));
                        maxMs[i] = Math.max(maxMs[i], oldest);
                        maxEntries[i] = Math.max(maxEntries[i], PhotonProbe.pendingEntries(hubs.get(i)));
                        maxStandIns[i] = Math.max(maxStandIns[i], PhotonProbe.standIns(hubs.get(i)));
                        if (oldest > 0) ages[bucket(oldest)]++;
                    }
                    Thread.sleep(periodMs);
                }
            } catch (Throwable t) {
                failure = t;
            }
        }

        void halt() {
            running = false;
            try {
                join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("interrupted stopping the pending-write monitor", e);
            }
            if (failure != null) throw new IllegalStateException("the pending-write monitor failed", failure);
        }

        Map<String, Object> maxMs(PhotonRig rig) {
            Map<String, Object> m = new LinkedHashMap<>();
            for (int i = 0; i < hubs.size(); i++) m.put(rig.key(hubs.get(i)), maxMs[i]);
            return m;
        }

        Map<String, Object> maxEntries(PhotonRig rig) {
            Map<String, Object> m = new LinkedHashMap<>();
            for (int i = 0; i < hubs.size(); i++) m.put(rig.key(hubs.get(i)), maxEntries[i]);
            return m;
        }

        Map<String, Object> maxStandIns(PhotonRig rig) {
            Map<String, Object> m = new LinkedHashMap<>();
            for (int i = 0; i < hubs.size(); i++) m.put(rig.key(hubs.get(i)), maxStandIns[i]);
            return m;
        }

        /** Samples of the oldest unacknowledged write's age, any hub, by bucket ("<10" ... ">=250" ms). */
        Map<String, Object> ageHistogram() {
            Map<String, Object> m = new LinkedHashMap<>();
            for (int i = 0; i < ages.length; i++) {
                m.put(i < AGE_BUCKETS_MS.length ? "<" + (int) AGE_BUCKETS_MS[i] : ">=" + (int) AGE_BUCKETS_MS[i - 1], ages[i]);
            }
            return m;
        }

        private static int bucket(double ms) {
            int i = 0;
            while (i < AGE_BUCKETS_MS.length && ms >= AGE_BUCKETS_MS[i]) i++;
            return i;
        }
    }

    final class Poller extends Thread {
        private final Thread opModeThread;
        private final Samples.Synced commandMs = Samples.synced();
        private volatile Throwable failure;
        private volatile boolean running = true;

        Poller(Thread opModeThread) {
            super("PhotonBenchPoller");
            this.opModeThread = opModeThread;
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                long next = System.nanoTime();
                while (running && opModeThread.isAlive()) {
                    long t0 = System.nanoTime();
                    new LynxGetBulkInputDataCommand(rig.controlHub).sendReceive();
                    commandMs.add((System.nanoTime() - t0) / 1e6);
                    next += POLL_NS;
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
            if (t != null) throw new IllegalStateException("the poller thread failed", t);
        }

        void halt() {
            running = false;
            try {
                join(250);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("interrupted while stopping the poller", e);
            }
            if (isAlive()) throw new IllegalStateException("the poller was still running 250 ms after it was told to stop");
            check();
        }

        Map<String, Object> summary() {
            return commandMs.summary();
        }
    }
}
