package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "Bench: Analog Poll", group = "Test")
public class AnalogPollBench extends OpMode {
    private static final String HINT = "run it on laser_rangefinder.xml, whose analog0 and analog1 are the two rangefinders";
    private static final long CHUNK_NS = 250_000_000L;
    private static final long PACKET_INTERVAL_NS = 100_000_000L;
    private static final double CPU_MS_PER_LOOP = 2.7;

    private enum Poll { NONE, BULK, TWO_READS }

    private static final class Profile {
        final String name;
        final String label;
        final int hubCommands;

        Profile(String name, String label, int hubCommands) {
            this.name = name;
            this.label = label;
            this.hubCommands = hubCommands;
        }
    }

    private static final Profile[] PROFILES = {
            new Profile("idle", "idle", 0),
            new Profile("octoQuadDriving", "OctoQuad driving", 6),
            new Profile("pinpointDriving", "Pinpoint driving", 8)};

    private abstract static class Phase {
        final String name;
        final Map<String, Object> result = new LinkedHashMap<>();
        private BenchIO.Gc gc;
        private BenchIO.Cpu cpu;
        long loops;

        Phase(String name) {
            this.name = name;
        }

        void begin() {}

        abstract boolean step() throws InterruptedException, LynxNackException;

        void end() {}

        final void start() {
            gc = BenchIO.Gc.now();
            cpu = BenchIO.Cpu.now();
            begin();
        }

        final void finish() {
            end();
            result.put("gc", BenchIO.Gc.now().since(gc, loops));
            result.put("cpu", BenchIO.Cpu.now().since(cpu));
        }
    }

    private static final class Series {
        private long[] nanos = new long[1024];
        private int[] millivolts = new int[1024];
        private int[] chunk = new int[1024];
        private int count;
        private final Samples newValueIntervals = new Samples();
        private double newFraction;
        private int min;
        private int max;

        void add(long tNanos, int mv, int chunkIndex) {
            if (count == nanos.length) {
                nanos = Arrays.copyOf(nanos, count * 2);
                millivolts = Arrays.copyOf(millivolts, count * 2);
                chunk = Arrays.copyOf(chunk, count * 2);
            }
            nanos[count] = tNanos;
            millivolts[count] = mv;
            chunk[count] = chunkIndex;
            count++;
        }

        Map<String, Object> analyze() {
            Samples readPeriods = new Samples();
            int pairs = 0;
            int changes = 0;
            long lastChange = -1;
            min = Integer.MAX_VALUE;
            max = Integer.MIN_VALUE;
            for (int i = 0; i < count; i++) {
                min = Math.min(min, millivolts[i]);
                max = Math.max(max, millivolts[i]);
                if (i == 0 || chunk[i] != chunk[i - 1]) {
                    lastChange = -1;
                    continue;
                }
                pairs++;
                readPeriods.addNanos(nanos[i] - nanos[i - 1]);
                if (millivolts[i] != millivolts[i - 1]) {
                    changes++;
                    if (lastChange >= 0) newValueIntervals.addNanos(nanos[i] - lastChange);
                    lastChange = nanos[i];
                }
            }
            newFraction = pairs == 0 ? 0 : (double) changes / pairs;
            double[] ms = new double[count];
            for (int i = 0; i < count; i++) ms[i] = nanos[i] / 1e6;
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("reads", count);
            m.put("readPeriodMs", readPeriods.summary());
            m.put("readsWithNewValueFraction", newFraction);
            m.put("newValueIntervalMs", newValueIntervals.summary());
            m.put("newValueIntervalHistogram", newValueIntervals.histogram(0.5, 80));
            m.put("minMv", min);
            m.put("maxMv", max);
            m.put("tMs", ms);
            m.put("mv", Arrays.copyOf(millivolts, count));
            return m;
        }

        String line(String label) {
            if (newValueIntervals.count() == 0) {
                return String.format(Locale.US, "%s: never changed (%d mV)", label, min);
            }
            return String.format(Locale.US, "%s: every %.1f ms (p10 %.1f), %.0f%% reads new, %d-%d mV", label,
                    newValueIntervals.percentile(50), newValueIntervals.percentile(10), 100 * newFraction, min, max);
        }
    }

    private BenchReport report;
    private final List<Phase> phases = new ArrayList<>();
    private final List<String> summary = new ArrayList<>();
    private int phaseIndex = -1;

    private LynxModule hub;
    private AnalogInput sensorA;
    private AnalogInput sensorB;
    private String nameA;
    private String nameB;
    private int portA;
    private int portB;
    private double refreshSeconds;
    private double loopSeconds;
    private long pollNs;
    private Poller poller;

    @Override
    public void init() {
        report = new BenchReport("Analog Poll", "analog_poll");
        nameA = BenchIO.param(report.params, "sensorA", "analog0");
        nameB = BenchIO.param(report.params, "sensorB", "analog1");
        refreshSeconds = BenchIO.param(report.params, "refreshSeconds", 4);
        loopSeconds = BenchIO.param(report.params, "loopSeconds", 6);
        pollNs = (long) (BenchIO.param(report.params, "pollMs", 10) * 1e6);
        sensorA = BenchIO.require(hardwareMap, AnalogInput.class, nameA, HINT);
        sensorB = BenchIO.require(hardwareMap, AnalogInput.class, nameB, HINT);
        for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
            if (portOn(m, sensorA) >= 0) hub = m;
        }
        if (hub == null) throw new IllegalStateException(nameA + " (" + sensorA.getConnectionInfo() + ") is on no REV hub in the config");
        portA = portOn(hub, sensorA);
        portB = portOn(hub, sensorB);
        if (portB < 0) {
            throw new IllegalStateException(nameB + " must be on the same hub as " + nameA + ", is on " + sensorB.getConnectionInfo());
        }
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Map<String, Object> info = new LinkedHashMap<>();
        info.put("hub", hub.getConnectionInfo());
        info.put("hubIsControlHub", hub.isParent());
        info.put(nameA + "Port", portA);
        info.put(nameB + "Port", portB);
        info.put("pollMs", pollNs / 1e6);
        info.put("cpuMsPerLoop", CPU_MS_PER_LOOP);
        info.put("devices", BenchIO.deviceNames(hardwareMap));
        report.put("info", info);

        phases.add(new AdcRefresh(nameA, LynxGetADCCommand.Channel.user(portA)));
        phases.add(new AdcRefresh(nameB, LynxGetADCCommand.Channel.user(portB)));
        phases.add(new AdcRefresh("battery monitor", LynxGetADCCommand.Channel.BATTERY_MONITOR));
        phases.add(new AdcRefresh("5 V monitor", LynxGetADCCommand.Channel.FIVE_VOLT_MONITOR));
        phases.add(new BulkRefresh());
        for (Profile profile : PROFILES) {
            for (Poll poll : Poll.values()) phases.add(new LoopPhase(profile, poll));
        }
    }

    private static int portOn(LynxModule module, AnalogInput input) {
        for (int port = LynxAnalogInputController.apiPortFirst; port <= LynxAnalogInputController.apiPortLast; port++) {
            if (input.getConnectionInfo().equals(module.getConnectionInfo() + "; analog port " + port)) return port;
        }
        return -1;
    }

    @Override
    public void init_loop() {
        show();
    }

    @Override
    public void start() {
        next();
    }

    @Override
    public void loop() {
        if (phaseIndex < phases.size()) {
            Phase p = phases.get(phaseIndex);
            if (step(p)) {
                p.finish();
                report.endPhase(p.name, p.result);
                next();
            }
        }
        show();
    }

    @Override
    public void stop() {
        if (poller != null) poller.halt();
    }

    private boolean step(Phase p) {
        try {
            return p.step();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted during " + p.name, e);
        } catch (LynxNackException e) {
            throw new IllegalStateException("the hub refused a command during " + p.name, e);
        }
    }

    private void next() {
        phaseIndex++;
        if (phaseIndex < phases.size()) {
            Phase p = phases.get(phaseIndex);
            report.phase(p.name + " (" + (phaseIndex + 1) + "/" + phases.size() + ")");
            p.start();
        } else {
            buildSummary();
            report.put("summaryLines", summary);
            report.done(phases.size() + " phases");
        }
    }

    private void show() {
        report.addTo(telemetry);
        for (String line : summary) telemetry.addLine(line);
        report.sendIfDue(PACKET_INTERVAL_NS, packet -> {
            for (String line : summary) packet.addLine(line);
        });
    }

    private void buildSummary() {
        summary.add("New value every (p50, p10), % of back-to-back reads new, range:");
        for (Phase p : phases) {
            if (p instanceof AdcRefresh) {
                AdcRefresh r = (AdcRefresh) p;
                summary.add(r.series.line(r.label + " own read"));
            } else if (p instanceof BulkRefresh) {
                BulkRefresh r = (BulkRefresh) p;
                summary.add(r.a.line(nameA + " bulk read"));
                summary.add(r.b.line(nameB + " bulk read"));
            }
        }
        summary.add(String.format(Locale.US, "Loop mean ms: no poller | %.0f ms bulk poller | %.0f ms two-read poller",
                pollNs / 1e6, pollNs / 1e6));
        for (Profile profile : PROFILES) {
            LoopPhase none = loopPhase(profile, Poll.NONE);
            LoopPhase bulk = loopPhase(profile, Poll.BULK);
            LoopPhase two = loopPhase(profile, Poll.TWO_READS);
            double base = none.period.mean();
            summary.add(String.format(Locale.US, "%s: %.1f | %.1f (%+.1f) | %.1f (%+.1f)", profile.label, base,
                    bulk.period.mean(), bulk.period.mean() - base, two.period.mean(), two.period.mean() - base));
            summary.add(String.format(Locale.US, "  new %s values/s: loop alone %.0f, bulk poller %.0f", nameA,
                    none.newValuesPerSecond(), bulk.thread.newValuesPerSecond()));
        }
    }

    private LoopPhase loopPhase(Profile profile, Poll poll) {
        for (Phase p : phases) {
            if (p instanceof LoopPhase && ((LoopPhase) p).profile == profile && ((LoopPhase) p).poll == poll) return (LoopPhase) p;
        }
        throw new IllegalStateException("no loop phase " + profile.name + " " + poll);
    }

    private int adc(LynxGetADCCommand.Channel channel) throws InterruptedException, LynxNackException {
        return new LynxGetADCCommand(hub, channel, LynxGetADCCommand.Mode.ENGINEERING).sendReceive().getValue();
    }

    private abstract class RefreshPhase extends Phase {
        final String label;
        final Samples readMs = new Samples();
        long startNs;
        int chunkIndex;

        RefreshPhase(String name, String label) {
            super(name);
            this.label = label;
        }

        abstract void sample() throws InterruptedException, LynxNackException;

        @Override
        final boolean step() throws InterruptedException, LynxNackException {
            long chunkStart = System.nanoTime();
            if (startNs == 0) startNs = chunkStart;
            while (System.nanoTime() - chunkStart < CHUNK_NS) sample();
            chunkIndex++;
            loops++;
            double elapsed = (System.nanoTime() - startNs) / 1e9;
            report.status("%s %.0f/%.0fs", label, elapsed, refreshSeconds);
            return elapsed >= refreshSeconds;
        }
    }

    private final class AdcRefresh extends RefreshPhase {
        final LynxGetADCCommand.Channel channel;
        final Series series = new Series();

        AdcRefresh(String label, LynxGetADCCommand.Channel channel) {
            super("refresh " + label, label);
            this.channel = channel;
        }

        @Override
        void sample() throws InterruptedException, LynxNackException {
            long t0 = System.nanoTime();
            int mv = adc(channel);
            long t1 = System.nanoTime();
            readMs.addNanos(t1 - t0);
            series.add((t0 + t1) / 2 - startNs, mv, chunkIndex);
        }

        @Override
        void end() {
            result.put("what", "LynxGetADCCommand on " + channel + " back to back, " + CHUNK_NS / 1_000_000
                    + " ms per loop(); a new value's interval is counted only within one loop()");
            result.put("readMs", readMs.summary());
            result.putAll(series.analyze());
        }
    }

    private final class BulkRefresh extends RefreshPhase {
        final Series a = new Series();
        final Series b = new Series();

        BulkRefresh() {
            super("refresh bulk", "bulk read");
        }

        @Override
        void sample() throws InterruptedException, LynxNackException {
            long t0 = System.nanoTime();
            LynxGetBulkInputDataResponse r = new LynxGetBulkInputDataCommand(hub).sendReceive();
            long t1 = System.nanoTime();
            readMs.addNanos(t1 - t0);
            a.add((t0 + t1) / 2 - startNs, r.getAnalogInput(portA), chunkIndex);
            b.add((t0 + t1) / 2 - startNs, r.getAnalogInput(portB), chunkIndex);
        }

        @Override
        void end() {
            result.put("what", "LynxGetBulkInputDataCommand back to back, both rangefinder ports from each");
            result.put("readMs", readMs.summary());
            result.put(nameA, a.analyze());
            result.put(nameB, b.analyze());
        }
    }

    private final class LoopPhase extends Phase {
        final Profile profile;
        final Poll poll;
        final Samples period = new Samples();
        final Samples bulkMs = new Samples();
        final Samples commandsMs = new Samples();
        final Samples cpuMs = new Samples();
        final long cpuNs = (long) (CPU_MS_PER_LOOP * 1e6);
        Poller thread;
        long startNs;
        double lastA = Double.NaN;
        int newValues;
        double sink;

        LoopPhase(Profile profile, Poll poll) {
            super("loop " + profile.name + " " + poll.name().toLowerCase(Locale.US));
            this.profile = profile;
            this.poll = poll;
        }

        @Override
        void begin() {
            if (poll == Poll.NONE) return;
            thread = new Poller(poll, Thread.currentThread());
            poller = thread;
            thread.start();
        }

        @Override
        boolean step() throws InterruptedException, LynxNackException {
            long t0 = System.nanoTime();
            if (period.tick(t0)) startNs = t0;
            loops++;
            if (thread != null) thread.check();
            hub.clearBulkCache();
            double a = sensorA.getVoltage();
            sensorB.getVoltage();
            long t1 = System.nanoTime();
            for (int i = 0; i < profile.hubCommands; i++) adc(LynxGetADCCommand.Channel.BATTERY_MONITOR);
            long t2 = System.nanoTime();
            double acc = 0;
            int n = 0;
            while (System.nanoTime() - t2 < cpuNs) acc += Math.sin(n++ * 0.001);
            sink += acc;
            long t3 = System.nanoTime();
            bulkMs.addNanos(t1 - t0);
            commandsMs.addNanos(t2 - t1);
            cpuMs.addNanos(t3 - t2);
            if (!Double.isNaN(lastA) && a != lastA) newValues++;
            lastA = a;
            double elapsed = (t3 - startNs) / 1e9;
            report.status("%s %.0f/%.0fs", name, elapsed, loopSeconds);
            return elapsed >= loopSeconds;
        }

        double newValuesPerSecond() {
            return newValues / ((period.lastTickNs() - startNs) / 1e9);
        }

        @Override
        void end() {
            if (thread != null) {
                thread.halt();
                poller = null;
            }
            result.put("what", "a stand-in robot loop: MANUAL bulk read of both rangefinders, " + profile.hubCommands
                    + " GetADC commands standing in for odometry and drive writes, " + CPU_MS_PER_LOOP
                    + " ms of CPU for telemetry; poller " + poll);
            result.put("loops", loops);
            result.put("loopPeriodMs", period.summary());
            result.put("loopPeriodHistogram", period.histogram(0.5, 80));
            result.put("bulkReadMs", bulkMs.summary());
            result.put("hubCommandsMs", commandsMs.summary());
            result.put("cpuWorkMs", cpuMs.summary());
            result.put("newValuesPerSecond", newValuesPerSecond());
            if (thread != null) result.put("poller", thread.summary());
        }
    }

    private final class Poller extends Thread {
        private final Poll poll;
        private final Thread opModeThread;
        private final Samples period = new Samples();
        private final Samples commandMs = new Samples();
        private volatile Throwable failure;
        private volatile boolean running = true;
        private long startNs;
        private int polls;
        private int newValues;

        Poller(Poll poll, Thread opModeThread) {
            super("AnalogPoller");
            this.poll = poll;
            this.opModeThread = opModeThread;
            setDaemon(true);
        }

        @Override
        public void run() {
            try {
                startNs = System.nanoTime();
                long next = startNs;
                int lastA = 0;
                while (running && opModeThread.isAlive()) {
                    long t0 = System.nanoTime();
                    period.tick(t0);
                    int a = read();
                    commandMs.addNanos(System.nanoTime() - t0);
                    if (polls > 0 && a != lastA) newValues++;
                    lastA = a;
                    polls++;
                    next += pollNs;
                    long wait = next - System.nanoTime();
                    if (wait > 0) Thread.sleep(wait / 1_000_000, (int) (wait % 1_000_000));
                    else next = System.nanoTime();
                }
            } catch (Throwable t) {
                failure = t;
            }
        }

        private int read() throws InterruptedException, LynxNackException {
            if (poll == Poll.BULK) return new LynxGetBulkInputDataCommand(hub).sendReceive().getAnalogInput(portA);
            int a = adc(LynxGetADCCommand.Channel.user(portA));
            adc(LynxGetADCCommand.Channel.user(portB));
            return a;
        }

        void check() {
            Throwable t = failure;
            if (t != null) throw new IllegalStateException("the analog poller thread failed", t);
        }

        void halt() {
            running = false;
            try {
                join(250);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("interrupted while stopping the analog poller", e);
            }
            if (isAlive()) throw new IllegalStateException("the analog poller was still running 250 ms after it was told to stop");
            check();
        }

        double newValuesPerSecond() {
            return newValues / ((period.lastTickNs() - startNs) / 1e9);
        }

        Map<String, Object> summary() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("polls", polls);
            m.put("pollPeriodMs", period.summary());
            m.put("pollCommandMs", commandMs.summary());
            m.put("newValuesPerSecond", newValuesPerSecond());
            return m;
        }
    }
}
