package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.seattlesolvers.solverslib.photon.PhotonCore;
import com.seattlesolvers.solverslib.photon.PhotonLynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Photon's write parallelization against everything this code does with hubs: write costs and read-backs on each hub,
 * bursts past the parallel limit, unparallelized commands right after parallel writes, bulk/voltage/I2C reads, a
 * TransferCounter-style poller thread, and STOP with writes in flight (the next run's initialState shows what it left).
 */
@TeleOp(name = "Bench: Photon", group = "Test")
public class PhotonBench extends LinearOpMode {
    private static final String[] CH_MOTORS = {"fl", "fr", "bl", "br"};
    private static final String[] CH_SERVOS = {"benchServo", "benchServo2"};
    private static final String[] EH_MOTORS = {"efl", "efr", "ebl", "ebr"};
    private static final String[] EH_SERVOS = {"eServo", "eServo2"};
    private static final double POWER_A = 0.2;
    private static final double POWER_B = 0.25;
    private static final double POSITION_A = 0.4;
    private static final double POSITION_B = 0.6;
    private static final int VERIFY_EVERY = 10;
    private static final double POWER_TOLERANCE = 0.01;
    private static final double PULSE_TOLERANCE_US = 5;
    private static final long DRAIN_TIMEOUT_NS = 500_000_000L;
    private static final long POLL_NS = 10_000_000L;

    private static final class Mode {
        final String name;
        final int parallel;
        final boolean poller;

        Mode(String name, int parallel, boolean poller) {
            this.name = name;
            this.parallel = parallel;
            this.poller = poller;
        }
    }

    private static final Mode[] TIMING_MODES = {
            new Mode("off", 0, false),
            new Mode("on8", 8, false),
            new Mode("on4", 4, false),
            new Mode("on2", 2, false),
            new Mode("off+poller", 0, true),
            new Mode("on8+poller", 8, true),
    };

    private final class Channel {
        final String name;
        final LynxModule hub;
        final DcMotorEx motor;
        final ServoImplEx servo;

        Channel(String name, LynxModule hub, DcMotorEx motor, ServoImplEx servo) {
            this.name = name;
            this.hub = hub;
            this.motor = motor;
            this.servo = servo;
        }

        void write(int step) {
            if (motor != null) motor.setPower(step % 2 == 0 ? POWER_A : POWER_B);
            else servo.setPosition(step % 2 == 0 ? POSITION_A : POSITION_B);
        }

        boolean at(int step) {
            if (motor != null) {
                return Math.abs(HubRegisters.motorPower(hub, motor.getPortNumber()) - (step % 2 == 0 ? POWER_A : POWER_B))
                        <= POWER_TOLERANCE;
            }
            return Math.abs(HubRegisters.servoPulseWidth(hub, servo.getPortNumber()) - pulse(step)) <= PULSE_TOLERANCE_US;
        }

        boolean enabled() {
            return motor != null ? HubRegisters.motorEnabled(hub, motor.getPortNumber())
                    : HubRegisters.servoEnabled(hub, servo.getPortNumber());
        }

        double pulse(int step) {
            PwmControl.PwmRange range = servo.getPwmRange();
            double position = step % 2 == 0 ? POSITION_A : POSITION_B;
            return range.usPulseLower + position * (range.usPulseUpper - range.usPulseLower);
        }

        Map<String, Object> state() {
            Map<String, Object> m = new LinkedHashMap<>();
            if (motor != null) m.put("power", HubRegisters.motorPower(hub, motor.getPortNumber()));
            else m.put("pulseUs", HubRegisters.servoPulseWidth(hub, servo.getPortNumber()));
            m.put("enabled", enabled());
            return m;
        }
    }

    private BenchReport report;
    private final List<LynxModule> hubs = new ArrayList<>();
    private LynxModule controlHub;
    private LynxModule expansionHub;
    private final List<Channel> channels = new ArrayList<>();
    private GoBildaPinpointDriver pinpoint;
    private OctoQuad octoQuad;
    private int loops;

    @Override
    public void runOpMode() {
        String runId = BenchIO.param(BenchIO.params(), "runId", "1");
        report = new BenchReport("Photon", "photon_" + runId);
        loops = (int) BenchIO.param(report.params, "loops", 500);
        boolean wantExpansion = BenchIO.param(report.params, "usbExpansion", 0) != 0;
        boolean wantI2c = BenchIO.param(report.params, "i2c", 0) != 0;

        for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
            if (LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) controlHub = m;
            else expansionHub = m;
            hubs.add(m);
        }
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("loops", loops);
        List<Map<String, Object>> hubInfo = new ArrayList<>();
        for (LynxModule m : hubs) {
            Map<String, Object> h = new LinkedHashMap<>();
            h.put("serial", m.getSerialNumber().toString());
            h.put("address", m.getModuleAddress());
            h.put("parent", m.isParent());
            h.put("class", m.getClass().getName());
            h.put("photonSwapped", m instanceof PhotonLynxModule);
            hubInfo.add(h);
        }
        info.put("hubs", hubInfo);
        info.put("photonControlHub", PhotonCore.CONTROL_HUB == controlHub && controlHub != null);
        info.put("photonExpansionHub", expansionHub != null && PhotonCore.EXPANSION_HUB == expansionHub);
        report.put("info", info);
        report.save();
        if (controlHub == null) throw new IllegalStateException("Bench: Photon found no Control Hub; " + BenchIO.RUNNER_HINT);
        if (wantExpansion != (expansionHub != null) || hubs.size() > (wantExpansion ? 2 : 1)) {
            throw new IllegalStateException("Bench: Photon expected " + (wantExpansion ? "the Control Hub and one USB Expansion Hub"
                    : "the Control Hub alone") + ", found " + hubs.size() + " hub(s); " + BenchIO.RUNNER_HINT);
        }
        if (expansionHub != null && !expansionHub.isParent()) {
            throw new IllegalStateException("the Expansion Hub is on RS485: Photon only parallelizes USB hubs; connect it by USB alone");
        }
        for (LynxModule m : hubs) {
            if (!(m instanceof PhotonLynxModule)) {
                throw new IllegalStateException("Photon didn't replace hub " + m.getModuleAddress() + "'s LynxModule (it is "
                        + m.getClass().getName() + "): is org.solverslib:photon in the installed APK?");
            }
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        addChannels(controlHub, CH_MOTORS, CH_SERVOS);
        if (expansionHub != null) addChannels(expansionHub, EH_MOTORS, EH_SERVOS);
        if (wantI2c) {
            pinpoint = BenchIO.require(hardwareMap, GoBildaPinpointDriver.class, BettaConstants.pinpoint.name, BenchIO.RUNNER_HINT);
            octoQuad = BenchIO.require(hardwareMap, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
        }

        Map<String, Object> initial = new LinkedHashMap<>();
        for (Channel c : channels) initial.put(c.name, c.state());
        report.put("initialState", initial);
        report.save();
        if (!report.holdUntilStart(this)) return;

        Map<String, Object> timing = new LinkedHashMap<>();
        report.put("timing", timing);
        Map<String, Object> burst = new LinkedHashMap<>();
        report.put("burst", burst);
        Map<String, Object> reads = new LinkedHashMap<>();
        report.put("reads", reads);
        try {
            for (Mode mode : TIMING_MODES) {
                if (!opModeIsActive()) return;
                timing.put(mode.name, timingMode(mode));
                report.save();
            }
            for (int parallel : new int[] {0, 8, 2}) {
                if (!opModeIsActive()) return;
                burst.put(parallel == 0 ? "off" : "on" + parallel, burst(parallel));
                report.save();
            }
            for (int parallel : new int[] {0, 8}) {
                if (!opModeIsActive()) return;
                report.put("interleave" + (parallel == 0 ? "Off" : "On8"), interleave(parallel));
                report.save();
            }
            for (int parallel : new int[] {0, 8}) {
                if (!opModeIsActive()) return;
                reads.put(parallel == 0 ? "off" : "on8", reads(parallel));
                report.save();
            }
        } catch (RuntimeException e) {
            PhotonCore.disable();
            for (Channel c : channels) if (c.motor != null) c.motor.setPower(0);
            throw e;
        }

        // Left energized with Photon on and writes undrained, so STOP itself runs with writes in flight.
        report.phase("stopUnderPhoton");
        enable(8);
        for (Channel c : channels) c.write(0);
        report.done(String.format("write block p50 %.2f ms off, %.2f ms with Photon (8 parallel); left energized for STOP",
                p50(timing, "off", "writeBlockMs"), p50(timing, "on8", "writeBlockMs")));
        report.holdUntilStop(this);
    }

    private void addChannels(LynxModule hub, String[] motorNames, String[] servoNames) {
        for (String name : motorNames) {
            DcMotorEx m = BenchIO.require(hardwareMap, DcMotorEx.class, name, BenchIO.RUNNER_HINT);
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            channels.add(new Channel(name, hub, m, null));
        }
        for (String name : servoNames) {
            ServoImplEx s = BenchIO.require(hardwareMap, ServoImplEx.class, name, BenchIO.RUNNER_HINT);
            s.setDirection(Servo.Direction.FORWARD);
            channels.add(new Channel(name, hub, null, s));
        }
    }

    private void enable(int parallel) {
        if (parallel > 0) {
            PhotonCore.experimental.setMaximumParallelCommands(parallel);
            PhotonCore.enable();
        } else {
            PhotonCore.disable();
        }
    }

    private Map<String, Object> timingMode(Mode mode) {
        report.phase("timing " + mode.name);
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("drainBeforeMs", drain());
        enable(mode.parallel);
        Poller poller = mode.poller ? new Poller(Thread.currentThread()) : null;
        if (poller != null) poller.start();

        Samples period = new Samples();
        Samples bulk = new Samples();
        Samples motorWrite = new Samples();
        Samples servoWrite = new Samples();
        Samples writeBlock = new Samples();
        Map<LynxModule, Samples> perHubBlock = new LinkedHashMap<>();
        for (LynxModule h : hubs) perHubBlock.put(h, new Samples());
        int checks = 0;
        int mismatches = 0;
        int maxInFlight = 0;
        long lastStartNs = 0;
        boolean lastWasVerify = false;
        int done = 0;
        try {
            for (int i = 0; i < loops && opModeIsActive(); i++) {
                long start = System.nanoTime();
                if (lastStartNs != 0 && !lastWasVerify) period.addNanos(start - lastStartNs);
                lastStartNs = start;
                long t = System.nanoTime();
                bulkRead();
                bulk.addNanos(System.nanoTime() - t);
                long blockStart = System.nanoTime();
                for (LynxModule h : hubs) {
                    long hubStart = System.nanoTime();
                    for (Channel c : channels) {
                        if (c.hub != h) continue;
                        t = System.nanoTime();
                        c.write(i);
                        (c.motor != null ? motorWrite : servoWrite).addNanos(System.nanoTime() - t);
                    }
                    perHubBlock.get(h).addNanos(System.nanoTime() - hubStart);
                }
                writeBlock.addNanos(System.nanoTime() - blockStart);
                maxInFlight = Math.max(maxInFlight, inFlight());
                lastWasVerify = i % VERIFY_EVERY == VERIFY_EVERY - 1;
                if (lastWasVerify) {
                    checks++;
                    if (!allAt(i)) mismatches++;
                }
                if (poller != null) poller.check();
                done = i;
                report.status("timing %s %d/%d", mode.name, i + 1, loops);
                report.sendIfDue();
            }
        } finally {
            if (poller != null) poller.halt();
        }
        r.put("drainAfterMs", drain());
        r.put("checks", checks);
        r.put("mismatches", mismatches);
        r.put("allAtLastAfterDrain", allAt(done));
        r.put("maxInFlight", maxInFlight);
        r.put("loopPeriodMs", period.summary());
        r.put("bulkReadMs", bulk.summary());
        r.put("motorWriteMs", motorWrite.summary());
        r.put("servoWriteMs", servoWrite.summary());
        r.put("writeBlockMs", writeBlock.summary());
        Map<String, Object> hubBlocks = new LinkedHashMap<>();
        for (Map.Entry<LynxModule, Samples> e : perHubBlock.entrySet()) {
            hubBlocks.put(String.valueOf(e.getKey().getModuleAddress()), e.getValue().summary());
        }
        r.put("writeBlockPerHubMs", hubBlocks);
        if (poller != null) r.put("pollerCommandMs", poller.summary());
        r.put("notResponding", notResponding());
        PhotonCore.disable();
        return r;
    }

    /** Every channel written, then read back at once with nothing drained: lost or reordered writes show here first. */
    private Map<String, Object> burst(int parallel) {
        String name = parallel == 0 ? "off" : "on" + parallel;
        report.phase("burst " + name);
        drain();
        enable(parallel);
        int iterations = Math.max(50, loops / 2);
        int mismatches = 0;
        Samples burstMs = new Samples();
        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            long t = System.nanoTime();
            for (Channel c : channels) c.write(i);
            burstMs.addNanos(System.nanoTime() - t);
            if (!allAt(i)) mismatches++;
            report.status("burst %s %d/%d", name, i + 1, iterations);
            report.sendIfDue();
        }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("iterations", iterations);
        r.put("mismatches", mismatches);
        r.put("burstMs", burstMs.summary());
        r.put("drainAfterMs", drain());
        r.put("notResponding", notResponding());
        PhotonCore.disable();
        return r;
    }

    /** A parallel write, then an enable toggle Photon sends the normal way, then a read-back of both. */
    private Map<String, Object> interleave(int parallel) {
        report.phase("interleave " + (parallel == 0 ? "off" : "on" + parallel));
        drain();
        enable(parallel);
        int iterations = Math.max(30, loops / 5);
        int valueMismatches = 0;
        int enableMismatches = 0;
        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            for (Channel c : channels) {
                c.write(i);
                if (c.motor != null) {
                    c.motor.setMotorDisable();
                    c.motor.setMotorEnable();
                } else {
                    c.servo.setPwmDisable();
                    c.servo.setPwmEnable();
                }
            }
            for (Channel c : channels) {
                if (!c.at(i)) valueMismatches++;
                if (!c.enabled()) enableMismatches++;
            }
            report.status("interleave %d/%d", i + 1, iterations);
            report.sendIfDue();
        }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("iterations", iterations);
        r.put("channelChecks", iterations * channels.size());
        r.put("valueMismatches", valueMismatches);
        r.put("enableMismatches", enableMismatches);
        r.put("drainAfterMs", drain());
        r.put("notResponding", notResponding());
        PhotonCore.disable();
        return r;
    }

    /** Bulk, voltage and (with i2c) Pinpoint and OctoQuad reads between parallel writes. */
    private Map<String, Object> reads(int parallel) {
        String name = parallel == 0 ? "off" : "on" + parallel;
        report.phase("reads " + name);
        drain();
        enable(parallel);
        Samples bulk = new Samples();
        Samples voltage = new Samples();
        Samples pinpointMs = new Samples();
        Samples octoQuadMs = new Samples();
        int pinpointNotReady = 0;
        int octoQuadCrcFailures = 0;
        int octoQuadInvalid = 0;
        OctoQuad.LocalizerDataBlock block = new OctoQuad.LocalizerDataBlock();
        int mismatches = 0;
        int checks = 0;
        int iterations = Math.max(100, loops / 2);
        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            for (Channel c : channels) c.write(i);
            long t = System.nanoTime();
            bulkRead();
            bulk.addNanos(System.nanoTime() - t);
            for (LynxModule h : hubs) {
                t = System.nanoTime();
                h.getInputVoltage(VoltageUnit.VOLTS);
                voltage.addNanos(System.nanoTime() - t);
            }
            if (pinpoint != null) {
                t = System.nanoTime();
                pinpoint.update();
                pinpointMs.addNanos(System.nanoTime() - t);
                if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) pinpointNotReady++;
                t = System.nanoTime();
                octoQuad.readLocalizerData(block);
                octoQuadMs.addNanos(System.nanoTime() - t);
                if (!block.crcOk) octoQuadCrcFailures++;
                else if (!block.isDataValid()) octoQuadInvalid++;
            }
            if (i % VERIFY_EVERY == VERIFY_EVERY - 1) {
                checks++;
                if (!allAt(i)) mismatches++;
            }
            report.status("reads %s %d/%d", name, i + 1, iterations);
            report.sendIfDue();
        }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("iterations", iterations);
        r.put("bulkReadMs", bulk.summary());
        r.put("inputVoltageMs", voltage.summary());
        if (pinpoint != null) {
            r.put("pinpointUpdateMs", pinpointMs.summary());
            r.put("pinpointNotReady", pinpointNotReady);
            r.put("octoQuadReadMs", octoQuadMs.summary());
            r.put("octoQuadCrcFailures", octoQuadCrcFailures);
            r.put("octoQuadInvalid", octoQuadInvalid);
        }
        r.put("checks", checks);
        r.put("mismatches", mismatches);
        r.put("drainAfterMs", drain());
        r.put("notResponding", notResponding());
        PhotonCore.disable();
        return r;
    }

    private void bulkRead() {
        for (LynxModule h : hubs) {
            h.clearBulkCache();
            h.getBulkData();
        }
    }

    private boolean allAt(int step) {
        for (Channel c : channels) if (!c.at(step)) return false;
        return true;
    }

    private int inFlight() {
        int n = 0;
        for (LynxModule h : hubs) n += ((PhotonLynxModule) h).getUnfinishedCommands().size();
        return n;
    }

    /** Waits for the hubs to acknowledge every command in flight; the time it took, or NaN if they never emptied. */
    private double drain() {
        long t0 = System.nanoTime();
        while (inFlight() > 0) {
            if (System.nanoTime() - t0 > DRAIN_TIMEOUT_NS) return Double.NaN;
            Thread.yield();
        }
        return (System.nanoTime() - t0) / 1e6;
    }

    private List<Integer> notResponding() {
        List<Integer> out = new ArrayList<>();
        for (LynxModule h : hubs) if (h.isNotResponding()) out.add(h.getModuleAddress());
        return out;
    }

    private static double p50(Map<String, Object> results, String mode, String key) {
        Object r = results.get(mode);
        if (!(r instanceof Map)) return Double.NaN;
        Object s = ((Map<?, ?>) r).get(key);
        if (!(s instanceof Map)) return Double.NaN;
        Object p50 = ((Map<?, ?>) s).get("p50");
        return p50 instanceof Number ? ((Number) p50).doubleValue() : Double.NaN;
    }

    /** TransferCounter's pattern: its own bulk-read command on the Control Hub every 10 ms, beside the OpMode thread. */
    private final class Poller extends Thread {
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
                    new LynxGetBulkInputDataCommand(controlHub).sendReceive();
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
