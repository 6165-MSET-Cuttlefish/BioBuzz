package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * One phase per run, picked by the runner's params: a Through Bore's counts on Control Hub encoder port 2 against its
 * absolute output on OctoQuad channel 4. It never sets a motor power; only the reset phases change br's mode.
 */
@TeleOp(name = "Bench: Turret Zero", group = "Test")
public class TurretZeroBench extends LinearOpMode {
    static final String MOTOR = "br";
    static final int PORT = 2;
    static final int CHANNEL = OctoQuadBench.TURRET_CHANNEL;
    static final String FINISH_FILE = "turret_zero_finish.json";
    private static final OctoQuad.ChannelBankConfig QUADRATURE = OctoQuad.ChannelBankConfig.ALL_QUADRATURE;
    private static final long REST_NS = 500_000_000L;
    private static final long SETTLE_NS = 50_000_000L;
    private static final long FINISH_POLL_NS = 200_000_000L;
    private static final long RECHECK_MS = 100;
    private static final int SETTLE_ROWS_KEPT = 20;
    private static final int REST_READS = 5;
    private static final int ABSOLUTE_TRIES = 5;
    private static final int SPREAD_READS = 20;
    private static final int MAX_ROWS = 8000;
    private static final int MAX_RESTS = 200;

    private final OctoQuad.EncoderDataBlock block = new OctoQuad.EncoderDataBlock();
    private BenchReport report;
    private OctoQuad octoQuad;
    private DcMotor motor;
    private LynxModule hub;
    private String phase;
    private int sign;
    private double minUs;
    private double maxUs;
    private int positions;
    private long t0;

    @Override
    public void runOpMode() {
        Map<String, Object> params = BenchIO.params();
        phase = BenchIO.param(params, "phase", "init");
        report = new BenchReport("Turret Zero", "turret_zero_" + BenchIO.param(params, "run", phase));
        TurretZeroMath.Version version = TurretZeroMath.Version.valueOf(BenchIO.param(params, "version", "V1"));
        minUs = BenchIO.param(params, "minUs", version.minUs);
        maxUs = BenchIO.param(params, "maxUs", version.maxUs);
        positions = version.positions;
        sign = (int) BenchIO.param(params, "sign", 1);
        if (sign != 1 && sign != -1) throw new IllegalArgumentException("bench param sign must be 1 or -1, was " + sign);
        int reads = (int) BenchIO.param(params, "reads", 1);
        File finish = new File(BenchIO.dir(), FINISH_FILE);
        if (finish.exists() && !finish.delete()) throw new IllegalStateException("can't delete " + finish);
        octoQuad = BenchIO.require(hardwareMap, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
        motor = BenchIO.require(hardwareMap, DcMotor.class, MOTOR, BenchIO.RUNNER_HINT);
        if (motor.getPortNumber() != PORT) {
            throw new IllegalStateException(MOTOR + " is on motor port " + motor.getPortNumber() + ", not " + PORT + ": "
                    + BenchIO.RUNNER_HINT);
        }
        hub = controlHub();
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        int chipId = octoQuad.getChipId() & 0xFF;
        if (chipId != (OctoQuad.OCTOQUAD_CHIP_ID & 0xFF)) {
            throw new IllegalStateException(String.format("no OctoQuad answered as \"%s\" (chip id 0x%02x): check its I2C "
                    + "cable in Control Hub I2C port 2", OctoQuadLocalizerTest.name, chipId));
        }
        boolean pulseWidth = !phase.equals("quad");
        boolean tracksWrap = octoQuad.getSingleChannelPulseWidthTracksWrap(CHANNEL);
        report.put("info", info(tracksWrap));
        report.save();
        t0 = System.nanoTime();
        String summary;
        try {
            report.put("bank", setBank(pulseWidth));
            if (pulseWidth && tracksWrap) octoQuad.setSingleChannelPulseWidthTracksWrap(CHANNEL, false);
            report.save();
            switch (phase) {
                case "init":
                    summary = initRead(reads);
                    break;
                case "sign":
                    summary = signPhase();
                    break;
                case "reset":
                    summary = modeChanges();
                    break;
                case "reset_after":
                    summary = encoderReset();
                    break;
                case "quad":
                case "split":
                case "span":
                case "agreement":
                case "brownout":
                case "v2":
                    summary = track(finish);
                    break;
                default:
                    throw new IllegalArgumentException("bench param phase " + phase + " isn't a Bench: Turret Zero phase");
            }
        } finally {
            report.put("restore", restore(pulseWidth && tracksWrap));
            report.save();
        }
        report.done(summary);
        if (report.holdUntilStart(this)) report.holdUntilStop(this);
    }

    private LynxModule controlHub() {
        for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
            if (m.isParent()) return m;
        }
        throw new IllegalStateException("no Control Hub in the hardware map: " + BenchIO.RUNNER_HINT);
    }

    private Map<String, Object> info(boolean tracksWrap) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("phase", phase);
        m.put("chipId", octoQuad.getChipId() & 0xFF);
        m.put("firmware", octoQuad.getFirmwareVersionString());
        m.put("uptimeMs", octoQuad.getUptimeMs());
        m.put("bankAsFound", octoQuad.getChannelBankConfig().name());
        OctoQuad.ChannelPulseWidthParams pw = octoQuad.getSingleChannelPulseWidthParams(CHANNEL);
        m.put("pulseWidthParams", new int[] {pw.min_length_us, pw.max_length_us});
        m.put("tracksWrapAsFound", tracksWrap);
        m.put("motor", MOTOR + " on Control Hub motor port " + motor.getPortNumber() + ", " + motor.getMotorType().getName());
        m.put("modeAtInit", motor.getMode().name());
        m.put("sign", sign);
        m.put("pulseRangeUs", new double[] {minUs, maxUs});
        m.put("positions", positions);
        return m;
    }

    private Map<String, Object> setBank(boolean pulseWidth) {
        OctoQuad.ChannelBankConfig target = pulseWidth ? OctoQuadBench.MIXED_BANKS : QUADRATURE;
        Map<String, Object> m = new LinkedHashMap<>();
        long set = System.nanoTime();
        octoQuad.setChannelBankConfig(target);
        OctoQuad.ChannelBankConfig readBack = octoQuad.getChannelBankConfig();
        m.put("set", target.name());
        m.put("readBack", readBack.name());
        if (readBack != target) {
            throw new IllegalStateException("the OctoQuad reads back " + readBack + " after setChannelBankConfig(" + target
                    + ")");
        }
        if (!pulseWidth) return m;
        List<double[]> first = new ArrayList<>();
        Double firstPlausibleMs = null;
        while (System.nanoTime() - set < SETTLE_NS) {
            octoQuad.readAllEncoderData(block);
            double ms = (System.nanoTime() - set) / 1e6;
            int pulse = block.positions[CHANNEL];
            if (first.size() < SETTLE_ROWS_KEPT) first.add(new double[] {ms, pulse, block.crcOk ? 1 : 0});
            if (firstPlausibleMs == null && block.crcOk && TurretZeroMath.pulsePlausible(pulse, minUs, maxUs)) {
                firstPlausibleMs = ms;
            }
        }
        m.put("firstReadsColumns", new String[] {"msAfterSet", "channel" + CHANNEL, "crcOk"});
        m.put("firstReads", first);
        m.put("firstPlausibleMs", firstPlausibleMs);
        return m;
    }

    private Map<String, Object> restore(boolean tracksWrapChanged) {
        boolean interrupted = Thread.interrupted();
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("stopInterruptCleared", interrupted);
        if (tracksWrapChanged) octoQuad.setSingleChannelPulseWidthTracksWrap(CHANNEL, true);
        octoQuad.setChannelBankConfig(QUADRATURE);
        m.put("bank", octoQuad.getChannelBankConfig().name());
        if (interrupted) Thread.currentThread().interrupt();
        return m;
    }

    private int hubCount() {
        hub.clearBulkCache();
        return motor.getController().getMotorCurrentPosition(PORT);
    }

    private int hubCountAfterAWhile() {
        sleep(RECHECK_MS);
        return hubCount();
    }

    private double toDeg(int pulse) {
        return TurretZeroMath.pulsePlausible(pulse, minUs, maxUs)
                ? TurretZeroMath.pulseToEncoderDeg(pulse, minUs, maxUs, positions) : Double.NaN;
    }

    /** The INIT read the robot would make: readSinglePosition_Caching's first call in AUTO returns the empty cache. */
    private Map<String, Object> absoluteRead() {
        Map<String, Object> m = new LinkedHashMap<>();
        int count = hubCount();
        int tries = 0;
        int pulse = 0;
        boolean ok = false;
        while (!ok && tries < ABSOLUTE_TRIES) {
            octoQuad.readAllEncoderData(block);
            pulse = block.positions[CHANNEL];
            ok = block.crcOk && TurretZeroMath.pulsePlausible(pulse, minUs, maxUs);
            tries++;
        }
        double deg = ok ? toDeg(pulse) : Double.NaN;
        m.put("count", count);
        m.put("pulseUs", pulse);
        m.put("tries", tries);
        m.put("valid", ok);
        m.put("absoluteDeg", deg);
        m.put("offsetDeg", ok ? TurretZeroMath.zeroOffsetDeg(deg, count, sign) : Double.NaN);
        return m;
    }

    private String initRead(int reads) {
        report.phase("init read");
        Map<String, Object> m = absoluteRead();
        report.put("init", m);
        Samples ms = new Samples();
        int[] pulses = new int[reads];
        int crcFailures = 0;
        for (int i = 0; i < reads; i++) {
            long t = System.nanoTime();
            octoQuad.readAllEncoderData(block);
            ms.addNanos(System.nanoTime() - t);
            pulses[i] = block.positions[CHANNEL];
            if (!block.crcOk) crcFailures++;
            report.sendIfDue();
        }
        m.put("readMs", ms.summary());
        m.put("pulsesUs", pulses);
        m.put("crcFailures", crcFailures);
        m.put("spreadDeg", spreadDeg(pulses, Math.min(SPREAD_READS, reads)));
        m.put("countAfter", hubCount());
        return String.format("INIT read: %s us, %.2f deg, count %s, zero offset %.2f deg", m.get("pulseUs"),
                (Double) m.get("absoluteDeg"), m.get("count"), (Double) m.get("offsetDeg"));
    }

    private double spreadDeg(int[] pulses, int n) {
        if (n == 0) return Double.NaN;
        double first = toDeg(pulses[0]);
        double lo = 0;
        double hi = 0;
        for (int i = 0; i < n; i++) {
            double e = TurretZeroMath.wrapErrorDeg(toDeg(pulses[i]), first);
            lo = Math.min(lo, e);
            hi = Math.max(hi, e);
        }
        return hi - lo;
    }

    private String signPhase() {
        report.phase("sign");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("orientation", motor.getMotorType().getOrientation().name());
        for (DcMotorSimple.Direction d : new DcMotorSimple.Direction[] {DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE}) {
            motor.setDirection(d);
            Map<String, Object> r = new LinkedHashMap<>();
            // Only with caching off does the controller send its own encoder command; in MANUAL it reads the bulk data.
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            r.put("raw", motor.getController().getMotorCurrentPosition(PORT));
            r.put("getCurrentPosition", motor.getCurrentPosition());
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            LynxModule.BulkData bulk = hub.getBulkData();
            r.put("bulk", bulk.getMotorCurrentPosition(PORT));
            r.put("bulkFake", bulk.isFake());
            m.put(d.name(), r);
        }
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        report.put("sign", m);
        return "sign: " + m.get("FORWARD") + " / " + m.get("REVERSE");
    }

    private String modeChanges() {
        report.phase("mode changes");
        Map<String, Object> m = new LinkedHashMap<>();
        report.put("reset", m);
        m.put("countAtInit", hubCount());
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.put("afterRunWithoutEncoder", new int[] {hubCount(), hubCountAfterAWhile()});
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.put("afterRunUsingEncoder", new int[] {hubCount(), hubCountAfterAWhile()});
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.put("afterBackToRunWithoutEncoder", new int[] {hubCount(), hubCountAfterAWhile()});
        m.put("absolute", absoluteRead());
        return "mode changes: " + m;
    }

    private String encoderReset() {
        report.phase("encoder reset");
        Map<String, Object> m = new LinkedHashMap<>();
        report.put("resetAfter", m);
        m.put("countAtInit", hubCount());
        m.put("absoluteBefore", absoluteRead());
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.put("afterStopAndReset", new int[] {hubCount(), hubCountAfterAWhile()});
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.put("afterRunWithoutEncoder", new int[] {hubCount(), hubCountAfterAWhile()});
        m.put("absoluteAfter", absoluteRead());
        return "encoder reset: " + m;
    }

    private String track(File finish) {
        boolean pulseWidth = !phase.equals("quad");
        Map<String, Object> m = new LinkedHashMap<>();
        List<double[]> rows = new ArrayList<>();
        List<Map<String, Object>> rests = new ArrayList<>();
        List<double[]> hubEvents = new ArrayList<>();
        Map<String, Object> zero = null;
        if (phase.equals("agreement")) {
            zero = absoluteRead();
            if (!Boolean.TRUE.equals(zero.get("valid"))) {
                throw new IllegalStateException("no valid absolute read on OctoQuad channel " + CHANNEL + " for the zero: "
                        + zero);
            }
            m.put("zero", zero);
        }
        m.put("rowColumns", new String[] {"ms", "count", "channel" + CHANNEL, "crcOk", "bulkFake"});
        m.put("rows", rows);
        m.put("rests", rests);
        m.put("hubEventColumns", new String[] {"ms", "notResponding", "count"});
        m.put("hubEvents", hubEvents);
        report.put("track", m);
        report.save();
        if (!report.holdUntilStart(this)) return phase + ": stopped in INIT";
        report.phase(phase);
        Samples loopMs = new Samples();
        Integer last = null;
        long lastChange = System.nanoTime();
        boolean restTaken = false;
        boolean notResponding = hub.isNotResponding();
        long nextFinishPoll = 0;
        long crcFailures = 0;
        long fakeReads = 0;
        Integer firstCount = null;
        int count = 0;
        while (opModeIsActive()) {
            long now = System.nanoTime();
            loopMs.tick(now);
            hub.clearBulkCache();
            LynxModule.BulkData bulk = hub.getBulkData();
            count = bulk.getMotorCurrentPosition(PORT);
            if (bulk.isFake()) fakeReads++;
            else if (firstCount == null) firstCount = count;
            octoQuad.readAllEncoderData(block);
            int value = block.positions[CHANNEL];
            if (!block.crcOk) crcFailures++;
            double ms = (now - t0) / 1e6;
            if (rows.size() < MAX_ROWS) {
                rows.add(new double[] {ms, count, value, block.crcOk ? 1 : 0, bulk.isFake() ? 1 : 0});
            }
            boolean valid = pulseWidth ? !bulk.isFake() : block.crcOk;
            int moving = pulseWidth ? count : value;
            if (valid && (last == null || moving != last)) {
                last = moving;
                lastChange = now;
                restTaken = false;
            } else if (valid && !restTaken && now - lastChange >= REST_NS && rests.size() < MAX_RESTS) {
                rests.add(rest(ms, pulseWidth, zero));
                restTaken = true;
                report.save();
            }
            boolean nr = hub.isNotResponding();
            if (nr != notResponding) {
                hubEvents.add(new double[] {ms, nr ? 1 : 0, count});
                notResponding = nr;
            }
            if (now >= nextFinishPoll) {
                nextFinishPoll = now + FINISH_POLL_NS;
                if (finish.exists()) break;
            }
            report.status("%s: count %d, channel %d reads %d, %d rests", phase, count, CHANNEL, value, rests.size());
            report.sendIfDue();
        }
        m.put("loopMs", loopMs.summary());
        m.put("crcFailures", crcFailures);
        m.put("fakeReads", fakeReads);
        m.put("firstCount", firstCount);
        m.put("lastCount", count);
        m.put("rowsDropped", Math.max(0, loopMs.count() + 1 - rows.size()));
        m.put("finishedByRunner", finish.exists());
        return String.format("%s: %d rests, %d CRC failures, %d failed bulk reads", phase, rests.size(), crcFailures,
                fakeReads);
    }

    private Map<String, Object> rest(double ms, boolean pulseWidth, Map<String, Object> zero) {
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("ms", ms);
        r.put("count", hubCount());
        int[] values = new int[REST_READS];
        int crcFailures = 0;
        for (int i = 0; i < REST_READS; i++) {
            octoQuad.readAllEncoderData(block);
            values[i] = block.positions[CHANNEL];
            if (!block.crcOk) crcFailures++;
        }
        r.put("channel" + CHANNEL, values);
        r.put("crcFailures", crcFailures);
        int[] sorted = Arrays.copyOf(values, values.length);
        Arrays.sort(sorted);
        int median = sorted[REST_READS / 2];
        r.put("median", median);
        if (!pulseWidth) return r;
        double deg = crcFailures == 0 ? toDeg(median) : Double.NaN;
        r.put("absoluteDeg", deg);
        if (zero != null) {
            double zeroDeg = (Double) zero.get("absoluteDeg");
            int zeroCount = (Integer) zero.get("count");
            int count = (Integer) r.get("count");
            r.put("errorDeg", TurretZeroMath.agreementErrorDeg(zeroDeg, zeroCount, deg, count, sign));
            r.put("errorFlippedSignDeg", TurretZeroMath.agreementErrorDeg(zeroDeg, zeroCount, deg, count, -sign));
            r.put("error180Deg", TurretZeroMath.agreementErrorDeg(zeroDeg + 180, zeroCount, deg, count, sign));
        }
        return r;
    }
}
