package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: OctoQuad", group = "Test")
public class OctoQuadBench extends EnhancedOpMode {
    static final String KEY_STATUS = "Bench octoquad status";
    static final String KEY_VALID = "Bench octoquad valid";
    private static final Pose SEED = new Pose(56, 8, Math.PI / 2);
    private static final double SEED_RUN_S = 3;
    private static final int MAX_ROWS = 6000;
    private static final int READ_CALLS = 200;
    private static final int STATUS_CALLS = 50;
    private static final int BANK_READ_CALLS = 300;
    private static final int INIT_READ_CALLS = 50;
    private static final long LEAVE_RUNNING_NS = 1_000_000_000L;
    private static final long REACH_RUNNING_NS = 5_000_000_000L;
    static final OctoQuad.ChannelBankConfig MIXED_BANKS = OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH;
    static final int TURRET_CHANNEL = 4;

    private final StopHook saver = new StopHook(this::finish);
    private BenchReport report;
    private String mode;
    private OctoQuad octoQuad;
    private GuardedOctoQuadLocalizer guard;
    private OctoQuad.LocalizerDataBlock localizerData;
    private boolean present;
    private long createStartNs;
    private long createdNs;
    private long startNs;
    private long lastLoopNs;
    private long loops;
    private boolean running;
    private boolean finished;
    private boolean observed;
    private boolean lastValid;
    private boolean seenValid;
    private OctoQuad.LocalizerStatus lastStatus;
    private String lastLabel;
    private long crcFailures;
    private long invalidLoops;
    private final List<Object[]> statusChanges = new ArrayList<>();
    private final Map<String, Integer> statusLoops = new LinkedHashMap<>();
    private final Samples loopValidMs = new Samples();
    private final Samples loopInvalidMs = new Samples();
    private final Map<String, Object> seed = new LinkedHashMap<>();
    private double maxOffsetIn;
    private double maxTurnDeg;
    private final Map<String, Object> dropout = new LinkedHashMap<>();
    private final List<double[]> rows = new ArrayList<>();
    private long badLoops;
    private long silentBadLoops;
    private Pose lastValidPose;
    private Pose firstBadPose;

    @Override
    protected Robot createRobot() {
        mode = BenchIO.param(BenchIO.params(), "mode", "seed");
        if (!mode.equals("seed") && !mode.equals("dropout")) {
            throw new IllegalArgumentException("bench param mode must be seed or dropout, was " + mode);
        }
        report = new BenchReport("OctoQuad", "octoquad_" + mode);
        octoQuad = BenchIO.require(hardwareMap, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
        int chipId = octoQuad.getChipId() & 0xFF;
        present = chipId == (OctoQuad.OCTOQUAD_CHIP_ID & 0xFF);
        report.put("chipId", chipId);
        report.put("present", present);
        if (!present) {
            report.done("no OctoQuad answered on " + OctoQuadLocalizerTest.name + "'s I2C port");
            return new BenchRobot(this);
        }
        report.put("firmware", octoQuad.getFirmwareVersionString());
        report.phase("create");
        report.put("readCostMs", readCost());
        createStartNs = System.nanoTime();
        OctoQuadSetup.OctoQuadRobot octoQuadRobot = new OctoQuadSetup.OctoQuadRobot(this);
        createdNs = System.nanoTime();
        guard = OctoQuadSetup.guard(octoQuadRobot.follower);
        localizerData = guard.lastRead();
        return octoQuadRobot;
    }

    private Map<String, Object> readCost() {
        OctoQuad.LocalizerDataBlock block = new OctoQuad.LocalizerDataBlock();
        OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
        Samples localizerOnly = new Samples();
        Samples withEncoders = new Samples();
        Samples status = new Samples();
        for (int i = 0; i < READ_CALLS; i++) {
            long t0 = System.nanoTime();
            octoQuad.readLocalizerData(block);
            localizerOnly.addNanos(System.nanoTime() - t0);
        }
        for (int i = 0; i < READ_CALLS; i++) {
            long t0 = System.nanoTime();
            octoQuad.readLocalizerDataAndAllEncoderData(block, encoders);
            withEncoders.addNanos(System.nanoTime() - t0);
        }
        for (int i = 0; i < STATUS_CALLS; i++) {
            long t0 = System.nanoTime();
            octoQuad.getLocalizerStatus();
            status.addNanos(System.nanoTime() - t0);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("readLocalizerData", localizerOnly.summary());
        m.put("readLocalizerDataAndAllEncoderData", withEncoders.summary());
        m.put("getLocalizerStatus", status.summary());
        return m;
    }

    @Override
    protected void initialize() {
        if (!present) return;
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("statusAfterCreate", octoQuad.getLocalizerStatus().name());
        info.put("createMs", (createdNs - createStartNs) / 1e6);
        info.put("headingAxis", octoQuad.getLocalizerHeadingAxisChoice().name());
        info.put("seed", SEED.toString());
        info.put("params", OctoQuadSetup.params());
        report.put("info", info);
        report.put("statusChanges", statusChanges);
        robot.follower.setPose(SEED);
        report.phase(mode);
        report.save();
    }

    @Override
    protected void onLoopStart() {
        long now = System.nanoTime();
        if (present && !finished && lastLoopNs != 0) {
            (lastValid ? loopValidMs : loopInvalidMs).add((now - lastLoopNs) / 1e6);
        }
        lastLoopNs = now;
    }

    @Override
    protected void initializeLoop() {
        observe();
    }

    @Override
    protected void onStart() {
        startNs = System.nanoTime();
        running = true;
        lastLoopNs = 0;
        if (!present) return;
        Pose pose = robot.follower.pose();
        seed.put("offsetAtStartIn", pose.distance(SEED));
        seed.put("turnAtStartDeg", PinpointBench.turnDeg(pose, SEED));
    }

    @Override
    protected void gameLoop() {
        observe();
        if (present && mode.equals("seed") && !finished && (System.nanoTime() - startNs) / 1e9 >= SEED_RUN_S) {
            finish();
            String bank = bankPhase();
            report.done(String.format("start pose held within %.3f in and %.3f deg through START + %.0f s; %s",
                    maxOffsetIn, maxTurnDeg, SEED_RUN_S, bank));
        }
    }

    private void observe() {
        if (!present || finished) return;
        loops++;
        OctoQuad.LocalizerStatus status = localizerData.localizerStatus;
        boolean crcOk = localizerData.crcOk;
        boolean valid = localizerData.isDataValid();
        double msSinceCreate = (System.nanoTime() - createdNs) / 1e6;
        String label = status == null ? "polled " + guard.status() : crcOk ? status.name() : status + " CRC";
        if (!label.equals(lastLabel)) {
            statusChanges.add(new Object[] {msSinceCreate, label, running ? "running" : "init"});
            lastLabel = label;
        }
        Integer count = statusLoops.get(label);
        statusLoops.put(label, count == null ? 1 : count + 1);
        if (valid) seenValid = true;
        if (seenValid && !crcOk) crcFailures++;
        if (seenValid && !valid) invalidLoops++;
        lastStatus = status;
        lastValid = valid;
        observed = true;
        Pose pose = robot.follower.pose();
        if (mode.equals("seed")) {
            double offset = pose.distance(SEED);
            double turn = PinpointBench.turnDeg(pose, SEED);
            if (!seed.containsKey("firstOffsetIn")) {
                seed.put("firstOffsetIn", offset);
                seed.put("firstTurnDeg", turn);
            }
            maxOffsetIn = Math.max(maxOffsetIn, offset);
            maxTurnDeg = Math.max(maxTurnDeg, turn);
        } else if (running) {
            if (rows.size() < MAX_ROWS) {
                rows.add(new double[] {(System.nanoTime() - startNs) / 1e6, status.ordinal(), crcOk ? 1 : 0,
                        pose.x(), pose.y(), Math.toDegrees(pose.heading())});
            }
            if (valid) {
                if (badLoops > 0 && !dropout.containsKey("validAgainMs")) {
                    dropout.put("validAgainMs", (System.nanoTime() - startNs) / 1e6);
                    dropout.put("poseWhenValidAgain", pose.toString());
                }
                lastValidPose = pose;
            } else {
                badLoops++;
                if (!octoQuadFaultActive()) silentBadLoops++;
                if (firstBadPose == null) {
                    firstBadPose = pose;
                    dropout.put("firstInvalidMs", (System.nanoTime() - startNs) / 1e6);
                    dropout.put("firstInvalidStatus", String.valueOf(status));
                    dropout.put("firstInvalidCrcOk", crcOk);
                    dropout.put("firstInvalidPose", pose.toString());
                    dropout.put("lastValidPose", String.valueOf(lastValidPose));
                    dropout.put("poseDuringDropout", PinpointBench.describeDropout(pose, lastValidPose));
                }
            }
        }
        report.status("%s", label);
    }

    private boolean octoQuadFaultActive() {
        for (Faults.Fault f : activeFaults()) {
            if ((f.key + " " + f.message).toLowerCase().contains("octoquad")) return true;
        }
        return false;
    }

    void finish() {
        if (finished || report == null) return;
        finished = true;
        if (!present) {
            report.save();
            return;
        }
        report.put("crcFailures", crcFailures);
        report.put("invalidLoops", invalidLoops);
        report.put("statusLoops", statusLoops);
        report.put("loopPeriodValidMs", loopValidMs.summary());
        report.put("loopPeriodInvalidMs", loopInvalidMs.summary());
        report.put("finalPose", robot.follower.pose().toString());
        report.put("finalStatus", octoQuad.getLocalizerStatus().name());
        if (mode.equals("seed")) {
            seed.put("maxOffsetIn", maxOffsetIn);
            seed.put("maxTurnDeg", maxTurnDeg);
            seed.put("loops", loops);
            seed.put("ran", running);
            report.put("seed", seed);
        } else {
            dropout.put("badLoops", badLoops);
            dropout.put("silentBadLoops", silentBadLoops);
            dropout.put("loops", loops);
            dropout.put("statusOrdinals", OctoQuad.LocalizerStatus.values());
            dropout.put("rowColumns", new String[] {"msSinceStart", "status", "crcOk", "x", "y", "headingDeg"});
            dropout.put("rows", rows);
            report.put("dropout", dropout);
        }
        report.save();
    }

    /** Channel 4 carries an odometry pod on this bench, so only the timings and bank read-backs mean anything. */
    private String bankPhase() {
        report.phase("bank");
        Map<String, Object> bank = new LinkedHashMap<>();
        Map<String, Object> setup = OctoQuadSetup.params();
        bank.put("podPorts", new Object[] {setup.get("portX"), setup.get("portY")});
        report.put("bank", bank);
        OctoQuad.ChannelBankConfig found = octoQuad.getChannelBankConfig();
        bank.put("asFound", found.name());
        report.save();
        try {
            bankSteps(bank);
        } finally {
            octoQuad.setChannelBankConfig(found);
            bank.put("afterRestore", octoQuad.getChannelBankConfig().name());
            bank.put("stopRequested", stopping());
            report.save();
        }
        return "bank " + found + " -> " + bank.get("afterSet") + " -> " + bank.get("afterRestore");
    }

    private void bankSteps(Map<String, Object> bank) {
        bank.put("readLocalizerDataAsFound", timeLocalizerReads(false));
        bank.put("readLocalizerDataAndAllEncoderDataAsFound", timeLocalizerReads(true));
        if (stopping()) return;
        octoQuad.setChannelBankConfig(MIXED_BANKS);
        bank.put("afterSet", octoQuad.getChannelBankConfig().name());
        report.save();
        bank.put("readLocalizerDataMixed", timeLocalizerReads(false));
        bank.put("initRead", timeInitReads());
        report.save();
        if (stopping()) return;
        GuardedOctoQuadLocalizer rebuilt = new GuardedOctoQuadLocalizer(hardwareMap, OctoQuadSetup.localizerConfig());
        bank.put("afterLocalizerSetup", awaitLocalizerReset());
        report.save();
        if (stopping()) return;
        rebuilt.reset();
        bank.put("afterLocalizerReset", awaitLocalizerReset());
        report.save();
    }

    // STOP waits for this loop() to return and, 900 ms in, makes every Lynx call throw, the bank restore's included.
    private static boolean stopping() {
        return OpModeManagerImpl.shouldPreventDangerousHardwareAccess();
    }

    private Map<String, Object> timeLocalizerReads(boolean withEncoders) {
        OctoQuad.LocalizerDataBlock block = new OctoQuad.LocalizerDataBlock();
        OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
        Samples ms = new Samples();
        int invalid = 0;
        for (int i = 0; i < BANK_READ_CALLS && !stopping(); i++) {
            long t0 = System.nanoTime();
            if (withEncoders) octoQuad.readLocalizerDataAndAllEncoderData(block, encoders);
            else octoQuad.readLocalizerData(block);
            ms.addNanos(System.nanoTime() - t0);
            if (!block.isDataValid()) invalid++;
            report.sendIfDue();
        }
        Map<String, Object> m = ms.summary();
        m.put("invalid", invalid);
        return m;
    }

    private Map<String, Object> timeInitReads() {
        OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
        Samples ms = new Samples();
        int crcFailures = 0;
        int[] channel = new int[INIT_READ_CALLS];
        for (int i = 0; i < INIT_READ_CALLS && !stopping(); i++) {
            long t0 = System.nanoTime();
            octoQuad.readAllEncoderData(encoders);
            ms.addNanos(System.nanoTime() - t0);
            if (!encoders.crcOk) crcFailures++;
            channel[i] = encoders.positions[TURRET_CHANNEL];
            report.sendIfDue();
        }
        Map<String, Object> m = ms.summary();
        m.put("crcFailures", crcFailures);
        m.put("channel" + TURRET_CHANNEL, channel);
        return m;
    }

    private Map<String, Object> awaitLocalizerReset() {
        Map<String, Object> m = new LinkedHashMap<>();
        long t0 = System.nanoTime();
        OctoQuad.LocalizerStatus status = octoQuad.getLocalizerStatus();
        while (status == OctoQuad.LocalizerStatus.RUNNING && System.nanoTime() - t0 < LEAVE_RUNNING_NS && !stopping()) {
            status = octoQuad.getLocalizerStatus();
        }
        m.put("leftRunningMs", status == OctoQuad.LocalizerStatus.RUNNING ? null : (System.nanoTime() - t0) / 1e6);
        while (status != OctoQuad.LocalizerStatus.RUNNING && System.nanoTime() - t0 < REACH_RUNNING_NS && !stopping()) {
            status = octoQuad.getLocalizerStatus();
            report.sendIfDue();
        }
        m.put("runningMs", status == OctoQuad.LocalizerStatus.RUNNING ? (System.nanoTime() - t0) / 1e6 : null);
        m.put("status", status.name());
        m.put("bank", octoQuad.getChannelBankConfig().name());
        return m;
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(BenchReport.KEY_LOOP, loops);
        if (observed) {
            telemetry.addData(KEY_STATUS, String.valueOf(lastStatus));
            telemetry.addData(KEY_VALID, String.valueOf(lastValid));
        }
        if (running && present && mode.equals("dropout") && !finished) {
            telemetry.addData(BenchReport.KEY_DONE, "dropout: recording until STOP");
        }
    }
}
