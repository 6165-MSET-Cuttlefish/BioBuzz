package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: OctoQuad", group = "Test")
public class OctoQuadBench extends EnhancedOpMode {
    static final String KEY_STATUS = "Bench octoquad status";
    static final String KEY_VALID = "Bench octoquad valid";
    static final String KEY_FAULT = "Bench octoquad fault";
    static final String KEY_REPLUG = "Bench octoquad replug";
    static final String ROLLED_FILE = "octoquad_rolled.json";
    private static final List<String> MODES = Arrays.asList("seed", "dropout", "earlystart", "markers", "unguarded",
            "latejoin");
    private static final Pose SEED = new Pose(56, 8, Math.PI / 2);
    private static final double SEED_RUN_S = 3;
    private static final double EARLY_RUN_S = 5;
    private static final int MAX_ROWS = 6000;
    private static final int READ_CALLS = 200;
    private static final int STATUS_CALLS = 50;
    private static final int BANK_READ_CALLS = 300;
    private static final int INIT_READ_CALLS = 50;
    private static final long LEAVE_RUNNING_NS = 1_000_000_000L;
    private static final long REACH_RUNNING_NS = 5_000_000_000L;
    private static final long ANSWER_WITHIN_NS = 5_000_000_000L;
    private static final long ROLLED_POLL_NS = 200_000_000L;
    private static final double REPLUG_OUT_MS = 500;
    static final OctoQuad.ChannelBankConfig MIXED_BANKS = OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH;
    static final int TURRET_CHANNEL = 4;
    static final int MARKER_CHANNEL = 7;
    static final int MARKER_INTERVAL_MS = 17;
    private static final String NO_REPLY = "no reply";
    private static final String NOT_READ = "not read yet";

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
    private long lastInitLoopNs;
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

    private int[] podPorts;
    private Settings found;
    private Settings markersSet;
    private final Map<String, Object> early = new LinkedHashMap<>();
    private final Map<String, Object> lateJoin = new LinkedHashMap<>();
    private String faultState = "none";
    private final Map<String, Object> unguarded = new LinkedHashMap<>();
    private boolean replugRead;
    private double noReplySinceMs = -1;
    private boolean markersRestored;
    private final Map<String, Object> markers = new LinkedHashMap<>();
    private final List<String> readFailures = new ArrayList<>();
    private String markerStep = "before";
    private long nextRolledPollNs;

    private static final class Settings {
        int chipId;
        int uptimeMs;
        String bank;
        int[] intervalsMs = new int[OctoQuadMath.CHANNELS];
        int[] directions;

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("chipId", chipId);
            m.put("uptimeMs", uptimeMs);
            m.put("bank", bank);
            m.put("intervalsMs", intervalsMs);
            m.put("directionsReversed", directions);
            return m;
        }
    }

    @Override
    protected Robot createRobot() {
        Map<String, Object> params = BenchIO.params();
        mode = BenchIO.param(params, "mode", "seed");
        if (!MODES.contains(mode)) {
            throw new IllegalArgumentException("bench param mode must be one of " + MODES + ", was " + mode);
        }
        if (mode.equals("unguarded") == OctoQuadSetup.guarded(params)) {
            throw new IllegalArgumentException("bench param guard must be 0 in mode unguarded and 1 in every other mode");
        }
        report = new BenchReport("OctoQuad", "octoquad_" + mode);
        File rolled = new File(BenchIO.dir(), ROLLED_FILE);
        if (rolled.exists() && !rolled.delete()) throw new IllegalStateException("can't delete " + rolled);
        octoQuad = BenchIO.require(hardwareMap, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
        int chipId = octoQuad.getChipId() & 0xFF;
        present = chipId == (OctoQuad.OCTOQUAD_CHIP_ID & 0xFF);
        report.put("chipId", chipId);
        report.put("present", present);
        if (mode.equals("latejoin")) {
            lateJoin.put("answeredAtInit", present);
            present = true;
        } else if (!present) {
            report.done("no OctoQuad answered on " + OctoQuadLocalizerTest.name + "'s I2C port");
            return new BenchRobot(this);
        } else {
            report.put("firmware", octoQuad.getFirmwareVersionString());
        }
        if (mode.equals("seed") || mode.equals("dropout")) {
            report.phase("create");
            report.put("readCostMs", readCost());
        }
        OctoQuadConfig config = OctoQuadSetup.localizerConfig(params);
        podPorts = new int[] {config.xPodPort.get(), config.yPodPort.get()};
        createStartNs = System.nanoTime();
        Robot built;
        if (mode.equals("markers")) {
            built = new PinpointBench.NoOdometryRobot(this);
        } else {
            built = new OctoQuadSetup.OctoQuadRobot(this);
        }
        createdNs = System.nanoTime();
        if (!mode.equals("markers")) {
            localizerData = OctoQuadSetup.lastRead(built.follower);
            if (!mode.equals("unguarded")) guard = OctoQuadSetup.guard(built.follower);
        }
        return built;
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
        if (mode.equals("markers")) {
            new GuardedOctoQuadLocalizer(hardwareMap, OctoQuadSetup.localizerConfig());
            markers.put("podPorts", podPorts);
            setMarkers(markers);
            report.put("markers", markers);
        } else {
            if (mode.equals("unguarded")) {
                setMarkers(unguarded);
                report.put("unguarded", unguarded);
            }
            robot.follower.setPose(SEED);
        }
        report.phase(mode);
        report.save();
    }

    // Neither marker is saved to flash, so a reboot shows as both back at their defaults.
    private void setMarkers(Map<String, Object> into) {
        found = readSettings();
        into.put("found", found.toMap());
        octoQuad.setSingleVelocitySampleInterval(MARKER_CHANNEL, MARKER_INTERVAL_MS);
        octoQuad.setChannelBankConfig(MIXED_BANKS);
        markersSet = readSettings();
        into.put("markersSet", markersSet.toMap());
        into.put("markersTook", markersSet.bank.equals(MIXED_BANKS.name())
                && OctoQuadMath.kept(markersSet.intervalsMs, MARKER_CHANNEL, MARKER_INTERVAL_MS));
    }

    private Settings readSettings() {
        Settings s = new Settings();
        s.chipId = octoQuad.getChipId() & 0xFF;
        s.uptimeMs = octoQuad.getUptimeMs();
        s.bank = octoQuad.getChannelBankConfig().name();
        boolean[] reversed = new boolean[OctoQuadMath.CHANNELS];
        for (int ch = 0; ch < OctoQuadMath.CHANNELS; ch++) {
            s.intervalsMs[ch] = octoQuad.getSingleVelocitySampleInterval(ch);
            reversed[ch] = octoQuad.getSingleEncoderDirection(ch) == OctoQuad.EncoderDirection.REVERSE;
        }
        s.directions = OctoQuadMath.directionBits(reversed);
        return s;
    }

    private void restoreMarkers(Map<String, Object> into) {
        if (found == null || markersRestored) return;
        markersRestored = true;
        octoQuad.setChannelBankConfig(OctoQuad.ChannelBankConfig.valueOf(found.bank));
        octoQuad.setSingleVelocitySampleInterval(MARKER_CHANNEL, Math.max(1, found.intervalsMs[MARKER_CHANNEL]));
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("bank", octoQuad.getChannelBankConfig().name());
        m.put("interval" + MARKER_CHANNEL + "Ms", octoQuad.getSingleVelocitySampleInterval(MARKER_CHANNEL));
        into.put("restored", m);
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
        lastInitLoopNs = System.nanoTime();
    }

    @Override
    protected void onStart() {
        long now = System.nanoTime();
        startNs = now;
        running = true;
        lastLoopNs = 0;
        if (!present) return;
        if (lastInitLoopNs != 0) {
            early.put("startWaitMs", (now - lastInitLoopNs) / 1e6);
            early.put("validAtLastInitLoop", lastValid);
            early.put("msSinceCreateAtStart", (now - createdNs) / 1e6);
        }
        if (mode.equals("markers")) return;
        Pose pose = robot.follower.pose();
        seed.put("offsetAtStartIn", pose.distance(SEED));
        seed.put("turnAtStartDeg", PinpointBench.turnDeg(pose, SEED));
    }

    @Override
    protected void gameLoop() {
        observe();
        if (!present || finished) return;
        double seconds = (System.nanoTime() - startNs) / 1e9;
        if (mode.equals("seed") && seconds >= SEED_RUN_S) {
            finish();
            String bank = bankPhase();
            report.done(String.format("start pose held within %.3f in and %.3f deg through START + %.0f s; %s",
                    maxOffsetIn, maxTurnDeg, SEED_RUN_S, bank));
        } else if (mode.equals("latejoin") && seconds >= SEED_RUN_S) {
            finish();
            report.done(String.format("joined: fault at %s ms, RUNNING %s ms after it answered; start pose held within "
                    + "%.3f in and %.3f deg", lateJoin.get("faultMs"), lateJoin.get("runningAfterAnswerMs"), maxOffsetIn,
                    maxTurnDeg));
        } else if (mode.equals("earlystart") && seconds >= EARLY_RUN_S) {
            early.put("ranSeconds", seconds);
            finish();
            report.done(String.format("ran %.0f s after START; START waited %s ms for RUNNING", EARLY_RUN_S,
                    early.get("startWaitMs")));
        } else if (mode.equals("markers")) {
            markerStep();
        }
    }

    private void observe() {
        if (!present || finished || mode.equals("markers")) return;
        loops++;
        OctoQuad.LocalizerStatus status = localizerData.localizerStatus;
        boolean crcOk = localizerData.crcOk;
        boolean valid = localizerData.isDataValid();
        double msSinceCreate = (System.nanoTime() - createdNs) / 1e6;
        String polled = guard == null ? NOT_READ : guard.status();
        String label = status == null ? "polled " + polled : crcOk ? status.name() : status + " CRC";
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
        if (mode.equals("latejoin")) joining(msSinceCreate, polled, valid);
        if (mode.equals("seed") || mode.equals("latejoin") || mode.equals("earlystart")) {
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
            if (mode.equals("unguarded")) afterReplug(crcOk, valid, pose);
        }
        report.status("%s", label);
    }

    private void joining(double msSinceCreate, String polled, boolean valid) {
        boolean fault = octoQuadFaultActive();
        if (fault && !lateJoin.containsKey("faultMs")) {
            lateJoin.put("faultMs", msSinceCreate);
            lateJoin.put("faultMessage", octoQuadFaultMessage());
            faultState = "up";
        }
        boolean answering = !polled.equals(NO_REPLY) && !polled.equals(NOT_READ);
        if (answering && lateJoin.containsKey("faultMs") && !lateJoin.containsKey("answerMs")) {
            lateJoin.put("answerMs", msSinceCreate);
            lateJoin.put("answerStatus", polled);
        }
        if (!fault && "up".equals(faultState)) {
            lateJoin.put("clearMs", msSinceCreate);
            faultState = "cleared";
        }
        if (valid && !lateJoin.containsKey("runningMs")) {
            lateJoin.put("runningMs", msSinceCreate);
            lateJoin.put("runningPhase", running ? "running" : "init");
            int[] read = new int[podPorts.length];
            int[] expected = new int[podPorts.length];
            OctoQuad.EncoderDirection[] want = {OctoQuadLocalizerTest.directionX, OctoQuadLocalizerTest.directionY};
            for (int i = 0; i < podPorts.length; i++) {
                read[i] = octoQuad.getSingleEncoderDirection(podPorts[i]) == OctoQuad.EncoderDirection.REVERSE ? 1 : 0;
                expected[i] = want[i] == OctoQuad.EncoderDirection.REVERSE ? 1 : 0;
            }
            lateJoin.put("podPorts", podPorts);
            lateJoin.put("directionsReversed", read);
            lateJoin.put("directionsExpected", expected);
            lateJoin.put("directionsMatch", Arrays.equals(read, expected));
        }
        Object answer = lateJoin.get("answerMs");
        if (answer != null) {
            double a = (Double) answer;
            if (lateJoin.containsKey("clearMs")) lateJoin.put("clearAfterAnswerMs", (Double) lateJoin.get("clearMs") - a);
            if (lateJoin.containsKey("runningMs")) {
                lateJoin.put("runningAfterAnswerMs", (Double) lateJoin.get("runningMs") - a);
            }
        }
    }

    private void afterReplug(boolean crcOk, boolean valid, Pose pose) {
        double ms = (System.nanoTime() - startNs) / 1e6;
        if (!crcOk) {
            if (noReplySinceMs < 0) noReplySinceMs = ms;
            return;
        }
        double outMs = noReplySinceMs < 0 ? 0 : ms - noReplySinceMs;
        noReplySinceMs = -1;
        if (!replugRead && outMs >= REPLUG_OUT_MS && !stopping()) {
            replugRead = true;
            unguarded.put("answeredAgainMs", ms);
            unguarded.put("noReplyForMs", outMs);
            Settings after = readSettings();
            unguarded.put("afterReplug", after.toMap());
            unguarded.put("rebooted", after.uptimeMs < markersSet.uptimeMs);
            unguarded.put("bankKept", after.bank.equals(markersSet.bank));
            unguarded.put("markerIntervalKept", OctoQuadMath.kept(after.intervalsMs, MARKER_CHANNEL, MARKER_INTERVAL_MS));
            unguarded.put("intervalChanges", OctoQuadMath.diff(markersSet.intervalsMs, after.intervalsMs));
            unguarded.put("directionChanges", OctoQuadMath.diff(markersSet.directions, after.directions));
            restoreMarkers(unguarded);
            report.save();
        }
        if (replugRead && valid && !unguarded.containsKey("runningAgainMs")) {
            unguarded.put("runningAgainMs", ms);
            unguarded.put("runningAfterAnswerMs", ms - (Double) unguarded.get("answeredAgainMs"));
            unguarded.put("poseWhenRunningAgain", pose.toString());
            report.save();
        }
    }

    private void markerStep() {
        if (markerStep.equals("before")) {
            readPositions("positionsBefore");
            readPose("poseBefore");
            markerStep = "roll";
            report.phase("roll");
            report.save();
            return;
        }
        if (markerStep.equals("roll")) {
            long now = System.nanoTime();
            if (now < nextRolledPollNs) return;
            nextRolledPollNs = now + ROLLED_POLL_NS;
            if (!new File(BenchIO.dir(), ROLLED_FILE).exists()) return;
            markerStep = "resets";
            report.phase("resets");
            int[] before = (int[]) markers.get("positionsBefore");
            int[] rolled = readPositions("positionsAfterRoll");
            readPose("poseAfterRoll");
            if (before != null && rolled != null) {
                markers.put("rollCounts", (long) rolled[podPorts[0]] - before[podPorts[0]]);
            }
            if (stopping()) return;
            octoQuad.resetLocalizerAndCalibrateIMU();
            markers.put("afterLocalizerReset", awaitLocalizerReset());
            int[] reset = readPositions("positionsAfterLocalizerReset");
            readPose("poseAfterLocalizerReset");
            if (reset != null) markers.put("localizerResetZeroedPods", OctoQuadMath.zeroed(reset, podPorts, 0));
            report.save();
            if (stopping()) return;
            octoQuad.resetEverything();
            long t0 = System.nanoTime();
            boolean answered = false;
            while (!stopping() && System.nanoTime() - t0 < ANSWER_WITHIN_NS) {
                if ((octoQuad.getChipId() & 0xFF) == (OctoQuad.OCTOQUAD_CHIP_ID & 0xFF)) {
                    answered = true;
                    break;
                }
                report.sendIfDue();
            }
            markers.put("resetEverythingAnsweredMs", answered ? (System.nanoTime() - t0) / 1e6 : null);
            if (!answered) readFailures.add("chip id after resetEverything");
            if (answered) {
                Settings after = readSettings();
                markers.put("afterResetEverything", after.toMap());
                markers.put("markerIntervalKeptThroughResetEverything",
                        OctoQuadMath.kept(after.intervalsMs, MARKER_CHANNEL, MARKER_INTERVAL_MS));
                markers.put("bankKeptThroughResetEverything", after.bank.equals(markersSet.bank));
                markers.put("intervalChanges", OctoQuadMath.diff(markersSet.intervalsMs, after.intervalsMs));
                markers.put("directionChanges", OctoQuadMath.diff(markersSet.directions, after.directions));
                markers.put("rebootedByResetEverything", after.uptimeMs < markersSet.uptimeMs);
                int[] everything = readPositions("positionsAfterResetEverything");
                if (everything != null) {
                    markers.put("resetEverythingZeroedPods", OctoQuadMath.zeroed(everything, podPorts, 0));
                }
                markers.put("statusAfterResetEverything", octoQuad.getLocalizerStatus().name());
                restoreMarkers(markers);
            }
            markerStep = "done";
            finish();
            report.done("markers: " + (readFailures.isEmpty() ? "every read answered" : "no answer for " + readFailures));
        }
    }

    private int[] readPositions(String key) {
        OctoQuad.EncoderDataBlock block = new OctoQuad.EncoderDataBlock();
        long t0 = System.nanoTime();
        while (!stopping()) {
            octoQuad.readAllEncoderData(block);
            if (block.crcOk) {
                int[] positions = block.positions.clone();
                markers.put(key, positions);
                markers.put(key + "ReadMs", (System.nanoTime() - t0) / 1e6);
                return positions;
            }
            if (System.nanoTime() - t0 > ANSWER_WITHIN_NS) break;
            report.sendIfDue();
        }
        markers.put(key, null);
        if (!stopping()) readFailures.add(key);
        return null;
    }

    private void readPose(String key) {
        if (stopping()) return;
        OctoQuad.LocalizerDataBlock block = new OctoQuad.LocalizerDataBlock();
        octoQuad.readLocalizerData(block);
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("status", String.valueOf(block.localizerStatus));
        m.put("crcOk", block.crcOk);
        m.put("xMm", block.posX_mm);
        m.put("yMm", block.posY_mm);
        m.put("headingDeg", Math.toDegrees(block.heading_rad));
        markers.put(key, m);
    }

    private boolean octoQuadFaultActive() {
        return octoQuadFaultMessage() != null;
    }

    private String octoQuadFaultMessage() {
        for (Faults.Fault f : activeFaults()) {
            if ((f.key + " " + f.message).toLowerCase().contains("octoquad")) return f.key + ": " + f.message;
        }
        return null;
    }

    void finish() {
        if (finished || report == null) return;
        finished = true;
        if (!present) {
            report.save();
            return;
        }
        report.put("early", early);
        if (mode.equals("markers")) {
            markers.put("step", markerStep);
            markers.put("readFailures", readFailures);
            markers.put("stopRequested", stopping());
            restoreMarkers(markers);
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
        if (mode.equals("seed") || mode.equals("latejoin") || mode.equals("earlystart")) {
            seed.put("maxOffsetIn", maxOffsetIn);
            seed.put("maxTurnDeg", maxTurnDeg);
            seed.put("loops", loops);
            seed.put("ran", running);
            report.put("seed", seed);
            if (mode.equals("latejoin")) report.put("latejoin", lateJoin);
        } else {
            dropout.put("badLoops", badLoops);
            dropout.put("silentBadLoops", silentBadLoops);
            dropout.put("loops", loops);
            dropout.put("finishedMs", running ? (System.nanoTime() - startNs) / 1e6 : null);
            dropout.put("statusOrdinals", OctoQuad.LocalizerStatus.values());
            dropout.put("rowColumns", new String[] {"msSinceStart", "status", "crcOk", "x", "y", "headingDeg"});
            dropout.put("rows", rows);
            report.put("dropout", dropout);
            if (mode.equals("unguarded")) {
                unguarded.put("faultsAtEnd", String.valueOf(octoQuadFaultMessage()));
                if (localizerData.crcOk) restoreMarkers(unguarded);
            }
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
        if (mode.equals("latejoin")) telemetry.addData(KEY_FAULT, faultState);
        if (mode.equals("unguarded")) telemetry.addData(KEY_REPLUG, replugRead ? "read" : "waiting");
        if (running && present && (mode.equals("dropout") || mode.equals("unguarded")) && !finished) {
            telemetry.addData(BenchReport.KEY_DONE, "dropout: recording until STOP");
        }
    }
}
