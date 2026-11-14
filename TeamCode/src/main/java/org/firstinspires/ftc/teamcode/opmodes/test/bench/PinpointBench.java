package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import android.os.Debug;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Pinpoint", group = "Test")
public class PinpointBench extends EnhancedOpMode {
    static final String KEY_STATUS = "Bench pinpoint status";
    private static final List<String> MODES = Arrays.asList("seed", "dropout", "soak", "recal", "rawDropout");
    private static final Pose SEED = new Pose(56, 8, Math.PI / 2);
    private static final double SEED_RUN_S = 3;
    private static final int MAX_ROWS = 6000;
    private static final int SOAK_OCTOQUAD_EVERY = 10;
    private static final int SOAK_STATUS_EVERY = 100;
    private static final int SOAK_MAX_ROWS = 40000;
    private static final int SOAK_KEEP = 2000;
    private static final long SOAK_SAVE_NS = 300_000_000_000L;
    private static final double RECAL_HOLD_S = 60;
    private static final long RECAL_SETTLE_NS = 15_000_000_000L;
    private static final long RECAL_LEAVE_NS = 1_000_000_000L;
    private static final long RECAL_BACK_NS = 15_000_000_000L;
    private static final long RECAL_ROW_NS = 100_000_000L;
    private static final String GC_BLOCKING = "art.gc.blocking-gc-count";
    private static final String GC_ALL = "art.gc.gc-count";

    public static class PinpointRobot extends Robot {
        public PinpointRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BettaConstants.create(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {}
    }

    static final class SoakRobot extends Robot {
        SoakRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return new Follower(new TimedLocalizer(BettaConstants.localizer(hardwareMap)),
                    BettaConstants.drivetrain(hardwareMap), BettaConstants.foresight());
        }

        @Override
        protected void initializeGameModules() {}
    }

    static final class NoOdometryRobot extends Robot {
        NoOdometryRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {}
    }

    static final class TimedLocalizer implements Localizer {
        private final Localizer inner;
        long lastNanos;
        long lastBlockingGcDelta;
        long lastGcDelta;
        long updates;

        TimedLocalizer(Localizer inner) {
            this.inner = inner;
        }

        @Override
        public void update() {
            long blocking = gcStat(GC_BLOCKING);
            long all = gcStat(GC_ALL);
            long t0 = System.nanoTime();
            inner.update();
            lastNanos = System.nanoTime() - t0;
            lastBlockingGcDelta = gcStat(GC_BLOCKING) - blocking;
            lastGcDelta = gcStat(GC_ALL) - all;
            updates++;
        }

        @Override
        public void setPose(Pose pose) {
            inner.setPose(pose);
        }

        @Override
        public MotionState state() {
            return inner.state();
        }

        @Override
        public void reset() {
            inner.reset();
        }
    }

    // Read just before follower.update(), so gameLoop's start minus this is that update plus the pose history.
    static final class UpdateClock extends Module {
        long readNs;

        UpdateClock() {
            setTelemetryEnabled(false);
        }

        @Override protected void initStates() {}
        @Override protected void read() {
            readNs = System.nanoTime();
        }
        @Override protected void write() {}
        @Override public void stop() {}
    }

    static long gcStat(String key) {
        String v = Debug.getRuntimeStat(key);
        return v == null ? -1 : Long.parseLong(v.trim());
    }

    private final StopHook saver = new StopHook(this::finish);
    private final UpdateClock clock = new UpdateClock();
    private BenchReport report;
    private String mode;
    private GoBildaPinpointDriver driver;
    private boolean present;
    private long createStartNs;
    private long createdNs;
    private long startNs;
    private long lastLoopNs;
    private long loops;
    private boolean running;
    private boolean finished;
    private GoBildaPinpointDriver.DeviceStatus lastStatus;
    private final List<Object[]> statusChanges = new ArrayList<>();
    private final Map<String, Integer> statusLoops = new LinkedHashMap<>();
    private final Samples frequencyHz = new Samples();
    private final Samples loopReadyMs = new Samples();
    private final Samples loopNotReadyMs = new Samples();
    private final Map<String, Object> seed = new LinkedHashMap<>();
    private double maxOffsetIn;
    private double maxTurnDeg;
    private final Map<String, Object> dropout = new LinkedHashMap<>();
    private final List<double[]> rows = new ArrayList<>();
    private long badLoops;
    private long silentBadLoops;
    private Pose lastReadyPose;
    private Pose firstBadPose;

    private OctoQuad octoQuad;
    private boolean octoQuadPresent;
    private final OctoQuad.LocalizerDataBlock octoData = new OctoQuad.LocalizerDataBlock();
    private TimedLocalizer timed;
    private double soakSeconds;
    private long runLoops;
    private long timedUpdatesSeen;
    private long lastSaveNs;
    private final PinpointMath.Tail readTail = new PinpointMath.Tail();
    private final PinpointMath.Tail followerTail = new PinpointMath.Tail();
    private final PinpointMath.Tail loopTail = new PinpointMath.Tail();
    private final PinpointMath.SlowReads slowReads = new PinpointMath.SlowReads(SOAK_KEEP);
    private final PinpointMath.Streaks streaks = new PinpointMath.Streaks(SOAK_KEEP);
    private final PinpointMath.Drift pinpointDrift = new PinpointMath.Drift();
    private final PinpointMath.Drift octoQuadDrift = new PinpointMath.Drift();
    private final List<double[]> headingRows = new ArrayList<>();
    private long octoQuadReads;
    private long octoQuadInvalid;

    private final Map<String, Object> recal = new LinkedHashMap<>();
    private String recalStep = "settle";
    private long recalStepNs;
    private boolean pinpointLeftReady;
    private boolean octoQuadLeftRunning;
    private long nextRecalRowNs;

    @Override
    protected Robot createRobot() {
        mode = BenchIO.param(BenchIO.params(), "mode", "seed");
        if (!MODES.contains(mode)) {
            throw new IllegalArgumentException("bench param mode must be one of " + MODES + ", was " + mode);
        }
        report = new BenchReport("Pinpoint", resultFile(mode));
        driver = BenchIO.require(hardwareMap, GoBildaPinpointDriver.class, BettaConstants.pinpoint.name, BenchIO.RUNNER_HINT);
        int version = driver.getDeviceVersion();
        present = version != 0;
        report.put("deviceVersion", version);
        report.put("present", present);
        if (!present) {
            report.done("no Pinpoint answered on " + BettaConstants.pinpoint.name + "'s I2C port");
            return new BenchRobot(this);
        }
        if (mode.equals("soak") || mode.equals("recal")) {
            octoQuad = BenchIO.require(hardwareMap, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
            int chipId = octoQuad.getChipId() & 0xFF;
            octoQuadPresent = chipId == (OctoQuad.OCTOQUAD_CHIP_ID & 0xFF);
            report.put("octoQuadChipId", chipId);
            report.put("octoQuadPresent", octoQuadPresent);
        }
        createStartNs = System.nanoTime();
        Robot built;
        if (mode.equals("soak")) {
            SoakRobot soakRobot = new SoakRobot(this);
            timed = (TimedLocalizer) soakRobot.follower.localizer;
            built = soakRobot;
        } else if (mode.equals("recal") || mode.equals("rawDropout")) {
            built = new NoOdometryRobot(this);
        } else {
            built = new PinpointRobot(this);
        }
        createdNs = System.nanoTime();
        return built;
    }

    static String resultFile(String mode) {
        return "pinpoint_" + (mode.equals("rawDropout") ? "raw_dropout" : mode);
    }

    @Override
    protected void initialize() {
        if (!present) return;
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("deviceId", driver.getDeviceID());
        info.put("statusAfterCreate", driver.getDeviceStatus().name());
        info.put("createMs", (createdNs - createStartNs) / 1e6);
        info.put("xOffsetIn", driver.getXOffset(DistanceUnit.INCH));
        info.put("yOffsetIn", driver.getYOffset(DistanceUnit.INCH));
        info.put("yawScalar", driver.getYawScalar());
        if (mode.equals("soak")) {
            soakSeconds = BenchIO.param(report.params, "seconds", 300);
            info.put("seconds", soakSeconds);
            info.put("gcStatsReadable", gcStat(GC_BLOCKING) >= 0 && gcStat(GC_ALL) >= 0);
        }
        if (octoQuadPresent) {
            new GuardedOctoQuadLocalizer(hardwareMap, OctoQuadSetup.localizerConfig());
            info.put("octoQuadSetup", OctoQuadSetup.params());
        }
        if (mode.equals("seed") || mode.equals("dropout")) {
            info.put("seed", SEED.toString());
            robot.follower.setPose(SEED);
        }
        report.put("info", info);
        report.put("statusChanges", statusChanges);
        report.phase(mode);
        report.save();
    }

    @Override
    protected void onLoopStart() {
        long now = System.nanoTime();
        if (present && !finished && lastLoopNs != 0) {
            double ms = (now - lastLoopNs) / 1e6;
            if (mode.equals("soak")) {
                if (running) loopTail.add(ms);
            } else {
                (lastStatus == GoBildaPinpointDriver.DeviceStatus.READY ? loopReadyMs : loopNotReadyMs).add(ms);
            }
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
        lastSaveNs = startNs;
        recalStepNs = startNs;
        if (mode.equals("soak")) lastLoopNs = 0;
        if (!present) return;
        if (mode.equals("seed") || mode.equals("dropout")) {
            Pose pose = robot.follower.pose();
            seed.put("offsetAtStartIn", pose.distance(SEED));
            seed.put("turnAtStartDeg", turnDeg(pose, SEED));
        }
    }

    @Override
    protected void gameLoop() {
        observe();
        if (!present || finished) return;
        double seconds = (System.nanoTime() - startNs) / 1e9;
        if (mode.equals("seed") && seconds >= SEED_RUN_S) {
            finish();
            report.done(String.format("start pose held within %.3f in and %.3f deg through START + %.0f s",
                    maxOffsetIn, maxTurnDeg, SEED_RUN_S));
        } else if (mode.equals("soak") && seconds >= soakSeconds) {
            finish();
            report.done(String.format("soak %.0f s: %d non-READY reads after the first READY, %d reads over %.0f ms "
                            + "without a blocking GC, Pinpoint read max %.1f ms", seconds, streaks.notReady(),
                    slowReads.over40WithoutGc(), PinpointMath.STALL_MS, readTail.max()));
        } else if (mode.equals("recal") && recalStep.equals("done")) {
            finish();
            report.done("recalibrated: Pinpoint after " + recal.get("pinpointRecalMs") + " ms, OctoQuad after "
                    + recal.get("octoQuadRecalMs") + " ms, then " + RECAL_HOLD_S + " s still");
        }
    }

    private void observe() {
        if (!present || finished) return;
        if (mode.equals("soak")) {
            observeSoak();
            return;
        }
        if (mode.equals("recal")) {
            observeRecal();
            return;
        }
        loops++;
        if (mode.equals("rawDropout")) driver.update();
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        noteStatus(status);
        boolean ready = status == GoBildaPinpointDriver.DeviceStatus.READY;
        if (ready) frequencyHz.add(driver.getFrequency());
        Pose pose = mode.equals("rawDropout") ? rawPose() : robot.follower.pose();
        if (mode.equals("seed")) {
            double offset = pose.distance(SEED);
            double turn = turnDeg(pose, SEED);
            if (!seed.containsKey("firstOffsetIn")) {
                seed.put("firstOffsetIn", offset);
                seed.put("firstTurnDeg", turn);
            }
            maxOffsetIn = Math.max(maxOffsetIn, offset);
            maxTurnDeg = Math.max(maxTurnDeg, turn);
        } else if (running) {
            if (rows.size() < MAX_ROWS) {
                rows.add(new double[] {(System.nanoTime() - startNs) / 1e6, status.ordinal(), pose.x(), pose.y(),
                        Math.toDegrees(pose.heading()), driver.getLoopTime()});
            }
            if (ready) {
                if (badLoops > 0 && !dropout.containsKey("readyAgainMs")) {
                    dropout.put("readyAgainMs", (System.nanoTime() - startNs) / 1e6);
                    dropout.put("poseWhenReadyAgain", pose.toString());
                }
                lastReadyPose = pose;
            } else {
                badLoops++;
                if (!pinpointFaultActive()) silentBadLoops++;
                if (firstBadPose == null) {
                    firstBadPose = pose;
                    dropout.put("firstNotReadyMs", (System.nanoTime() - startNs) / 1e6);
                    dropout.put("firstNotReadyStatus", status.name());
                    dropout.put("firstNotReadyPose", pose.toString());
                    dropout.put("lastReadyPose", String.valueOf(lastReadyPose));
                    dropout.put("poseDuringDropout", describeDropout(pose, lastReadyPose));
                }
            }
        }
        report.status("%s", status.name());
    }

    private void noteStatus(GoBildaPinpointDriver.DeviceStatus status) {
        if (status != lastStatus) {
            statusChanges.add(new Object[] {(System.nanoTime() - createdNs) / 1e6, status.name(),
                    running ? "running" : "init"});
            lastStatus = status;
        }
        Integer count = statusLoops.get(status.name());
        statusLoops.put(status.name(), count == null ? 1 : count + 1);
    }

    private Pose rawPose() {
        return new Pose(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH),
                Math.toRadians(driver.getHeading(AngleUnit.DEGREES)));
    }

    private void observeSoak() {
        long now = System.nanoTime();
        loops++;
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        if (status != lastStatus && statusChanges.size() < SOAK_KEEP) {
            statusChanges.add(new Object[] {(now - createdNs) / 1e6, status.name(), running ? "running" : "init"});
        }
        lastStatus = status;
        boolean ready = status == GoBildaPinpointDriver.DeviceStatus.READY;
        Pose pose = robot.follower.pose();
        streaks.add((now - createdNs) / 1e6, status.name(), ready, running ? "running" : "init",
                ready ? null : pose.toString());
        if (!running) return;
        double msSinceStart = (now - startNs) / 1e6;
        if (timed.updates != timedUpdatesSeen) {
            timedUpdatesSeen = timed.updates;
            double readMs = timed.lastNanos / 1e6;
            readTail.add(readMs);
            followerTail.add((now - clock.readNs) / 1e6);
            slowReads.add(runLoops, msSinceStart, readMs, timed.lastBlockingGcDelta, timed.lastGcDelta);
        }
        if (ready) pinpointDrift.add(msSinceStart / 1000, pose.heading());
        if (runLoops % SOAK_OCTOQUAD_EVERY == 0) {
            double octoDeg = Double.NaN;
            if (octoQuadPresent) {
                octoQuad.readLocalizerData(octoData);
                octoQuadReads++;
                if (octoData.isDataValid()) {
                    octoQuadDrift.add(msSinceStart / 1000, octoData.heading_rad);
                    octoDeg = Math.toDegrees(octoData.heading_rad);
                } else {
                    octoQuadInvalid++;
                }
            }
            if (headingRows.size() < SOAK_MAX_ROWS) {
                headingRows.add(new double[] {msSinceStart, status.ordinal(), Math.toDegrees(pose.heading()), octoDeg});
            }
        }
        if (runLoops % SOAK_STATUS_EVERY == 0) {
            report.status("%s, %d s of %.0f", status.name(), (long) (msSinceStart / 1000), soakSeconds);
        }
        runLoops++;
        if (now - lastSaveNs >= SOAK_SAVE_NS) {
            lastSaveNs = now;
            report.put("soak", soakSummary(false));
            report.save();
        }
    }

    private Map<String, Object> soakSummary(boolean end) {
        double nowMs = (System.nanoTime() - createdNs) / 1e6;
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("ran", running);
        m.put("seconds", running ? (System.nanoTime() - startNs) / 1e9 : 0);
        m.put("askedSeconds", soakSeconds);
        m.put("ended", end);
        m.put("loops", loops);
        m.put("runLoops", runLoops);
        m.put("readMs", readTail.summary());
        m.put("followerUpdateMs", followerTail.summary());
        m.put("loopPeriodMs", loopTail.summary());
        m.put("slowReads", slowReads.summary(readTail.count()));
        m.put("status", streaks.summary(nowMs));
        Map<String, Object> drift = new LinkedHashMap<>();
        drift.put("pinpoint", pinpointDrift.summary());
        drift.put("octoQuad", octoQuadPresent ? octoQuadDrift.summary() : null);
        drift.put("octoQuadReads", octoQuadReads);
        drift.put("octoQuadInvalidReads", octoQuadInvalid);
        m.put("drift", drift);
        return m;
    }

    private void observeRecal() {
        long now = System.nanoTime();
        loops++;
        driver.update();
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        noteStatus(status);
        boolean ready = status == GoBildaPinpointDriver.DeviceStatus.READY;
        boolean octoRunning = false;
        if (octoQuadPresent) {
            octoQuad.readLocalizerData(octoData);
            octoRunning = octoData.isDataValid();
        }
        report.status("%s, OctoQuad %s: %s", status.name(),
                octoQuadPresent ? String.valueOf(octoData.localizerStatus) : "absent", recalStep);
        if (!running) return;
        double stepMs = (now - recalStepNs) / 1e6;
        if (recalStep.equals("settle")) {
            if ((ready && (!octoQuadPresent || octoRunning)) || now - recalStepNs > RECAL_SETTLE_NS) {
                recal.put("settledMs", stepMs);
                recal.put("settledReady", ready);
                recal.put("settledOctoQuadRunning", octoQuadPresent ? octoRunning : null);
                driver.recalibrateIMU();
                if (octoQuadPresent) octoQuad.resetLocalizerAndCalibrateIMU();
                recalStep = "calibrating";
                recalStepNs = System.nanoTime();
                report.phase("recal calibrating");
            }
            return;
        }
        if (recalStep.equals("calibrating")) {
            if (!ready) pinpointLeftReady = true;
            if (octoQuadPresent && !octoRunning) octoQuadLeftRunning = true;
            boolean leaveWindowOver = now - recalStepNs > RECAL_LEAVE_NS;
            if (ready && (pinpointLeftReady || leaveWindowOver) && !recal.containsKey("pinpointRecalMs")) {
                recal.put("pinpointRecalMs", stepMs);
                recal.put("pinpointLeftReady", pinpointLeftReady);
            }
            if (octoQuadPresent && octoRunning && (octoQuadLeftRunning || leaveWindowOver)
                    && !recal.containsKey("octoQuadRecalMs")) {
                recal.put("octoQuadRecalMs", stepMs);
                recal.put("octoQuadLeftRunning", octoQuadLeftRunning);
            }
            boolean pinpointBack = recal.containsKey("pinpointRecalMs");
            boolean octoBack = !octoQuadPresent || recal.containsKey("octoQuadRecalMs");
            if ((pinpointBack && octoBack) || now - recalStepNs > RECAL_BACK_NS) {
                recal.put("backWithinLimit", pinpointBack && octoBack);
                recalStep = "hold";
                recalStepNs = System.nanoTime();
                nextRecalRowNs = recalStepNs;
                report.phase("recal hold");
            }
            return;
        }
        if (recalStep.equals("hold")) {
            double seconds = (now - recalStepNs) / 1e9;
            if (now >= nextRecalRowNs) {
                nextRecalRowNs += RECAL_ROW_NS;
                double heading = Math.toRadians(driver.getHeading(AngleUnit.DEGREES));
                if (ready) pinpointDrift.add(seconds, heading);
                if (octoRunning) octoQuadDrift.add(seconds, octoData.heading_rad);
                if (headingRows.size() < SOAK_MAX_ROWS) {
                    headingRows.add(new double[] {seconds * 1000, status.ordinal(), Math.toDegrees(heading),
                            octoRunning ? Math.toDegrees(octoData.heading_rad) : Double.NaN});
                }
            }
            if (seconds >= RECAL_HOLD_S) recalStep = "done";
        }
    }

    private boolean pinpointFaultActive() {
        for (Faults.Fault f : activeFaults()) {
            if ((f.key + " " + f.message).toLowerCase().contains("pinpoint")) return true;
        }
        return false;
    }

    static String describeDropout(Pose pose, Pose before) {
        if (Math.abs(pose.x()) < 0.01 && Math.abs(pose.y()) < 0.01 && Math.abs(Math.sin(pose.heading())) < 1e-3) {
            return "zeroed";
        }
        if (before != null && pose.distance(before) < 0.01 && turnDeg(pose, before) < 0.01) return "frozen";
        return "moved";
    }

    static double turnDeg(Pose a, Pose b) {
        return Math.toDegrees(Angle.smallestDifference(a.heading(), b.heading()));
    }

    void finish() {
        if (finished || report == null) return;
        finished = true;
        if (!present) {
            report.save();
            return;
        }
        report.put("statusLoops", statusLoops);
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        report.put("finalStatus", status.name());
        if (mode.equals("soak") || mode.equals("recal")) {
            if (mode.equals("soak")) {
                report.put("soak", soakSummary(true));
            } else {
                recal.put("step", recalStep);
                recal.put("ran", running);
                recal.put("pinpointDrift", pinpointDrift.summary());
                recal.put("octoQuadDrift", octoQuadPresent ? octoQuadDrift.summary() : null);
                report.put("recal", recal);
            }
            report.put("statusOrdinals", GoBildaPinpointDriver.DeviceStatus.values());
            report.put("rowColumns", new String[] {mode.equals("soak") ? "msSinceStart" : "msSinceHold", "status",
                    "pinpointHeadingDeg", "octoQuadHeadingDeg"});
            report.put("rows", headingRows);
            report.save();
            return;
        }
        report.put("frequencyHz", frequencyHz.summary());
        report.put("loopPeriodReadyMs", loopReadyMs.summary());
        report.put("loopPeriodNotReadyMs", loopNotReadyMs.summary());
        Pose pose = mode.equals("rawDropout") ? rawPose() : robot.follower.pose();
        report.put("finalPose", pose.toString());
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
            if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                Map<String, Object> after = new LinkedHashMap<>();
                after.put("xOffsetIn", driver.getXOffset(DistanceUnit.INCH));
                after.put("yOffsetIn", driver.getYOffset(DistanceUnit.INCH));
                after.put("yawScalar", driver.getYawScalar());
                dropout.put("afterReplug", after);
            } else {
                dropout.put("afterReplug", "not read: status " + status.name() + " at STOP");
            }
            dropout.put("statusOrdinals", GoBildaPinpointDriver.DeviceStatus.values());
            dropout.put("rowColumns", new String[] {"msSinceStart", "status", "x", "y", "headingDeg", "loopTimeUs"});
            dropout.put("rows", rows);
            report.put("dropout", dropout);
        }
        report.save();
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(BenchReport.KEY_LOOP, loops);
        if (lastStatus != null) telemetry.addData(KEY_STATUS, lastStatus.name());
        if (running && present && (mode.equals("dropout") || mode.equals("rawDropout")) && !finished) {
            telemetry.addData(BenchReport.KEY_DONE, "dropout: recording until STOP");
        }
    }
}
