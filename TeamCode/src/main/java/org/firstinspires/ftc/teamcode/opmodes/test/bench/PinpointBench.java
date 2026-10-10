package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Pinpoint", group = "Test")
public class PinpointBench extends EnhancedOpMode {
    static final String KEY_STATUS = "Bench pinpoint status";
    static final String KEY_LOOP = "Bench loop";
    private static final Pose SEED = new Pose(56, 8, Math.PI / 2);
    private static final double SEED_RUN_S = 3;
    private static final int MAX_ROWS = 6000;

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

    private static final class Saver extends Module {
        private final PinpointBench bench;

        Saver(PinpointBench bench) {
            this.bench = bench;
            setTelemetryEnabled(false);
        }

        @Override protected void initStates() {}
        @Override protected void read() {}
        @Override protected void write() {}

        @Override
        public void stop() {
            bench.finish();
        }
    }

    private final Saver saver = new Saver(this);
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

    @Override
    protected Robot createRobot() {
        mode = BenchIO.param(BenchIO.params(), "mode", "seed");
        if (!mode.equals("seed") && !mode.equals("dropout")) {
            throw new IllegalArgumentException("bench param mode must be seed or dropout, was " + mode);
        }
        report = new BenchReport("Pinpoint", "pinpoint_" + mode);
        driver = BenchIO.require(hardwareMap, GoBildaPinpointDriver.class, BettaConstants.pinpoint.name, BenchIO.RUNNER_HINT);
        int version = driver.getDeviceVersion();
        present = version != 0;
        report.put("deviceVersion", version);
        report.put("present", present);
        if (!present) {
            report.done("no Pinpoint answered on " + BettaConstants.pinpoint.name + "'s I2C port");
            return new BenchRobot(this);
        }
        createStartNs = System.nanoTime();
        PinpointRobot pinpointRobot = new PinpointRobot(this);
        createdNs = System.nanoTime();
        return pinpointRobot;
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
        info.put("seed", SEED.toString());
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
            (lastStatus == GoBildaPinpointDriver.DeviceStatus.READY ? loopReadyMs : loopNotReadyMs).add((now - lastLoopNs) / 1e6);
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
        if (!present) return;
        Pose pose = robot.follower.pose();
        seed.put("offsetAtStartIn", offsetIn(pose, SEED));
        seed.put("turnAtStartDeg", turnDeg(pose, SEED));
    }

    @Override
    protected void gameLoop() {
        observe();
        if (present && mode.equals("seed") && !finished && (System.nanoTime() - startNs) / 1e9 >= SEED_RUN_S) {
            finish();
            report.done(String.format("start pose held within %.3f in and %.3f deg through START + %.0f s",
                    maxOffsetIn, maxTurnDeg, SEED_RUN_S));
        }
    }

    private void observe() {
        if (!present || finished) return;
        loops++;
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        double msSinceCreate = (System.nanoTime() - createdNs) / 1e6;
        if (status != lastStatus) {
            statusChanges.add(new Object[] {msSinceCreate, status.name(), running ? "running" : "init"});
            lastStatus = status;
        }
        Integer count = statusLoops.get(status.name());
        statusLoops.put(status.name(), count == null ? 1 : count + 1);
        boolean ready = status == GoBildaPinpointDriver.DeviceStatus.READY;
        if (ready) frequencyHz.add(driver.getFrequency());
        Pose pose = robot.follower.pose();
        if (mode.equals("seed")) {
            double offset = offsetIn(pose, SEED);
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
        if (before != null && offsetIn(pose, before) < 0.01 && turnDeg(pose, before) < 0.01) return "frozen";
        return "moved";
    }

    static double offsetIn(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    static double turnDeg(Pose a, Pose b) {
        double d = a.heading() - b.heading();
        return Math.abs(Math.toDegrees(Math.atan2(Math.sin(d), Math.cos(d))));
    }

    void finish() {
        if (finished || report == null) return;
        finished = true;
        if (!present) {
            report.save();
            return;
        }
        report.put("statusLoops", statusLoops);
        report.put("frequencyHz", frequencyHz.summary());
        report.put("loopPeriodReadyMs", loopReadyMs.summary());
        report.put("loopPeriodNotReadyMs", loopNotReadyMs.summary());
        Pose pose = robot.follower.pose();
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        report.put("finalPose", pose.toString());
        report.put("finalStatus", status.name());
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
        telemetry.addData(KEY_LOOP, loops);
        if (lastStatus != null) telemetry.addData(KEY_STATUS, lastStatus.name());
        if (running && present && mode.equals("dropout") && !finished) {
            telemetry.addData(BenchReport.KEY_DONE, "dropout: recording until STOP");
        }
    }
}
