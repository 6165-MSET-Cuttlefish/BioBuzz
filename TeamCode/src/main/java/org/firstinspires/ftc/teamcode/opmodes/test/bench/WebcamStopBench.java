package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;

import java.util.LinkedHashMap;
import java.util.Map;

@TeleOp(name = "Bench: Webcam Stop", group = "Test")
public class WebcamStopBench extends EnhancedOpMode {

    public static class CameraStopRobot extends Robot {
        public Drivetrain drivetrain;
        public TimedCellTipCamera cellTip;

        public CameraStopRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {
            drivetrain = new Drivetrain(opMode.hardwareMap, follower);
            cellTip = new TimedCellTipCamera(opMode.hardwareMap, this);
        }
    }

    /** The real module; only its stop() is timed. */
    public static class TimedCellTipCamera extends CellTipCamera {
        private final CameraStopRobot owner;

        TimedCellTipCamera(HardwareMap hardwareMap, CameraStopRobot owner) {
            super(hardwareMap);
            this.owner = owner;
        }

        @Override
        public void stop() {
            WebcamStopBench bench = (WebcamStopBench) owner.opMode;
            long t0 = System.nanoTime();
            bench.mark("cellTipStopStartMs", t0);
            try {
                super.stop();
            } finally {
                bench.mark("cellTipStopEndMs", System.nanoTime());
                bench.results.put("cellTipStopMs", (System.nanoTime() - t0) / 1e6);
                bench.save();
            }
        }
    }

    /** Discovered before the robot's modules (the OpMode's fields are walked first), so its stop() runs first. */
    private final StopHook clock = new StopHook(this::stopPassStarted);
    private final long constructedNs = System.nanoTime();
    final Map<String, Object> results = new LinkedHashMap<>();
    private String fileName;
    private BenchReport report;
    volatile long lastLoopNs;
    volatile long startNs;
    private long firstFreshNs;

    @Override
    protected Robot createRobot() {
        String runId = BenchIO.param(BenchIO.params(), "runId", "manual");
        fileName = "webcam_stop_" + runId;
        report = new BenchReport("Webcam Stop", fileName + "_live");
        results.put("runId", runId);
        results.put("device", BenchIO.deviceInfo());
        mark("createRobotMs", System.nanoTime());
        save();
        return new CameraStopRobot(this);
    }

    private CameraStopRobot bench() {
        return (CameraStopRobot) robot;
    }

    void mark(String key, long nanos) {
        results.put(key, (nanos - constructedNs) / 1e6);
    }

    void save() {
        BenchIO.write(fileName, results);
    }

    private void stopPassStarted() {
        long now = System.nanoTime();
        mark("moduleStopsStartMs", now);
        results.put("sinceLastTelemetryHookMs", lastLoopNs == 0 ? null : (now - lastLoopNs) / 1e6);
        results.put("wasStarted", startNs != 0);
        results.put("framesFreshAtStop", fresh());
        long w0 = System.nanoTime();
        save();
        results.put("resultWriteMs", (System.nanoTime() - w0) / 1e6);
    }

    boolean fresh() {
        CameraStopRobot r = bench();
        return r != null && r.cellTip != null && (r.cellTip.isTipped() || r.cellTip.isScorable());
    }

    @Override
    protected void initialize() {
        mark("initializeMs", System.nanoTime());
        save();
    }

    @Override
    protected void initializeLoop() {
        watchFrames();
    }

    @Override
    protected void onStart() {
        startNs = System.nanoTime();
        mark("startMs", startNs);
        save();
    }

    @Override
    protected void gameLoop() {
        watchFrames();
    }

    private void watchFrames() {
        if (firstFreshNs == 0 && fresh()) {
            firstFreshNs = System.nanoTime();
            mark("firstFreshVerdictMs", firstFreshNs);
            save();
        }
        report.status("fresh=%s", fresh());
    }

    @Override
    protected void telemetry() {
        lastLoopNs = System.nanoTime();
        report.addTo(telemetry);
        telemetry.addData("Bench fresh", fresh());
    }

    @Override
    protected void onEnd() {
        mark("onEndMs", System.nanoTime());
        results.put("onEndReached", true);
        save();
    }
}
