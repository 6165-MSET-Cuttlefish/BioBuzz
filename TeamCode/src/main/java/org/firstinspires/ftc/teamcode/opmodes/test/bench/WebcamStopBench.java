package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * BioBuzzRobot's modules (Drivetrain, then the real CellTipCamera) on the software-localizer follower, for the
 * framework's real stop path: run_bench.py presses STOP at chosen delays after INIT or START and watches whether
 * the SDK force-stops the OpMode or restarts the app. Each stop step rewrites webcam_stop_&lt;runId&gt;.json, so
 * the file shows how far the pass got even if the SDK kills it. With the webcam unplugged, INIT must fail with
 * CellTipCamera's "failed to open" error.
 */
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
            drivetrain = new Drivetrain(opMode.hardwareMap).withFollower(follower);
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
    private static final class StopClock extends Module {
        private final WebcamStopBench bench;

        StopClock(WebcamStopBench bench) {
            this.bench = bench;
            setTelemetryEnabled(false);
        }

        @Override protected void initStates() {}
        @Override protected void read() {}
        @Override protected void write() {}

        @Override
        public void stop() {
            long now = System.nanoTime();
            bench.mark("moduleStopsStartMs", now);
            bench.results.put("sinceLastTelemetryHookMs", bench.lastLoopNs == 0 ? null : (now - bench.lastLoopNs) / 1e6);
            bench.results.put("wasStarted", bench.startNs != 0);
            bench.results.put("framesFreshAtStop", bench.fresh());
            long w0 = System.nanoTime();
            bench.save();
            bench.results.put("resultWriteMs", (System.nanoTime() - w0) / 1e6);
        }
    }

    private final StopClock clock = new StopClock(this);
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
