package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.planner.VisibilityGraphPlanner;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.CellTipPipeline;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Planner Stop", group = "Test")
public class PlannerStopBench extends EnhancedOpMode {
    static final String KEY_PLANS = "Bench plans";
    static final String KEY_FRAMES = "Bench frames";
    private static final int WARM_PLANS = 3;
    private static final double REPLAN_INTERVAL_MS = 250;
    private static final long WATCH_LIMIT_NS = 600_000_000_000L;

    static final class CameraRobot extends BenchRobot {
        TimedCellTip cellTip;

        CameraRobot(PlannerStopBench bench) {
            super(bench);
        }

        @Override
        protected void initializeGameModules() {
            super.initializeGameModules();
            cellTip = new TimedCellTip(opMode.hardwareMap, (PlannerStopBench) opMode);
        }
    }

    /** The real module, as BioBuzzRobot builds it; only its stop() is timed and its fresh verdicts counted. */
    static final class TimedCellTip extends CellTipCamera {
        private final PlannerStopBench bench;
        private CellTipPipeline pipeline;
        private double lastVerdictSeconds;
        volatile int frames;

        TimedCellTip(HardwareMap hardwareMap, PlannerStopBench bench) {
            super(hardwareMap);
            this.bench = bench;
        }

        @Override
        protected void read() {
            super.read();
            if (pipeline == null) pipeline = pipelineOf(this);
            double t = pipeline.latest().timestampSeconds;
            if (t == 0 || t == lastVerdictSeconds) return;
            lastVerdictSeconds = t;
            if (++frames == 1) bench.mark("firstFrameMs", System.nanoTime());
        }

        @Override
        public void stop() {
            long t0 = System.nanoTime();
            bench.mark("cellTipStopStartMs", t0);
            try {
                super.stop();
            } finally {
                bench.camera.put("cellTipStopMs", (System.nanoTime() - t0) / 1e6);
                bench.save();
            }
        }

        private static CellTipPipeline pipelineOf(CellTipCamera camera) {
            try {
                Field f = CellTipCamera.class.getDeclaredField("pipeline");
                f.setAccessible(true);
                return (CellTipPipeline) f.get(camera);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                throw new IllegalStateException("CellTipCamera has no readable pipeline field; Bench: Planner Stop "
                        + "counts webcam frames by it and needs updating", e);
            }
        }
    }

    private final StopHook stopMark = new StopHook(() -> stopPassStarted(System.nanoTime()));
    private final long constructedNs = System.nanoTime();
    private final PlannerWorkload workload = new PlannerWorkload();
    private final List<Double> planMs = new ArrayList<>();
    private final List<Double> planComputeMs = new ArrayList<>();
    private final List<Integer> planLoops = new ArrayList<>();
    private final Samples stepMs = new Samples();
    private final Samples overrunMs = new Samples();
    private final Samples loopPeriodMs = new Samples();
    private final Map<String, Object> stopResult = new LinkedHashMap<>();
    private final Map<String, Object> timeline = new LinkedHashMap<>();
    final Map<String, Object> camera = new LinkedHashMap<>();
    private BenchReport report;
    private String runId;
    private boolean useCamera;
    private boolean planInInit = true;
    private int framesBeforePlanning;
    private boolean sliceChanged;
    private double sliceMsBefore;
    private PlannerWorkload.Layout layout;
    private PlannerWorkload.Planning planning;
    private Field stopRequested;
    private Thread watcher;
    private long lastRequestNs;
    private long lastLoopNs;
    private boolean steppedLastLoop;
    private long lastStepStartNs;
    private long lastStepEndNs;
    private long stopPassNs;
    private volatile int planNumber;
    private volatile long planStartNs;
    private volatile long stepStartNs;
    private volatile long stopSeenNs;
    private volatile int stopSeenDuringPlan;
    private volatile long stopSeenPlanStartNs;
    private volatile long stopSeenStepStartNs;
    private volatile Throwable watcherFailure;

    @Override
    protected Robot createRobot() {
        useCamera = BenchIO.param(BenchIO.params(), "camera", 0) != 0;
        return useCamera ? new CameraRobot(this) : new BenchRobot(this);
    }

    @Override
    protected void initialize() {
        Map<String, Object> params = BenchIO.params();
        boolean unsliced = BenchIO.param(params, "unsliced", 0) != 0;
        if (unsliced) {
            sliceMsBefore = VisibilityGraphPlanner.routeSearchSliceMs;
            sliceChanged = true;
            // Infinity, not 0: a step ends once this much time has passed, so 0 would plan one advance per step.
            VisibilityGraphPlanner.routeSearchSliceMs = Double.POSITIVE_INFINITY;
        }
        runId = BenchIO.param(params, "runId", "manual");
        int balls = (int) BenchIO.param(params, "balls", 4);
        planInInit = !"start".equals(BenchIO.param(params, "planFrom", "init"));
        framesBeforePlanning = (int) BenchIO.param(params, "framesBeforePlanning", 0);
        report = new BenchReport("Planner Stop", "planner_stop_" + runId);
        mark("initializeMs", System.nanoTime());
        layout = workload.bothSidesOfTheRail(balls);
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("layout", layout.describe());
        info.put("balls", balls);
        info.put("replanIntervalMs", REPLAN_INTERVAL_MS);
        info.put("sliceMs", VisibilityGraphPlanner.routeSearchSliceMs);
        info.put("unsliced", unsliced);
        info.put("planFrom", planInInit ? "init" : "start");
        info.put("camera", useCamera);
        info.put("framesBeforePlanning", framesBeforePlanning);
        info.put("warmPlans", WARM_PLANS);
        info.put("firstPlanInProcess", !PlannerWorkload.plannedBefore());
        report.put("info", info);
        report.put("planMs", planMs);
        report.put("planComputeMs", planComputeMs);
        report.put("planLoops", planLoops);
        report.put("stop", stopResult);
        report.put("timeline", timeline);
        if (useCamera) report.put("camera", camera);
        stopRequested = stopRequestedField();
        watcher = new Thread(this::watchForStop, "bench-stop-watch");
        watcher.setDaemon(true);
        watcher.start();
        report.phase("planning");
        save();
    }

    private static Field stopRequestedField() {
        try {
            Field f = OpMode.class.getSuperclass().getDeclaredField("stopRequested");
            f.setAccessible(true);
            return f;
        } catch (NoSuchFieldException e) {
            throw new IllegalStateException("the SDK's OpModeInternal has no stopRequested field; Bench: Planner Stop needs updating", e);
        }
    }

    void mark(String key, long nanos) {
        timeline.put(key, (nanos - constructedNs) / 1e6);
    }

    private int frames() {
        return useCamera ? ((CameraRobot) robot).cellTip.frames : 0;
    }

    private void watchForStop() {
        long deadline = System.nanoTime() + WATCH_LIMIT_NS;
        try {
            while (System.nanoTime() < deadline) {
                if (stopRequested.getBoolean(this)) {
                    long now = System.nanoTime();
                    long start = planStartNs;
                    int plan = planNumber;
                    long step = stepStartNs;
                    stopSeenPlanStartNs = start;
                    stopSeenDuringPlan = start == 0 ? 0 : plan;
                    stopSeenStepStartNs = step;
                    stopSeenNs = now;
                    BenchIO.log("planner_stop %s STOP seen %s", runId, start == 0 ? "between plans"
                            : String.format("%.1f ms into plan %d, %s", (now - start) / 1e6, plan, step == 0
                            ? "between slices" : String.format("%.1f ms into a slice", (now - step) / 1e6)));
                    return;
                }
                Thread.sleep(1);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Throwable t) {
            watcherFailure = t;
        }
    }

    @Override
    protected void initializeLoop() {
        if (planInInit) {
            planLoop();
        } else {
            checkWatcher();
        }
    }

    @Override
    protected void onStart() {
        mark("startMs", System.nanoTime());
        if (useCamera) camera.put("framesAtStart", frames());
        save();
    }

    @Override
    protected void gameLoop() {
        planLoop();
    }

    private void checkWatcher() {
        Throwable failure = watcherFailure;
        if (failure != null) throw new IllegalStateException("Bench: Planner Stop's stop watcher failed", failure);
    }

    // As Vision Ball Collection plans when its balls move after each plan: a new request at the first replan check after it.
    private void planLoop() {
        checkWatcher();
        long now = System.nanoTime();
        if (steppedLastLoop) loopPeriodMs.addNanos(now - lastLoopNs);
        lastLoopNs = now;
        steppedLastLoop = false;
        if (planning == null && frames() < framesBeforePlanning) {
            report.status("waiting for %d webcam frames, %d so far", framesBeforePlanning, frames());
            return;
        }
        if (planning == null && (lastRequestNs == 0 || (now - lastRequestNs) / 1e6 >= REPLAN_INTERVAL_MS)) {
            if (lastRequestNs == 0) {
                mark("firstPlanMs", now);
                if (useCamera) camera.put("framesAtFirstPlan", frames());
            }
            lastRequestNs = now;
            planNumber = planMs.size() + 1;
            planStartNs = now;
            planning = workload.request(layout);
        }
        if (planning == null) return;
        steppedLastLoop = true;
        lastStepStartNs = System.nanoTime();
        stepStartNs = lastStepStartNs;
        PlannerWorkload.Timing timing = planning.step(stepMs, overrunMs);
        lastStepEndNs = System.nanoTime();
        stepStartNs = 0;
        if (timing == null) return;
        planning = null;
        planStartNs = 0;
        planMs.add(timing.planMs);
        planComputeMs.add(timing.computeMs);
        planLoops.add(timing.loops);
        if (planMs.size() == 1) report.put("ballsVisited", timing.visited);
        BenchIO.log("planner_stop %s plan %d %.1f ms over %d loops (%.1f ms in slices)", runId, planMs.size(),
                timing.planMs, timing.loops, timing.computeMs);
        report.status("%d plans, last %.0f ms over %d loops", planMs.size(), timing.planMs, timing.loops);
        save();
    }

    void save() {
        if (report == null) return;
        report.put("stepMs", stepMs.summary());
        report.put("overrunMs", overrunMs.summary());
        report.put("loopPeriodMs", loopPeriodMs.summary());
        report.save();
    }

    void stopPassStarted(long now) {
        try {
            if (watcher == null) return;
            watcher.interrupt();
            stopPassNs = now;
            mark("stopPassMs", now);
            if (useCamera) camera.put("framesAtStop", frames());
            stopResult.put("plans", planMs.size());
            long seen = stopSeenNs;
            stopResult.put("stopSeen", seen != 0);
            if (seen != 0) {
                mark("stopSeenMs", seen);
                int plan = stopSeenDuringPlan;
                stopResult.put("planRunningAtStop", plan == 0 ? null : plan);
                stopResult.put("stopToStopPassMs", (now - seen) / 1e6);
                if (plan != 0) stopResult.put("msIntoPlanAtStop", (seen - stopSeenPlanStartNs) / 1e6);
                long step = stopSeenStepStartNs;
                stopResult.put("inSliceAtStop", step != 0);
                if (step != 0) {
                    stopResult.put("msIntoSliceAtStop", (seen - step) / 1e6);
                    stopResult.put("sliceLeftAtStopMs", (lastStepEndNs - seen) / 1e6);
                }
                // A slice begun after STOP in the same loop also runs: the SDK calls stop() only once the loop returns.
                stopResult.put("sliceAfterStopMs", lastStepEndNs > seen
                        ? (lastStepEndNs - Math.max(seen, lastStepStartNs)) / 1e6 : 0.0);
                stopResult.put("tailMs", (now - Math.max(seen, lastStepEndNs)) / 1e6);
            }
            BenchIO.log("planner_stop %s stop() pass: %s", runId, stopResult);
            save();
        } finally {
            if (sliceChanged) {
                VisibilityGraphPlanner.routeSearchSliceMs = sliceMsBefore;
                sliceChanged = false;
            }
        }
    }

    @Override
    protected void onEnd() {
        long now = System.nanoTime();
        mark("onEndMs", now);
        if (stopPassNs != 0) stopResult.put("stopPassToOnEndMs", (now - stopPassNs) / 1e6);
        save();
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(KEY_PLANS, planMs.size());
        if (useCamera) telemetry.addData(KEY_FRAMES, frames());
        if (planMs.size() >= WARM_PLANS) telemetry.addData(BenchReport.KEY_DONE, "warm; the runner presses STOP mid-plan");
    }
}
