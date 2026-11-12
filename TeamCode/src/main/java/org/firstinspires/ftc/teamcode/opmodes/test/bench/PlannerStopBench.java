package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.planner.VisibilityGraphPlanner;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Planner Stop", group = "Test")
public class PlannerStopBench extends EnhancedOpMode {
    static final String KEY_PLANS = "Bench plans";
    private static final int WARM_PLANS = 3;
    private static final double REPLAN_INTERVAL_MS = 250;
    private static final long WATCH_LIMIT_NS = 600_000_000_000L;

    private final StopHook stopMark = new StopHook(() -> stopPassStarted(System.nanoTime()));
    private final PlannerWorkload workload = new PlannerWorkload();
    private final List<Double> planMs = new ArrayList<>();
    private final List<Double> planComputeMs = new ArrayList<>();
    private final List<Integer> planLoops = new ArrayList<>();
    private final Samples stepMs = new Samples();
    private final Samples overrunMs = new Samples();
    private final Samples loopPeriodMs = new Samples();
    private final Map<String, Object> stopResult = new LinkedHashMap<>();
    private BenchReport report;
    private String runId;
    private PlannerWorkload.Layout layout;
    private PlannerWorkload.Planning planning;
    private Field stopRequested;
    private Thread watcher;
    private long lastRequestNs;
    private long lastLoopNs;
    private boolean steppedLastLoop;
    private long lastStepEndNs;
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
        return new BenchRobot(this);
    }

    @Override
    protected void initialize() {
        Map<String, Object> params = BenchIO.params();
        runId = BenchIO.param(params, "runId", "manual");
        int balls = (int) BenchIO.param(params, "balls", 4);
        report = new BenchReport("Planner Stop", "planner_stop_" + runId);
        layout = workload.bothSidesOfTheRail(balls);
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("layout", layout.describe());
        info.put("balls", balls);
        info.put("replanIntervalMs", REPLAN_INTERVAL_MS);
        info.put("sliceMs", VisibilityGraphPlanner.routeSearchSliceMs);
        info.put("warmPlans", WARM_PLANS);
        info.put("firstPlanInProcess", !PlannerWorkload.plannedBefore());
        report.put("info", info);
        report.put("planMs", planMs);
        report.put("planComputeMs", planComputeMs);
        report.put("planLoops", planLoops);
        report.put("stop", stopResult);
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

    // As Vision Ball Collection plans when its balls move after each plan: a new request at the first replan check after it.
    @Override
    protected void initializeLoop() {
        Throwable failure = watcherFailure;
        if (failure != null) throw new IllegalStateException("Bench: Planner Stop's stop watcher failed", failure);
        long now = System.nanoTime();
        if (steppedLastLoop) loopPeriodMs.addNanos(now - lastLoopNs);
        lastLoopNs = now;
        steppedLastLoop = false;
        if (planning == null && (lastRequestNs == 0 || (now - lastRequestNs) / 1e6 >= REPLAN_INTERVAL_MS)) {
            lastRequestNs = now;
            planNumber = planMs.size() + 1;
            planStartNs = now;
            planning = workload.request(layout);
        }
        if (planning == null) return;
        steppedLastLoop = true;
        stepStartNs = System.nanoTime();
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

    private void save() {
        report.put("stepMs", stepMs.summary());
        report.put("overrunMs", overrunMs.summary());
        report.put("loopPeriodMs", loopPeriodMs.summary());
        report.save();
    }

    void stopPassStarted(long now) {
        if (watcher == null) return;
        watcher.interrupt();
        stopResult.put("plans", planMs.size());
        long seen = stopSeenNs;
        stopResult.put("stopSeen", seen != 0);
        if (seen != 0) {
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
        }
        BenchIO.log("planner_stop %s stop() pass: %s", runId, stopResult);
        save();
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(KEY_PLANS, planMs.size());
        if (planMs.size() >= WARM_PLANS) telemetry.addData(BenchReport.KEY_DONE, "warm; the runner presses STOP mid-plan");
    }
}
