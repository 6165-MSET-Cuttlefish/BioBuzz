package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

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

    private static final class StopMark extends Module {
        private final PlannerStopBench bench;

        StopMark(PlannerStopBench bench) {
            this.bench = bench;
            setTelemetryEnabled(false);
        }

        @Override protected void initStates() {}
        @Override protected void read() {}
        @Override protected void write() {}

        @Override
        public void stop() {
            bench.stopPassStarted(System.nanoTime());
        }
    }

    private final StopMark stopMark = new StopMark(this);
    private final PlannerWorkload workload = new PlannerWorkload();
    private final List<Double> planMs = new ArrayList<>();
    private final Map<String, Object> stopResult = new LinkedHashMap<>();
    private BenchReport report;
    private String runId;
    private PlannerWorkload.Layout layout;
    private Field stopRequested;
    private Thread watcher;
    private long lastPlanStartNs;
    private long lastPlanEndNs;
    private volatile int planNumber;
    private volatile long planStartNs;
    private volatile long stopSeenNs;
    private volatile int stopSeenDuringPlan;
    private volatile long stopSeenPlanStartNs;

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
        info.put("warmPlans", WARM_PLANS);
        info.put("firstPlanInProcess", !PlannerWorkload.plannedBefore());
        report.put("info", info);
        report.put("planMs", planMs);
        report.put("stop", stopResult);
        stopRequested = stopRequestedField();
        watcher = new Thread(this::watchForStop, "bench-stop-watch");
        watcher.setDaemon(true);
        watcher.start();
        report.phase("planning");
        report.save();
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
                    stopSeenPlanStartNs = start;
                    stopSeenDuringPlan = start == 0 ? 0 : plan;
                    stopSeenNs = now;
                    BenchIO.log("planner_stop %s STOP seen %s", runId, start == 0 ? "between plans"
                            : String.format("%.1f ms into plan %d", (now - start) / 1e6, plan));
                    return;
                }
                Thread.sleep(1);
            }
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    protected void initializeLoop() {
        long now = System.nanoTime();
        if (lastPlanStartNs != 0 && (now - lastPlanStartNs) / 1e6 < REPLAN_INTERVAL_MS) return;
        lastPlanStartNs = now;
        planNumber = planMs.size() + 1;
        planStartNs = now;
        PlannerWorkload.Timing timing = workload.plan(layout);
        long end = System.nanoTime();
        planStartNs = 0;
        lastPlanEndNs = end;
        double ms = (end - now) / 1e6;
        planMs.add(ms);
        if (planMs.size() == 1) report.put("ballsVisited", timing.visited);
        BenchIO.log("planner_stop %s plan %d %.1f ms", runId, planMs.size(), ms);
        report.status("%d plans, last %.0f ms", planMs.size(), ms);
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
            if (plan != 0 && plan == planMs.size()) {
                stopResult.put("msIntoPlanAtStop", (seen - stopSeenPlanStartNs) / 1e6);
                stopResult.put("planAtStopMs", planMs.get(plan - 1));
                stopResult.put("planLeftAtStopMs", (lastPlanEndNs - seen) / 1e6);
            }
        }
        BenchIO.log("planner_stop %s stop() pass: %s", runId, stopResult);
        report.save();
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData(KEY_PLANS, planMs.size());
        if (planMs.size() >= WARM_PLANS) telemetry.addData(BenchReport.KEY_DONE, "warm; the runner presses STOP mid-plan");
    }
}
