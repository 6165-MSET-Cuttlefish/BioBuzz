package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.modules.vision.LimelightSync;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A plugged-in Limelight 3A (USB-powered) through the SDK's Limelight3A: getStatus latency, LimelightSync putting the
 * ball pipeline on it as INIT does, what results say right after a pipeline switch (pID, llpython), what the blocking POSTs (pipelineSwitch, updatePythonInputs,
 * updateRobotOrientation) cost the calling thread, the ts clock against the hub's, the real result JSON's parse cost,
 * poll rates 100/200/250 Hz (the SDK's cap), and the real LimelightBallSource when the pipeline is switched behind
 * its back. Leaves the Limelight stopped at the SDK's default 100 Hz, on the pipeline it started on; the
 * unplug-and-replug fault timeline is {@link LimelightFaultBench}'s replug mode.
 */
@TeleOp(name = "Bench: Limelight", group = "Test")
public class LimelightBench extends LinearOpMode {
    static final String PLUG_IN = "plug the Limelight 3A into a Control Hub USB port (it is USB-powered), wait for its "
            + "LEDs to settle, and rerun run_bench.py --limelight; its probe adds the Limelight to the bench config";

    private static final int SDK_DEFAULT_POLL_HZ = 100;
    private static final double PROXY_WAIT_S = 6;

    private BenchReport report;
    private Limelight3A ll;
    private final Samples switchMs = new Samples();

    @Override
    public void runOpMode() {
        report = new BenchReport("Limelight", "limelight");
        ll = BenchIO.require(hardwareMap, Limelight3A.class, LimelightBallSource.LIMELIGHT_NAME, PLUG_IN);
        int ballPipeline = (int) BenchIO.param(report.params, "ballPipeline", LimelightBallSource.Tuning.pipeline);
        int otherPipeline = (int) BenchIO.param(report.params, "otherPipeline", 0);
        int proxyPipeline = (int) BenchIO.param(report.params, "proxyPipeline", otherPipeline != 0 ? otherPipeline : 1);

        Map<String, Object> poller = new LinkedHashMap<>();
        poller.put("running", ll.isRunning());
        poller.put("pollIntervalMs", HubBench.pollIntervalMs(ll));
        report.put("pollerAtInit", poller);
        LLStatus initial = waitForStatus();
        report.put("initialStatus", status(initial));
        int originalPipeline = initial.getPipelineIndex();
        int savedTuningPipeline = LimelightBallSource.Tuning.pipeline;

        while (opModeInInit()) {
            report.sendIfDue();
            sleep(20);
        }
        if (!opModeIsActive()) return;

        try {
            proxyQuery(ballPipeline, proxyPipeline);
            statusLatency();
            HubBench.startPolling(ll, SDK_DEFAULT_POLL_HZ);
            sync(ballPipeline, otherPipeline);
            report.phase("pipelineSwitch");
            Map<String, Object> switches = new LinkedHashMap<>();
            switches.put("what", "pipelineSwitch() duration, then every new result for 10 s: when pID changes and "
                    + "whether llpython is all zeros; records are [msSinceSwitch, pID, llpython0, llpython1, ts, cl, tl, stalenessMs]");
            switches.put("toBallPipeline", switchAndWatch(ballPipeline, 10));
            report.endPhase("pipelineSwitch", switches);
            switches.put("toOtherPipeline", switchAndWatch(otherPipeline, 10));
            report.endPhase("pipelineSwitch", switches);
            switches.put("backToBallPipeline", switchAndWatch(ballPipeline, 10));
            report.endPhase("pipelineSwitch", switches);

            postLatency(ballPipeline);
            tsClock(20);
            realJsonParse();
            pollRates();
            behindItsBack(ballPipeline, otherPipeline);
            report.done("Limelight phases complete");
        } finally {
            ll.stop();
            ll.setPollRateHz(SDK_DEFAULT_POLL_HZ);
            LimelightBallSource.Tuning.pipeline = savedTuningPipeline;
            report.put("restoredPipeline", originalPipeline);
            report.put("restoreAccepted", ll.pipelineSwitch(originalPipeline));
            report.save();
        }
        while (opModeIsActive()) {
            report.sendIfDue();
            sleep(20);
        }
    }

    private LLStatus waitForStatus() {
        long deadline = System.currentTimeMillis() + 15000;
        while (true) {
            LLStatus s = ll.getStatus();
            // A failed request returns a default LLStatus, which has no name and a 0 °C temperature.
            if (!s.getName().isEmpty() || s.getTemp() > 0) return s;
            if (System.currentTimeMillis() > deadline) {
                throw new IllegalStateException("The Limelight at the configured address doesn't answer /status: " + PLUG_IN);
            }
            report.sendIfDue();
            sleep(250);
        }
    }

    private static Map<String, Object> status(LLStatus s) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("name", s.getName());
        m.put("pipelineIndex", s.getPipelineIndex());
        m.put("pipelineType", s.getPipelineType());
        m.put("fps", s.getFps());
        m.put("cpu", s.getCpu());
        m.put("tempC", s.getTemp());
        m.put("ram", s.getRam());
        m.put("hwType", s.getHwType());
        return m;
    }

    private void proxyQuery(int from, int to) {
        report.phase("proxyQuery");
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("what", "the runner POSTs /pipeline-switch?index=" + to + " to the hub's port 5807 (slothboard's Limelight "
                + "proxy) while this polls the Limelight's /status directly; indexChanges are [msSinceReady, pipelineIndex]");
        r.put("from", from);
        r.put("to", to);
        r.put("directSwitchToFromAccepted", ll.pipelineSwitch(from));
        long settle = System.nanoTime();
        while (opModeIsActive() && answeredIndex() != from && (System.nanoTime() - settle) / 1e9 < 5) {
            report.sendIfDue();
            sleep(50);
        }
        r.put("onFromBeforePost", answeredIndex() == from);
        report.status("ready: POST /pipeline-switch?index=%d through the proxy", to);
        long t0 = System.nanoTime();
        List<double[]> changes = new ArrayList<>();
        int last = -1;
        double reached = Double.NaN;
        while (opModeIsActive() && (System.nanoTime() - t0) / 1e9 < PROXY_WAIT_S) {
            int index = answeredIndex();
            double ms = (System.nanoTime() - t0) / 1e6;
            if (index != last) {
                changes.add(new double[] {ms, index});
                last = index;
            }
            if (index == to && Double.isNaN(reached)) reached = ms;
            report.sendIfDue();
            sleep(100);
        }
        r.put("indexChanges", changes);
        r.put("reachedToMs", reached);
        r.put("restoreAccepted", ll.pipelineSwitch(from));
        report.endPhase("proxyQuery", r);
    }

    private int answeredIndex() {
        LLStatus s = ll.getStatus();
        return !s.getName().isEmpty() || s.getTemp() > 0 ? s.getPipelineIndex() : -1;
    }

    private void statusLatency() {
        report.phase("statusLatency");
        Samples ms = new Samples();
        LLStatus last = null;
        for (int i = 0; i < 50 && opModeIsActive(); i++) {
            long t0 = System.nanoTime();
            last = ll.getStatus();
            ms.addNanos(System.nanoTime() - t0);
            report.sendIfDue();
        }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("what", "getStatus(): a blocking GET with a 100 ms connect and 100 ms read timeout");
        r.put("getStatusMs", ms.summary());
        if (last != null) r.put("lastStatus", status(last));
        report.endPhase("statusLatency", r);
    }

    private Map<String, Object> switchAndWatch(int target, double seconds) {
        report.status("switch to %d", target);
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("target", target);
        long t0 = System.nanoTime();
        boolean accepted = ll.pipelineSwitch(target);
        switchMs.addNanos(System.nanoTime() - t0);
        m.put("pipelineSwitchMs", (System.nanoTime() - t0) / 1e6);
        m.put("accepted", accepted);
        List<double[]> records = new ArrayList<>();
        double lastTs = Double.NaN;
        double firstOnTarget = Double.NaN;
        double lastOffTarget = Double.NaN;
        double firstNonZeroOnTarget = Double.NaN;
        int onTarget = 0;
        int zerosOnTarget = 0;
        String targetType = null;
        while (opModeIsActive() && (System.nanoTime() - t0) / 1e9 < seconds) {
            LLResult r = ll.getLatestResult();
            if (r.getTimestamp() != lastTs) {
                lastTs = r.getTimestamp();
                double t = (System.nanoTime() - t0) / 1e6;
                double[] py = r.getPythonOutput();
                boolean zeros = true;
                for (double v : py) if (v != 0) zeros = false;
                records.add(new double[] {t, r.getPipelineIndex(), py[0], py[1], r.getTimestamp(),
                        r.getCaptureLatency(), r.getTargetingLatency(), r.getStaleness()});
                if (r.getPipelineIndex() == target) {
                    onTarget++;
                    if (Double.isNaN(firstOnTarget)) {
                        firstOnTarget = t;
                        targetType = r.getPipelineType();
                    }
                    if (zeros) zerosOnTarget++;
                    else if (Double.isNaN(firstNonZeroOnTarget)) firstNonZeroOnTarget = t;
                } else {
                    lastOffTarget = t;
                }
            }
            report.sendIfDue();
            sleep(2);
        }
        m.put("firstResultOnTargetMs", firstOnTarget);
        m.put("lastResultOffTargetMs", lastOffTarget);
        m.put("resultsOnTarget", onTarget);
        m.put("allZeroLlpythonOnTarget", zerosOnTarget);
        m.put("firstNonZeroLlpythonOnTargetMs", firstNonZeroOnTarget);
        m.put("targetPipelineType", targetType);
        m.put("records", records);
        return m;
    }

    /** Each POST as a loop would send it: one call per ~10 ms tick with the poller running at the SDK default. */
    private void postLatency(int ballPipeline) {
        report.phase("postLatency");
        Samples python = new Samples();
        Samples orientation = new Samples();
        Samples sameSwitch = new Samples();
        int pythonOk = 0;
        int orientationOk = 0;
        int sameSwitchOk = 0;
        double[] inputs = {1, 2, 3, 4, 5, 6, 7, 8};
        for (int i = 0; i < 150 && opModeIsActive(); i++) {
            inputs[0] = i;
            long t0 = System.nanoTime();
            if (ll.updatePythonInputs(inputs)) pythonOk++;
            python.addNanos(System.nanoTime() - t0);
            sleep(5);
            long t1 = System.nanoTime();
            if (ll.updateRobotOrientation(i * 2.4 - 180)) orientationOk++;
            orientation.addNanos(System.nanoTime() - t1);
            report.status("%d/150 POST pairs", i + 1);
            report.sendIfDue();
            sleep(5);
        }
        for (int i = 0; i < 20 && opModeIsActive(); i++) {
            long t0 = System.nanoTime();
            if (ll.pipelineSwitch(ballPipeline)) sameSwitchOk++;
            sameSwitch.addNanos(System.nanoTime() - t0);
            report.sendIfDue();
            sleep(50);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "Limelight3A POSTs are synchronous on the calling thread (100 ms connect and 15 s read timeout), so "
                + "each is a loop stall when sent from the OpMode loop; the SDK swallows every failure into a false return. "
                + "Poller running at " + SDK_DEFAULT_POLL_HZ + " Hz; updatePythonInputs sends 8 doubles, updateRobotOrientation a yaw; "
                + "pipelineSwitchSameIndex re-sends pipeline " + ballPipeline + " while on it; pipelineSwitchPhaseMs is the "
                + "pipelineSwitch phase's three switches");
        m.put("updatePythonInputsMs", python.summary());
        m.put("updatePythonInputsReturnedTrue", pythonOk);
        m.put("updateRobotOrientationMs", orientation.summary());
        m.put("updateRobotOrientationReturnedTrue", orientationOk);
        m.put("pipelineSwitchSameIndexMs", sameSwitch.summary());
        m.put("pipelineSwitchSameIndexReturnedTrue", sameSwitchOk);
        m.put("pipelineSwitchPhaseMs", switchMs.summary());
        report.endPhase("postLatency", m);
    }

    private void tsClock(double seconds) {
        report.phase("tsClock");
        List<double[]> pairs = new ArrayList<>();
        Samples cl = new Samples();
        Samples tl = new Samples();
        Samples staleness = new Samples();
        double lastTs = Double.NaN;
        long start = System.nanoTime();
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            LLResult r = ll.getLatestResult();
            if (r.getTimestamp() != lastTs) {
                lastTs = r.getTimestamp();
                // As LimelightBallSource stamps arrival: now on the nanoTime clock minus the poller's staleness.
                double arrival = System.nanoTime() * 1e-9 - r.getStaleness() / 1000.0;
                pairs.add(new double[] {arrival, r.getTimestamp() / 1000.0, r.getCaptureLatency(), r.getTargetingLatency()});
                cl.add(r.getCaptureLatency());
                tl.add(r.getTargetingLatency());
                staleness.add(r.getStaleness());
            }
            report.status("%d results", pairs.size());
            report.sendIfDue();
            sleep(1);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "ts against hub arrival (nanoTime - staleness) for " + seconds + " s: slope 1 means ts is in ms; "
                + "offset minus its minimum is the wait for a poll; captureToArrival is LimelightBallSource's latencyMs");
        m.put("results", pairs.size());
        if (pairs.size() >= 10) {
            double n = pairs.size();
            double mx = 0, my = 0;
            for (double[] p : pairs) {
                mx += p[0];
                my += p[1];
            }
            mx /= n;
            my /= n;
            double sxy = 0, sxx = 0;
            for (double[] p : pairs) {
                sxy += (p[0] - mx) * (p[1] - my);
                sxx += (p[0] - mx) * (p[0] - mx);
            }
            double slope = sxy / sxx;
            m.put("tsSecondsPerHubSecond", slope);
            m.put("driftPpm", (slope - 1) * 1e6);
            double minOffset = Double.POSITIVE_INFINITY;
            for (double[] p : pairs) minOffset = Math.min(minOffset, p[0] - p[1]);
            Samples pollWait = new Samples();
            Samples frameInterval = new Samples();
            Samples captureToArrival = new Samples();
            for (int i = 0; i < pairs.size(); i++) {
                double[] p = pairs.get(i);
                pollWait.add((p[0] - p[1] - minOffset) * 1000);
                if (i > 0) frameInterval.add((p[1] - pairs.get(i - 1)[1]) * 1000);
            }
            for (double[] p : pairs) {
                double capture = p[1] - (p[2] + p[3]) / 1000.0 + minOffset;
                captureToArrival.add((p[0] - capture) * 1000);
            }
            m.put("pollWaitMs", pollWait.summary());
            m.put("limelightFrameIntervalMs", frameInterval.summary());
            m.put("captureToArrivalMs", captureToArrival.summary());
        }
        m.put("captureLatencyClMs", cl.summary());
        m.put("targetingLatencyTlMs", tl.summary());
        m.put("stalenessAtReadMs", staleness.summary());
        report.endPhase("tsClock", m);
    }

    private void realJsonParse() {
        report.phase("realJsonParse");
        String json = ll.getLatestResult().toString();
        Samples jsonMs = new Samples();
        Samples llMs = new Samples();
        for (int i = 0; i < 200; i++) {
            try {
                long t0 = System.nanoTime();
                JSONObject o = new JSONObject(json);
                long t1 = System.nanoTime();
                new ParsedLLResult(o).getPythonOutput();
                llMs.addNanos(System.nanoTime() - t1);
                jsonMs.addNanos(t1 - t0);
            } catch (JSONException e) {
                throw new IllegalStateException("the Limelight's own result JSON didn't parse again", e);
            }
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "the poller's per-result parse on this Limelight's real /results JSON");
        m.put("jsonChars", json.length());
        m.put("jsonObjectMs", jsonMs.summary());
        m.put("llResultMs", llMs.summary());
        m.put("sample", json.length() > 6000 ? json.substring(0, 6000) : json);
        report.endPhase("realJsonParse", m);
    }

    private void pollRates() {
        report.phase("pollRates");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "the SDK poller off, then at each rate up to its 250 Hz cap, for 6 s while this thread runs a fixed "
                + "workload: new frames per second, staleness when a frame is first seen, loop impact, and the busiest threads");
        for (int rate : new int[] {0, 100, 200, 250}) {
            report.status("%d Hz", rate);
            ll.stop();
            if (rate > 0) HubBench.startPolling(ll, rate);
            Samples periods = new Samples();
            Samples work = new Samples();
            Samples staleness = new Samples();
            BenchIO.Cpu cpu = BenchIO.Cpu.now();
            double lastTs = ll.getLatestResult().getTimestamp();
            int fresh = 0;
            double sink = 0;
            long start = System.nanoTime();
            long last = 0;
            while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < 6) {
                long now = System.nanoTime();
                if (last != 0) periods.addNanos(now - last);
                last = now;
                long w0 = System.nanoTime();
                for (int i = 0; i < 20000; i++) sink += Math.sin(i * 0.001) * Math.cos(i * 0.002);
                work.addNanos(System.nanoTime() - w0);
                LLResult r = ll.getLatestResult();
                if (r.getTimestamp() != lastTs) {
                    lastTs = r.getTimestamp();
                    fresh++;
                    staleness.add(r.getStaleness());
                }
                report.sendIfDue();
                sleep(1);
            }
            double seconds = (System.nanoTime() - start) / 1e9;
            Map<String, Object> one = new LinkedHashMap<>();
            one.put("pollIntervalMs", HubBench.pollIntervalMs(ll));
            one.put("newFramesPerSecond", fresh / seconds);
            one.put("stalenessWhenFirstSeenMs", staleness.summary());
            one.put("loopPeriodMs", periods.summary());
            one.put("workloadMs", work.summary());
            one.put("cpu", BenchIO.Cpu.now().since(cpu));
            one.put("checksum", sink);
            m.put(rate == 0 ? "off" : rate + "Hz", one);
            report.endPhase("pollRates", m);
        }
        HubBench.startPolling(ll, SDK_DEFAULT_POLL_HZ);
    }

    /** The INIT sync (park, upload, confirm) twice, each from otherPipeline; ends settled on otherPipeline. */
    private void sync(int ballPipeline, int otherPipeline) {
        report.phase("sync");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "LimelightSync.request(ballPipeline, upload=true) as LimelightBallSource sends it, twice, each "
                + "after a direct switch to otherPipeline: ms until done and its problem (null is success)");
        m.put("stamp", LimelightSync.STAMP);
        for (String run : new String[] {"first", "again"}) {
            if (!opModeIsActive()) break;
            Map<String, Object> one = new LinkedHashMap<>();
            one.put("switchToOtherAccepted", ll.pipelineSwitch(otherPipeline));
            sleep(1000);
            long t0 = System.nanoTime();
            LimelightSync.Request request = LimelightSync.request(ll, ballPipeline, true);
            while (!request.isDone() && opModeIsActive()) {
                report.sendIfDue();
                sleep(10);
            }
            if (!request.isDone()) request.cancel();
            request.rethrowIfCrashed();
            one.put("done", request.isDone());
            one.put("ms", (System.nanoTime() - t0) / 1e6);
            one.put("problem", request.problem());
            m.put(run, one);
            report.endPhase("sync", m);
        }
        if (opModeIsActive()) {
            m.put("switchToOtherAccepted", ll.pipelineSwitch(otherPipeline));
            // The switch phase's first record would otherwise be a result from before this switch.
            sleep(1000);
        }
        report.endPhase("sync", m);
    }

    /** Camera's source on the ball pipeline, then another pipeline selected without telling it, then back. */
    private void behindItsBack(int ballPipeline, int otherPipeline) {
        report.phase("behindItsBack");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "the real LimelightBallSource.update() each loop: 4 s on its pipeline, 5 s after a direct "
                + "pipelineSwitch(" + otherPipeline + ") it doesn't know about, 5 s after switching back; timeline rows are "
                + "[ms, newFrame, stale, detections, fps, frameAgeS] every 100 ms and on every change");
        ll.stop();
        LimelightBallSource.Tuning.pipeline = ballPipeline;
        LimelightBallSource source = new LimelightBallSource(hardwareMap);
        long t0 = System.nanoTime();
        try {
            m.put("firstFrame", firstFrame(source, t0));
            report.endPhase("behindItsBack", m);
            m.put("own", sourceSegment(source, t0, -1, 4));
            report.endPhase("behindItsBack", m);
            m.put("switchedBehindItsBack", sourceSegment(source, t0, otherPipeline, 5));
            report.endPhase("behindItsBack", m);
            m.put("switchedBack", sourceSegment(source, t0, ballPipeline, 5));
        } catch (RuntimeException e) {
            m.put("threw", e.toString());
            throw e;
        } finally {
            source.stop();
            report.endPhase("behindItsBack", m);
        }
    }

    /** INIT to the first frame through the real source, its full sync (park, upload, confirm) included. */
    private Map<String, Object> firstFrame(LimelightBallSource source, long t0) {
        Map<String, Object> seg = new LinkedHashMap<>();
        boolean fresh = false;
        while (opModeIsActive() && !fresh && source.problem() == null && (System.nanoTime() - t0) / 1e9 < 20) {
            fresh = source.update();
            report.sendIfDue();
            sleep(2);
        }
        seg.put("gotFrame", fresh);
        seg.put("ms", (System.nanoTime() - t0) / 1e6);
        seg.put("problem", source.problem());
        return seg;
    }

    private Map<String, Object> sourceSegment(LimelightBallSource source, long t0, int switchTo, double seconds) {
        Map<String, Object> seg = new LinkedHashMap<>();
        if (switchTo >= 0) {
            long s0 = System.nanoTime();
            seg.put("directSwitchAccepted", ll.pipelineSwitch(switchTo));
            seg.put("directSwitchMs", (System.nanoTime() - s0) / 1e6);
        }
        List<double[]> timeline = new ArrayList<>();
        Samples updateMs = new Samples();
        int newFrames = 0;
        int staleLoops = 0;
        long start = System.nanoTime();
        long lastRow = 0;
        boolean lastStale = true;
        try {
            while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
                long u0 = System.nanoTime();
                boolean fresh = source.update();
                long u1 = System.nanoTime();
                updateMs.addNanos(u1 - u0);
                LimelightBallSource.Frame f = source.latest();
                if (fresh) newFrames++;
                if (f.stale) staleLoops++;
                if (u1 - lastRow > 100_000_000L || f.stale != lastStale) {
                    timeline.add(new double[] {(u1 - t0) / 1e6, fresh ? 1 : 0, f.stale ? 1 : 0, f.detectionCount, f.fps,
                            f.ageSeconds()});
                    lastRow = u1;
                    lastStale = f.stale;
                }
                report.sendIfDue();
                sleep(2);
            }
        } finally {
            seg.put("newFrames", newFrames);
            seg.put("staleLoops", staleLoops);
            seg.put("updateMs", updateMs.summary());
            seg.put("timeline", timeline);
        }
        return seg;
    }
}
