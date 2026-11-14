package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightSync;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.TrackedBall;
import org.json.JSONException;
import org.json.JSONObject;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

@TeleOp(name = "Bench: Limelight", group = "Test")
public class LimelightBench extends LinearOpMode {
    static final String PLUG_IN = "plug the Limelight 3A into a Control Hub USB port (it is USB-powered), wait for its "
            + "LEDs to settle, and rerun run_bench.py --limelight; its probe adds the Limelight to the bench config";

    private static final int SDK_DEFAULT_POLL_HZ = 100;
    private static final double PROXY_WAIT_S = 6;
    private static final double STALENESS_S = 60;
    private static final double[] REPLAY_THRESHOLDS_S = {0.4, 0.5, 0.6, 0.7};
    private static final int GAP_BUCKET_MS = 10;
    private static final int GAP_BUCKETS = 100;
    private static final double STILL_S = 30;
    private static final double STILL_RADIUS_IN = 6;
    private static final double IN_FLIGHT_S = 0.1;
    private static final double LAST_CHANGE_S = 5;
    private static final String SENDER_THREAD = "LimelightSync";

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
        poller.put("pollIntervalMs", BenchLimelight.pollIntervalMs(ll));
        report.put("pollerAtInit", poller);
        LLStatus initial = waitForStatus();
        report.put("initialStatus", status(initial));
        int originalPipeline = initial.getPipelineIndex();
        int savedTuningPipeline = LimelightBallSource.Tuning.pipeline;
        boolean savedSyncOnInit = LimelightBallSource.Tuning.syncOnInit;

        if (!report.holdUntilStart(this)) return;

        try {
            proxyQuery(ballPipeline, proxyPipeline);
            statusLatency();
            BenchLimelight.startPolling(ll, SDK_DEFAULT_POLL_HZ);
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
            ballPhases(ballPipeline, otherPipeline);
            report.done("Limelight phases complete");
        } finally {
            ll.stop();
            ll.setPollRateHz(SDK_DEFAULT_POLL_HZ);
            LimelightBallSource.Tuning.pipeline = savedTuningPipeline;
            LimelightBallSource.Tuning.syncOnInit = savedSyncOnInit;
            report.put("restoredPipeline", originalPipeline);
            report.put("restoreAccepted", ll.pipelineSwitch(originalPipeline));
            report.save();
        }
        report.holdUntilStop(this);
    }

    private LLStatus waitForStatus() {
        long deadline = System.nanoTime() + 15000 * 1_000_000L;
        while (true) {
            LLStatus s = ll.getStatus();
            if (BenchLimelight.answered(s)) return s;
            if (System.nanoTime() > deadline) {
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
        return BenchLimelight.answered(s) ? s.getPipelineIndex() : -1;
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
            if (rate > 0) BenchLimelight.startPolling(ll, rate);
            Samples periods = new Samples();
            Samples work = new Samples();
            Samples staleness = new Samples();
            BenchIO.Cpu cpu = BenchIO.Cpu.now();
            double lastTs = ll.getLatestResult().getTimestamp();
            int fresh = 0;
            double sink = 0;
            long start = System.nanoTime();
            while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < 6) {
                periods.tick(System.nanoTime());
                long w0 = System.nanoTime();
                sink += BenchIO.workload();
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
            one.put("pollIntervalMs", BenchLimelight.pollIntervalMs(ll));
            one.put("newFramesPerSecond", fresh / seconds);
            one.put("stalenessWhenFirstSeenMs", staleness.summary());
            one.put("loopPeriodMs", periods.summary());
            one.put("workloadMs", work.summary());
            one.put("cpu", BenchIO.Cpu.now().since(cpu));
            one.put("checksum", sink);
            m.put(rate == 0 ? "off" : rate + "Hz", one);
            report.endPhase("pollRates", m);
        }
        BenchLimelight.startPolling(ll, SDK_DEFAULT_POLL_HZ);
    }

    /** The INIT sync (park, upload, confirm) twice, then switch only, each from otherPipeline; ends on otherPipeline. */
    private void sync(int ballPipeline, int otherPipeline) {
        report.phase("sync");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "LimelightSync.request(ballPipeline, upload=true) as LimelightBallSource sends it, twice, then "
                + "with upload=false (syncOnInit off: switch only), each after a direct switch to otherPipeline: ms until "
                + "done and its problem (null is success); then a LimelightBallSource with syncOnInit off, from "
                + "otherPipeline to its first frame");
        m.put("stamp", LimelightSync.STAMP);
        for (String run : new String[] {"first", "again", "switchOnly"}) {
            if (!opModeIsActive()) break;
            m.put(run, timedSync(ballPipeline, otherPipeline, !run.equals("switchOnly")));
            report.endPhase("sync", m);
        }
        if (opModeIsActive()) {
            m.put("syncOnInitOff", syncOnInitOffFirstFrame(ballPipeline, otherPipeline));
            report.endPhase("sync", m);
        }
        if (opModeIsActive()) {
            m.put("switchToOtherAccepted", ll.pipelineSwitch(otherPipeline));
            // The switch phase's first record would otherwise be a result from before this switch.
            sleep(1000);
        }
        report.endPhase("sync", m);
    }

    private Map<String, Object> timedSync(int ballPipeline, int otherPipeline, boolean upload) {
        Map<String, Object> one = new LinkedHashMap<>();
        one.put("upload", upload);
        one.put("switchToOtherAccepted", ll.pipelineSwitch(otherPipeline));
        sleep(1000);
        long t0 = System.nanoTime();
        LimelightSync.Request request = LimelightSync.request(ll, ballPipeline, upload);
        while (!request.isDone() && opModeIsActive()) {
            report.sendIfDue();
            sleep(10);
        }
        if (!request.isDone()) request.cancel();
        request.rethrowIfCrashed();
        one.put("done", request.isDone());
        one.put("ms", (System.nanoTime() - t0) / 1e6);
        one.put("problem", request.problem());
        return one;
    }

    /** Leaves the poller running at the SDK rate, which the switch phase reads. */
    private Map<String, Object> syncOnInitOffFirstFrame(int ballPipeline, int otherPipeline) {
        boolean before = LimelightBallSource.Tuning.syncOnInit;
        boolean accepted = ll.pipelineSwitch(otherPipeline);
        sleep(1000);
        ll.stop();
        LimelightBallSource.Tuning.pipeline = ballPipeline;
        LimelightBallSource.Tuning.syncOnInit = false;
        LimelightBallSource source = new LimelightBallSource(hardwareMap);
        Map<String, Object> m;
        try {
            m = firstFrame(source, System.nanoTime());
        } finally {
            source.stop();
            LimelightBallSource.Tuning.syncOnInit = before;
        }
        m.put("switchToOtherAccepted", accepted);
        BenchLimelight.startPolling(ll, SDK_DEFAULT_POLL_HZ);
        return m;
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

    private void ballPhases(int ballPipeline, int otherPipeline) {
        report.phase("staleness");
        ll.stop();
        LimelightBallSource.Tuning.pipeline = ballPipeline;
        LimelightBallSource.Tuning.syncOnInit = true;
        LimelightBallSource source = new LimelightBallSource(hardwareMap);
        try {
            Map<String, Object> first = firstFrame(source, System.nanoTime());
            first.put("syncOnInit", true);
            staleness(source, first);
            if (opModeIsActive()) stillBalls(source);
            if (opModeIsActive()) pipelineChange(source, ballPipeline, otherPipeline);
        } finally {
            source.stop();
        }
    }

    private void staleness(LimelightBallSource source, Map<String, Object> first) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "LimelightBallSource.update() every loop for " + STALENESS_S + " s on the ball pipeline: each new "
                + "frame's arrival on the hub (the source's own: capture time plus latency), the gaps between arrivals, "
                + "loops publishing balls from the first loop that did, and stale episodes, live and replayed at "
                + Arrays.toString(REPLAY_THRESHOLDS_S) + " s");
        m.put("firstFrame", first);
        m.put("liveThresholdS", LimelightBallSource.Tuning.staleFrameSeconds);
        List<Long> arrivals = new ArrayList<>();
        Samples updateMs = new Samples();
        Set<String> problems = new LinkedHashSet<>();
        boolean wasStale = source.latest().stale;
        int liveEpisodes = 0;
        long loops = 0;
        long fromFirstBall = 0;
        long withBalls = 0;
        long firstBallNs = 0;
        long start = System.nanoTime();
        try {
            while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < STALENESS_S) {
                long u0 = System.nanoTime();
                boolean fresh = source.update();
                long u1 = System.nanoTime();
                updateMs.addNanos(u1 - u0);
                LimelightBallSource.Frame f = source.latest();
                if (fresh) arrivals.add(Math.round((f.timestampSeconds + f.latencyMs / 1000) * 1e9));
                if (f.stale && !wasStale) liveEpisodes++;
                wasStale = f.stale;
                if (source.problem() != null) problems.add(source.problem());
                boolean balls = !f.balls.isEmpty();
                if (balls && firstBallNs == 0) firstBallNs = u1;
                if (firstBallNs != 0) {
                    fromFirstBall++;
                    if (balls) withBalls++;
                }
                loops++;
                report.status("%d frames, %d loops with balls", arrivals.size(), withBalls);
                report.sendIfDue();
                sleep(2);
            }
        } finally {
            long[] ns = new long[arrivals.size()];
            for (int i = 0; i < ns.length; i++) ns[i] = arrivals.get(i);
            Samples gaps = new Samples();
            for (int i = 1; i < ns.length; i++) gaps.addNanos(ns[i] - ns[i - 1]);
            Map<String, Object> replay = new LinkedHashMap<>();
            for (double t : REPLAY_THRESHOLDS_S) replay.put(String.valueOf(t), StalenessReplay.episodes(ns, t));
            double[] arrivalMs = new double[ns.length];
            for (int i = 0; i < ns.length; i++) arrivalMs[i] = (ns[i] - start) / 1e6;
            m.put("seconds", (System.nanoTime() - start) / 1e9);
            m.put("loops", loops);
            m.put("frames", ns.length);
            m.put("firstBallLoopMs", firstBallNs == 0 ? null : (firstBallNs - start) / 1e6);
            m.put("loopsFromFirstBall", fromFirstBall);
            m.put("loopsWithBalls", withBalls);
            m.put("liveEpisodes", liveEpisodes);
            m.put("replayEpisodes", replay);
            m.put("smallestCleanThresholdS", StalenessReplay.smallestClean(ns, REPLAY_THRESHOLDS_S));
            m.put("longestGapMs", StalenessReplay.longestGapMs(ns));
            m.put("gapMs", gaps.summary());
            m.put("gapHistogram", gaps.histogram(GAP_BUCKET_MS, GAP_BUCKETS));
            m.put("arrivalsMs", arrivalMs);
            m.put("updateMs", updateMs.summary());
            m.put("problems", new ArrayList<>(problems));
            report.endPhase("staleness", m);
        }
    }

    private void stillBalls(LimelightBallSource source) {
        report.phase("stillBalls");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "the source and its BallTracker for " + STILL_S + " s with the balls standing still; per ball (a "
                + "place, radiusIn wide): the visible tracks' position spread around their mean, the speed the tracker "
                + "reports, the track ids it had and the new frames it was missing from");
        m.put("radiusIn", STILL_RADIUS_IN);
        TrackStats stats = new TrackStats(STILL_RADIUS_IN);
        Set<String> problems = new LinkedHashSet<>();
        long start = System.nanoTime();
        try {
            while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < STILL_S) {
                if (source.update()) {
                    stats.beginFrame();
                    for (TrackedBall b : source.latest().balls) stats.add(b.id, b.x, b.y, b.vx, b.vy, b.visible());
                    stats.endFrame();
                }
                if (source.problem() != null) problems.add(source.problem());
                report.status("%d frames", stats.frames());
                report.sendIfDue();
                sleep(2);
            }
        } finally {
            m.put("seconds", (System.nanoTime() - start) / 1e9);
            m.put("frames", stats.frames());
            m.put("balls", stats.balls());
            m.put("problems", new ArrayList<>(problems));
            report.endPhase("stillBalls", m);
        }
    }

    /**
     * Tuning.pipeline changed under the running source, as a dashboard edit does. Changes away from the ball pipeline
     * only switch (syncOnInit off), so the ball files are never uploaded into the other pipeline.
     */
    private void pipelineChange(LimelightBallSource source, int ballPipeline, int otherPipeline) {
        report.phase("pipelineChange");
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", "Tuning.pipeline changed under the running source: to otherPipeline (switch only) until its sync "
                + "is done, back to the ball pipeline (upload), then, with that sync in flight, otherPipeline "
                + Math.round(IN_FLIGHT_S * 1000) + " ms later and the ball pipeline again " + Math.round(IN_FLIGHT_S * 1000)
                + " ms after that; per change: ms to its sync done and to the first new frame, problem changes [ms, "
                + "text], loop period, LimelightSync threads; senders are [ms, LimelightSync threads] on every change "
                + "of the count, sampled every 100 ms; the cancelled requests' ends are read from the source");
        SourceLoop loop = new SourceLoop(source);
        LimelightSync.Request back = null;
        LimelightSync.Request inFlight = null;
        try {
            Map<String, Object> start = change(loop, ballPipeline, true, 0, 0, 10);
            start.put("what", "no change: the running source to its next new frame");
            m.put("start", start);
            report.endPhase("pipelineChange", m);
            m.put("toOther", change(loop, otherPipeline, false, 0, 1, 10));
            report.endPhase("pipelineChange", m);
            m.put("back", change(loop, ballPipeline, true, IN_FLIGHT_S, -1, IN_FLIGHT_S));
            back = syncOf(source);
            m.put("otherInFlight", change(loop, otherPipeline, false, IN_FLIGHT_S, -1, IN_FLIGHT_S));
            inFlight = syncOf(source);
            m.put("last", change(loop, ballPipeline, true, LAST_CHANGE_S, 1, 15));
            m.put("finalPipelineIndex", ll.getLatestResult().getPipelineIndex());
        } finally {
            m.put("backRequestEnded", ended(back));
            m.put("otherInFlightRequestEnded", ended(inFlight));
            m.put("loopMs", loop.loopMs.summary());
            m.put("sendersMax", loop.sendersMax);
            m.put("senders", loop.senders);
            report.endPhase("pipelineChange", m);
        }
    }

    /**
     * Sets syncOnInit and Tuning.pipeline, then loops the source for at least {@code minS} and, with
     * {@code afterSyncS} >= 0, until that long after its sync is done; never past {@code maxS}.
     */
    private Map<String, Object> change(SourceLoop loop, int target, boolean upload, double minS, double afterSyncS,
                                       double maxS) {
        boolean changes = LimelightBallSource.Tuning.pipeline != target;
        LimelightSync.Request old = syncOf(loop.source);
        LimelightBallSource.Tuning.syncOnInit = upload;
        LimelightBallSource.Tuning.pipeline = target;
        long c0 = System.nanoTime();
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("to", target);
        m.put("upload", upload);
        m.put("atMs", loop.ms(c0));
        int problemsFrom = loop.problems.size();
        Samples periods = new Samples();
        int sendersMax = 0;
        double syncDone = Double.NaN;
        double firstFresh = Double.NaN;
        while (opModeIsActive()) {
            periods.tick(System.nanoTime());
            boolean fresh = loop.step();
            sendersMax = Math.max(sendersMax, loop.lastSenders);
            double s = (System.nanoTime() - c0) / 1e9;
            LimelightSync.Request now = syncOf(loop.source);
            if (changes && Double.isNaN(syncDone) && now != old && now.isDone()) {
                syncDone = s * 1000;
                m.put("problemAtSyncDone", now.problem());
            }
            if (fresh && Double.isNaN(firstFresh)) firstFresh = s * 1000;
            if (s >= maxS) break;
            if (s < minS) continue;
            if (afterSyncS < 0) break;
            if (!changes && !Double.isNaN(firstFresh)) break;
            if (changes && !Double.isNaN(syncDone) && s * 1000 >= syncDone + afterSyncS * 1000) break;
        }
        m.put("syncDoneMs", syncDone);
        m.put("firstFreshMs", firstFresh);
        m.put("syncingAtEnd", loop.source.isSyncing());
        m.put("problems", new ArrayList<>(loop.problems.subList(problemsFrom, loop.problems.size())));
        m.put("problemAtEnd", loop.source.problem());
        m.put("loopMs", periods.summary());
        m.put("sendersMax", sendersMax);
        return m;
    }

    private static Map<String, Object> ended(LimelightSync.Request request) {
        Map<String, Object> m = new LinkedHashMap<>();
        if (request == null) return m;
        m.put("pipeline", request.pipeline);
        m.put("upload", request.upload);
        m.put("done", request.isDone());
        m.put("problem", request.problem());
        return m;
    }

    private final class SourceLoop {
        final LimelightBallSource source;
        final long t0 = System.nanoTime();
        final Samples loopMs = new Samples();
        final List<Object[]> problems = new ArrayList<>();
        final List<double[]> senders = new ArrayList<>();
        int sendersMax;
        int lastSenders = -1;
        long lastSenderSampleNs;
        String problem;

        SourceLoop(LimelightBallSource source) {
            this.source = source;
        }

        double ms(long ns) {
            return (ns - t0) / 1e6;
        }

        boolean step() {
            long now = System.nanoTime();
            loopMs.tick(now);
            boolean fresh = source.update();
            String p = source.problem();
            if (!Objects.equals(p, problem)) {
                problems.add(new Object[] {ms(System.nanoTime()), p});
                problem = p;
            }
            if (now - lastSenderSampleNs >= 100_000_000L) {
                int n = senderThreads();
                sendersMax = Math.max(sendersMax, n);
                if (n != lastSenders) senders.add(new double[] {ms(now), n});
                lastSenders = n;
                lastSenderSampleNs = now;
            }
            report.sendIfDue();
            sleep(2);
            return fresh;
        }
    }

    // Not Thread.getAllStackTraces(): it suspends every thread, which would show up as loop time.
    private static int senderThreads() {
        ThreadGroup root = Thread.currentThread().getThreadGroup();
        while (root.getParent() != null) root = root.getParent();
        Thread[] threads = new Thread[root.activeCount() * 2 + 16];
        int n = 0;
        for (Thread t : Arrays.copyOf(threads, root.enumerate(threads, true))) if (t.getName().equals(SENDER_THREAD)) n++;
        return n;
    }

    static Field syncField() {
        try {
            Field f = LimelightBallSource.class.getDeclaredField("sync");
            if (f.getType() != LimelightSync.Request.class) {
                throw new IllegalStateException("LimelightBallSource.sync is a " + f.getType() + ", not a LimelightSync.Request");
            }
            f.setAccessible(true);
            return f;
        } catch (NoSuchFieldException e) {
            throw new IllegalStateException("LimelightBallSource has no field sync for Bench: Limelight to read", e);
        }
    }

    private static LimelightSync.Request syncOf(LimelightBallSource source) {
        try {
            return (LimelightSync.Request) syncField().get(source);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("LimelightBallSource.sync can't be read", e);
        }
    }
}
