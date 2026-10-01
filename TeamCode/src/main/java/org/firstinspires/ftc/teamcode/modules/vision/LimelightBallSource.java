package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.Faults;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Tracks the detections limelight/ball_contour_snapscript.py packs into llpython. OpMode thread only.
 * After its first poll, and on each {@link Tuning#pipeline} change, {@link LimelightSync} puts the ball pipeline on
 * the Limelight; the frame is stale until it has. A Limelight problem never throws: it raises the {@link #FAULT_KEY}
 * fault with {@link #problem()} and the frame goes stale until the Limelight recovers, which clears it.
 */
public final class LimelightBallSource {

    @Config("LimelightBalls")
    public static class Tuning {
        /** The pipeline running ball_contour_snapscript.py. */
        public static int pipeline = 4;
        /**
         * Upload limelight/'s ball pipeline and script to {@link #pipeline} at every INIT. Turn off while tuning in
         * the Limelight's web editor, or the next INIT replaces the edits; it turns back on at every deploySloth or
         * app restart, so copy edits into limelight/ first. Off, the file's SCRIPT_ID and any stamp are accepted.
         */
        public static boolean syncOnInit = true;
        /** With no new Limelight frame for this long, the frame goes stale: no balls, none visible. */
        public static double staleFrameSeconds = 0.25;
        /** No successful poll for this long faults; 250 is the SDK's own isConnected() threshold. */
        public static double noDataFaultMs = 250;
        /** Reporting another pipeline for this long faults; a switch takes a few frames to show. */
        public static double pipelineGraceMs = 1000;
    }

    public static final String LIMELIGHT_NAME = "limelight";
    public static final String FAULT_KEY = "Limelight";

    private static final int NO_PIPELINE = Integer.MIN_VALUE;
    private static final double FPS_SMOOTHING = 0.1;
    private static final int OFFSET_WINDOW = 128;
    private static final double TS_UNIT_CHECK_SPAN_SECONDS = 5;
    private static final double TS_UNIT_TOLERANCE = 0.1;
    private static final BallType[] TYPES = BallType.values();

    public static final class Frame {
        public static final Frame EMPTY = new Frame(Collections.<TrackedBall>emptyList(), 0, 0, 0, 0, true);

        public final List<TrackedBall> balls;
        public final int detectionCount;
        public final double fps;
        /** Capture to arrival on the hub. */
        public final double latencyMs;
        /** Capture time on the System.nanoTime() clock, the one RobotStateHistory is stamped on; 0 before the first frame. */
        public final double timestampSeconds;
        /** No live frame: none yet, a Limelight fault, or none new for {@link Tuning#staleFrameSeconds}. */
        public final boolean stale;

        Frame(List<TrackedBall> balls, int detectionCount, double fps, double latencyMs, double timestampSeconds,
              boolean stale) {
            this.balls = Collections.unmodifiableList(balls);
            this.detectionCount = detectionCount;
            this.fps = fps;
            this.latencyMs = latencyMs;
            this.timestampSeconds = timestampSeconds;
            this.stale = stale;
        }

        public double ageSeconds() {
            return timestampSeconds == 0 ? Double.POSITIVE_INFINITY : nowSeconds() - timestampSeconds;
        }
    }

    private final Limelight3A limelight;
    private final BallTracker tracker = new BallTracker();

    private Frame latest = Frame.EMPTY;
    private String problem;
    private int pipeline = NO_PIPELINE;
    private int requested = NO_PIPELINE;
    private LimelightSync.Request sync;
    private double lastTsMs = Double.NaN;
    private double lastCaptureLimelightSeconds = Double.NaN;
    private double fps = 0;

    // The SDK's Limelight3A outlives the OpMode, so its latest result may predate this one; a new result object means a poll succeeded.
    private LLResult resultAtStart;
    private boolean polledSinceStart;
    private double startedSeconds = Double.NaN;
    private double mismatchSinceSeconds = Double.NaN;
    private String outputProblem;
    private String clockProblem;

    // hub arrival - Limelight ts per frame; the smallest is the frame that waited least for a poll.
    private final double[] offsets = new double[OFFSET_WINDOW];
    private int offsetCount = 0;
    private int offsetNext = 0;
    private double clockCheckStartTs = Double.NaN;
    private double clockCheckStartHub = Double.NaN;
    private boolean clockChecked = false;

    public LimelightBallSource(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        resultAtStart = limelight.getLatestResult();
        // Faults outlive a bare LinearOpMode; this source owns the key from here on.
        Faults.clear(FAULT_KEY);
    }

    /** True when a new frame arrived; {@link #latest()} then holds it. */
    public boolean update() {
        double now = nowSeconds();
        // Also restarts polling after stop(), which a Module may get mid-OpMode.
        if (!limelight.isRunning()) {
            limelight.start();
            resultAtStart = limelight.getLatestResult();
            polledSinceStart = false;
            startedSeconds = now;
            // Limelight3A.start() swallows its own failure and stays stopped.
            if (!limelight.isRunning()) {
                setProblem("the SDK's Limelight poller didn't start");
                goStale();
                return false;
            }
        } else if (Double.isNaN(startedSeconds)) {
            startedSeconds = now;
        }

        boolean fresh = false;
        String found;
        if (!polledSinceStart && limelight.getLatestResult() == resultAtStart) {
            double waitedMs = (now - startedSeconds) * 1000;
            found = waitedMs > Tuning.noDataFaultMs
                    ? String.format("never connected (%.1f s since start)", waitedMs / 1000) : null;
            setProblem(found);
            goStale();
            return false;
        }
        polledSinceStart = true;
        found = connectionProblem();
        if (found == null) found = syncProblem();
        if (found == null && !sync.isDone()) {
            setProblem(null);
            goStale();
            return false;
        }
        if (found == null) {
            LLResult result = limelight.getLatestResult();
            if (result.getPipelineIndex() != pipeline) {
                if (Double.isNaN(mismatchSinceSeconds)) mismatchSinceSeconds = now;
                if ((now - mismatchSinceSeconds) * 1000 > Tuning.pipelineGraceMs) {
                    found = String.format("on pipeline %d, want %d", result.getPipelineIndex(), pipeline);
                }
            } else {
                mismatchSinceSeconds = Double.NaN;
                if (result.getTimestamp() != lastTsMs) fresh = accept(result);
                found = outputProblem != null ? outputProblem : clockProblem;
            }
        }
        setProblem(found);
        if (found != null) {
            goStale();
            return false;
        }
        if (!fresh) goStaleIfOld();
        return fresh;
    }

    /** Never null; {@link Frame#EMPTY} until the first frame lands. */
    public Frame latest() { return latest; }

    /** Why the Limelight is unusable, or null while it is healthy; the {@link #FAULT_KEY} fault carries the same text. */
    public String problem() { return problem; }

    /** True until {@link LimelightSync} has put the requested pipeline on the Limelight. */
    public boolean isSyncing() { return sync != null && !sync.isDone(); }

    /** For a caller that stops calling {@link #update()}: drops the tracks and the fault, and the frame goes {@link Frame#EMPTY}. */
    public void idle() {
        mismatchSinceSeconds = Double.NaN;
        setProblem(null);
        if (latest == Frame.EMPTY) return;
        tracker.reset();
        latest = Frame.EMPTY;
        fps = 0;
        lastCaptureLimelightSeconds = Double.NaN;
    }

    public void stop() {
        limelight.stop();
        if (sync != null && !sync.isDone()) {
            sync.cancel();
            requested = NO_PIPELINE;
        }
        setProblem(null);
    }

    private String connectionProblem() {
        long sinceMs = limelight.getTimeSinceLastUpdate();
        return sinceMs > Tuning.noDataFaultMs ? String.format("no data for %.1f s", sinceMs / 1000.0) : null;
    }

    // One sync per requested pipeline, never retried from here: a failure stays up until the next INIT or a pipeline change.
    private String syncProblem() {
        if (Tuning.pipeline != requested) {
            if (sync != null) sync.cancel();
            requested = Tuning.pipeline;
            pipeline = NO_PIPELINE;
            mismatchSinceSeconds = Double.NaN;
            sync = LimelightSync.request(limelight, requested, Tuning.syncOnInit);
        }
        if (!sync.isDone()) return null;
        sync.rethrowIfCrashed();
        if (sync.problem() != null) return sync.problem();
        if (pipeline != requested) {
            pipeline = requested;
            lastTsMs = Double.NaN;
            outputProblem = null;
            mismatchSinceSeconds = Double.NaN;
            dropTracks();
        }
        return null;
    }

    private boolean accept(LLResult result) {
        double tsMs = result.getTimestamp();
        if (tsMs < lastTsMs) {
            // ts counts from the Limelight's boot, so it only goes back after a Limelight reboot.
            resetClock();
            dropTracks();
        }
        lastTsMs = tsMs;

        double[] out = result.getPythonOutput();
        outputProblem = outputProblem(out);
        if (outputProblem != null) return false;
        int count = (int) out[1];
        List<BallDetection> detections = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            int base = 2 + 3 * i;
            detections.add(new BallDetection(BallType.fromCode((int) out[base]), out[base + 1], out[base + 2]));
        }

        // Staleness counts from when the SDK's poller parsed this result, on the wall clock; only its difference is used.
        double arrivalSeconds = nowSeconds() - result.getStaleness() / 1000.0;
        double tsSeconds = tsMs / 1000.0;
        checkTsUnit(tsSeconds, arrivalSeconds);
        if (clockProblem != null) return false;
        double offset = recordOffset(arrivalSeconds - tsSeconds);
        // ts is stamped when the result is published; cl + tl reach back from there to the exposure.
        double captureLimelightSeconds = tsSeconds
                - (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
        double captureSeconds = captureLimelightSeconds + offset;

        if (!Double.isNaN(lastCaptureLimelightSeconds)) {
            double dt = captureLimelightSeconds - lastCaptureLimelightSeconds;
            if (dt > 0) fps += FPS_SMOOTHING * (1.0 / dt - fps);
        }
        lastCaptureLimelightSeconds = captureLimelightSeconds;

        // The tracker only differences its timestamps, so it runs on the Limelight's clock, free of offset jitter.
        List<TrackedBall> balls = tracker.update(detections, captureLimelightSeconds);
        latest = new Frame(balls, count, fps, (arrivalSeconds - captureSeconds) * 1000, captureSeconds, false);
        return true;
    }

    // getPythonOutput() pads to 32 slots with zeros, so a pipeline with no PythonOut reads all zeros.
    // syncOnInit is read live, so turning it off mid-OpMode accepts a web-editor copy at once.
    private String outputProblem(double[] out) {
        boolean uploadOnly = sync.upload && Tuning.syncOnInit;
        boolean known = out[0] == LimelightSync.STAMP
                || (!uploadOnly && (out[0] == LimelightSync.FILE_SCRIPT_ID || LimelightSync.isStamp(out[0])));
        if (!known) {
            if (allZero(out)) return String.format("no script output on pipeline %d (llpython all zeros)", pipeline);
            if (out[0] == LimelightSync.FILE_SCRIPT_ID) {
                return String.format("pipeline %d runs a web-editor copy of the ball script (SCRIPT_ID %s, not the "
                        + "upload's %s); turn LimelightBalls → syncOnInit off while tuning there",
                        pipeline, number(out[0]), number(LimelightSync.STAMP));
            }
            return String.format("unknown SCRIPT_ID %s on pipeline %d, want %s", number(out[0]), pipeline,
                    number(uploadOnly ? LimelightSync.STAMP : LimelightSync.FILE_SCRIPT_ID));
        }
        int count = (int) out[1];
        if (count != out[1] || count < 0 || 2 + 3 * count > out.length) {
            return String.format("script reported %s balls in %d llpython slots", number(out[1]), out.length);
        }
        for (int i = 0; i < count; i++) {
            int base = 2 + 3 * i;
            if (!isTypeCode(out[base])) return String.format("script reported ball type code %s", number(out[base]));
            // The SDK parses a non-numeric llpython entry as NaN.
            if (!Double.isFinite(out[base + 1]) || !Double.isFinite(out[base + 2])) {
                return String.format("script reported ball position (%s, %s)", out[base + 1], out[base + 2]);
            }
        }
        return null;
    }

    private static boolean allZero(double[] out) {
        for (double v : out) if (v != 0) return false;
        return true;
    }

    private static boolean isTypeCode(double code) {
        for (BallType type : TYPES) if (type.code == code) return true;
        return false;
    }

    private static String number(double v) {
        return v == Math.rint(v) && !Double.isInfinite(v) ? String.valueOf((long) v) : String.valueOf(v);
    }

    private void setProblem(String found) {
        if (found == null) {
            if (problem != null) Faults.clear(FAULT_KEY);
        } else {
            Faults.raise(FAULT_KEY, found);
        }
        problem = found;
    }

    private void goStale() {
        if (!latest.stale) dropTracks();
    }

    private void goStaleIfOld() {
        if (!latest.stale && latest.ageSeconds() > Tuning.staleFrameSeconds) dropTracks();
    }

    // Stale tracks would match whatever is in frame next and derive velocity across the unseen gap.
    // The last capture time stays, so a remembered ball's prediction age stays finite.
    private void dropTracks() {
        tracker.reset();
        fps = 0;
        lastCaptureLimelightSeconds = Double.NaN;
        if (!latest.stale) {
            latest = new Frame(Collections.<TrackedBall>emptyList(), 0, 0, latest.latencyMs, latest.timestampSeconds, true);
        }
    }

    private double recordOffset(double offset) {
        offsets[offsetNext] = offset;
        offsetNext = (offsetNext + 1) % OFFSET_WINDOW;
        if (offsetCount < OFFSET_WINDOW) offsetCount++;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < offsetCount; i++) min = Math.min(min, offsets[i]);
        return min;
    }

    /** The SDK only calls ts "Limelight-local monotonic"; Limelight's docs say milliseconds since boot. Re-measured while wrong. */
    private void checkTsUnit(double tsSeconds, double arrivalSeconds) {
        if (clockChecked) return;
        if (Double.isNaN(clockCheckStartTs)) {
            clockCheckStartTs = tsSeconds;
            clockCheckStartHub = arrivalSeconds;
            return;
        }
        double hubSpan = arrivalSeconds - clockCheckStartHub;
        if (hubSpan < TS_UNIT_CHECK_SPAN_SECONDS) return;
        double ratio = (tsSeconds - clockCheckStartTs) / hubSpan;
        if (Math.abs(ratio - 1) > TS_UNIT_TOLERANCE) {
            clockProblem = String.format("ts ran %.3f s per hub second; expected milliseconds", ratio);
            clockCheckStartTs = tsSeconds;
            clockCheckStartHub = arrivalSeconds;
            return;
        }
        clockProblem = null;
        clockChecked = true;
    }

    private void resetClock() {
        offsetCount = 0;
        offsetNext = 0;
        clockCheckStartTs = Double.NaN;
        clockCheckStartHub = Double.NaN;
        clockChecked = false;
        clockProblem = null;
    }

    private static double nowSeconds() {
        return System.nanoTime() * 1e-9;
    }
}
