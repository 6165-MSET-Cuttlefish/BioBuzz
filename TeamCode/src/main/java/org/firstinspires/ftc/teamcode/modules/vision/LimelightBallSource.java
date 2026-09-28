package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Tracks the detections a ball SnapScript (limelight/ball_*_snapscript.py) packs into llpython. OpMode thread only. */
public final class LimelightBallSource {

    @Config("LimelightBalls")
    public static class Tuning {
        /** 4 runs ball_contour_snapscript.py, 3 ball_detection_snapscript.py (Hough). */
        public static int pipeline = 4;
    }

    public static final String LIMELIGHT_NAME = "limelight";

    private static final double HOUGH_SCRIPT_ID = 6165;
    private static final double CONTOUR_SCRIPT_ID = 6166;
    private static final double FPS_SMOOTHING = 0.1;

    public static final class Frame {
        public static final Frame EMPTY = new Frame(Collections.<TrackedBall>emptyList(), 0, 0, 0, 0);

        public final List<TrackedBall> balls;
        public final int detectionCount;
        public final double fps;
        public final double latencyMs;
        /** Capture time on the System.nanoTime() clock, the one RobotStateHistory is stamped on; 0 before the first frame. */
        public final double timestampSeconds;

        Frame(List<TrackedBall> balls, int detectionCount, double fps, double latencyMs, double timestampSeconds) {
            this.balls = Collections.unmodifiableList(balls);
            this.detectionCount = detectionCount;
            this.fps = fps;
            this.latencyMs = latencyMs;
            this.timestampSeconds = timestampSeconds;
        }
    }

    private final Limelight3A limelight;
    private final BallTracker tracker = new BallTracker();

    private Frame latest = Frame.EMPTY;
    private int pipeline = -1;
    private double lastLimelightTimestamp = Double.NaN;
    private double fps = 0;

    public LimelightBallSource(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
    }

    /** True when a new frame arrived; {@link #latest()} then holds it. */
    public boolean update() {
        // Also restarts polling after stop(), which a Module may get mid-OpMode.
        if (!limelight.isRunning()) limelight.start();
        if (Tuning.pipeline != pipeline) switchPipeline(Tuning.pipeline);

        LLResult result = limelight.getLatestResult();
        // Right after a switch the Limelight still reports the old pipeline for a few frames.
        if (result == null || result.getPipelineIndex() != pipeline) return false;
        if (result.getTimestamp() == lastLimelightTimestamp) return false;
        double[] out = result.getPythonOutput();
        if (out == null || out.length == 0) return false;
        lastLimelightTimestamp = result.getTimestamp();

        if (out[0] != HOUGH_SCRIPT_ID && out[0] != CONTOUR_SCRIPT_ID) {
            throw new IllegalStateException("Limelight pipeline " + pipeline
                    + " isn't running a ball SnapScript (llpython[0] = " + out[0] + ")");
        }
        int count = (int) out[1];
        if (count < 0 || 2 + 3 * count > out.length) {
            throw new IllegalStateException("Ball SnapScript reported " + count
                    + " balls in " + out.length + " llpython slots");
        }
        List<BallDetection> detections = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            int base = 2 + 3 * i;
            detections.add(new BallDetection(BallType.fromCode((int) out[base]), out[base + 1], out[base + 2]));
        }

        // Staleness counts from when the hub received the result; the latencies cover capture to publish.
        double latencyMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        double captureSeconds = System.nanoTime() * 1e-9 - latencyMs / 1000.0;
        if (latest.timestampSeconds > 0) {
            double dt = captureSeconds - latest.timestampSeconds;
            if (dt > 0) fps += FPS_SMOOTHING * (1.0 / dt - fps);
        }

        latest = new Frame(tracker.update(detections, captureSeconds), count, fps, latencyMs, captureSeconds);
        return true;
    }

    /** Never null; {@link Frame#EMPTY} until the first frame lands. */
    public Frame latest() { return latest; }

    /** Stale tracks would match whatever is in frame next and derive velocity across the unseen gap. */
    public void reset() {
        tracker.reset();
        latest = Frame.EMPTY;
        fps = 0;
    }

    public void stop() {
        limelight.stop();
    }

    private void switchPipeline(int index) {
        if (!limelight.pipelineSwitch(index)) {
            throw new IllegalStateException("Limelight rejected the switch to pipeline " + index);
        }
        pipeline = index;
        lastLimelightTimestamp = Double.NaN;
        reset();
    }
}
