package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamSession;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetection;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetectionPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.BallFieldTransform;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateHistory;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateSource;
import org.firstinspires.ftc.teamcode.modules.vision.TrackedBall;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Webcam ball-vision subsystem: owns the camera, runs {@link BallDetectionPipeline} on it, and
 * exposes the tracked balls — position <em>and</em> velocity in field inches — to robot code.
 *
 * <p>The pipeline runs on the camera thread at its own frame rate; {@link #read()} takes a snapshot
 * of its latest results so everything downstream of it sees one consistent frame for the whole
 * OpMode loop.
 *
 * <p>Results come in two frames. {@link #getBalls()} is camera-relative — what the homography
 * measured, with the robot's own motion still in it. Give the module a robot pose source
 * ({@link #withFollower}) and {@link #getFieldBalls()} additionally gives field coordinates and
 * true over-the-ground velocity, transformed with the robot state from the instant the frame was
 * captured rather than the instant it was read.
 */
@Config
public class Camera extends Module {

    public static boolean cameraTelemetry = true;
    public static boolean ballTelemetry = true;

    /** Seconds of lookahead used by {@link #getPredictedBallPosition}. */
    public static double defaultLookaheadSeconds = 0.25;
    /** A snapshot older than this is reported stale — the camera thread has stalled or died. */
    public static double staleFrameSeconds = 0.5;

    private static final String DEFAULT_WEBCAM_NAME = "nerdDetector";

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final HardwareMap hardwareMap;
    private final String webcamName;
    private final BallDetectionPipeline pipeline = new BallDetectionPipeline();
    private final RobotStateHistory robotHistory = new RobotStateHistory();

    private WebcamSession session;
    private RobotStateSource robotStateSource;
    private BallDetectionPipeline.Frame frame = BallDetectionPipeline.Frame.EMPTY;
    private List<FieldBall> fieldBalls = Collections.emptyList();
    private RobotStateHistory.Sample captureState;

    public Camera(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_WEBCAM_NAME);
    }

    public Camera(HardwareMap hardwareMap, String webcamName) {
        super();
        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    /** Enables the field-relative queries by giving the module the robot's pose and velocity. */
    public Camera withFollower(Follower follower) {
        return withRobotState(RobotStateSource.fromFollower(follower));
    }

    public Camera withRobotState(RobotStateSource source) {
        this.robotStateSource = source;
        robotHistory.clear();
        return this;
    }

    @Override
    protected void initStates() {
        setStates(VisionState.ENABLED);
    }

    @Override
    public void init() {
        // Opened here, not in the constructor: the camera reports open failures through telemetry,
        // which the framework only hands to a Module after initStates().
        session = new WebcamSession(hardwareMap, getTelemetry(), webcamName, pipeline);
    }

    @Override
    protected void read() {
        frame = pipeline.latest();
        // Pumped from read(), not write(): exposure/gain are tuned against the init-loop preview,
        // and the framework holds writes back until start().
        session.update();
        updateFieldBalls();
    }

    private void updateFieldBalls() {
        if (robotStateSource == null) {
            fieldBalls = Collections.emptyList();
            captureState = null;
            return;
        }
        robotHistory.record(robotStateSource.sample(nowSeconds()));
        // The frame is already tens of milliseconds old, so transform it with where the robot was
        // when the shutter fired, not where it is now.
        captureState = robotHistory.sampleAt(frame.timestampSeconds);
        fieldBalls = BallFieldTransform.toField(frame.balls, captureState);
    }

    @Override
    protected void write() {
        pipeline.setDetectionEnabled(isInAny(VisionState.ENABLED));
    }

    @Override
    public void stop() {
        if (session != null) session.close();
    }

    // =========================================================================
    // Camera-relative queries. Positions and velocities are in the camera's own
    // frame, so a stationary ball appears to move whenever the robot does — use
    // the field-relative queries below for anything that outlives a single loop.
    // All read the snapshot taken in read(), so they are stable for the whole
    // loop and cheap to call repeatedly.
    // =========================================================================

    /** Balls being tracked across frames, with velocity. Empty until the first detection lands. */
    public List<TrackedBall> getBalls() {
        return frame.balls;
    }

    /** This frame's raw detections, without identity or velocity. */
    public List<BallDetection> getDetections() {
        return frame.detections;
    }

    public int getBallCount() {
        return frame.balls.size();
    }

    public boolean hasBall() {
        return !frame.balls.isEmpty();
    }

    public TrackedBall getBallById(int id) {
        for (TrackedBall ball : frame.balls) {
            if (ball.id == id) return ball;
        }
        return null;
    }

    /** Ball closest to the camera frame's origin, or null if none are tracked. */
    public TrackedBall getNearestBall() {
        return getNearestBallTo(0, 0);
    }

    public TrackedBall getNearestBallTo(double fieldX, double fieldY) {
        TrackedBall nearest = null;
        double bestDistance = Double.MAX_VALUE;
        for (TrackedBall ball : frame.balls) {
            double distance = ball.distanceTo(fieldX, fieldY);
            if (distance < bestDistance) {
                bestDistance = distance;
                nearest = ball;
            }
        }
        return nearest;
    }

    /** Fastest-moving ball, or null if nothing is tracked. Never filters on {@code isMoving()}. */
    public TrackedBall getFastestBall() {
        TrackedBall fastest = null;
        for (TrackedBall ball : frame.balls) {
            if (fastest == null || ball.speed() > fastest.speed()) fastest = ball;
        }
        return fastest;
    }

    public List<TrackedBall> getMovingBalls() {
        List<TrackedBall> moving = new ArrayList<>(frame.balls.size());
        for (TrackedBall ball : frame.balls) {
            if (ball.isMoving()) moving.add(ball);
        }
        return Collections.unmodifiableList(moving);
    }

    public boolean isAnyBallMoving() {
        for (TrackedBall ball : frame.balls) {
            if (ball.isMoving()) return true;
        }
        return false;
    }

    /**
     * Where {@code ball} will be {@code defaultLookaheadSeconds} from the frame it was seen in,
     * including the time that frame has already spent waiting to be consumed.
     */
    public Point getPredictedBallPosition(TrackedBall ball) {
        return getPredictedBallPosition(ball, defaultLookaheadSeconds);
    }

    public Point getPredictedBallPosition(TrackedBall ball, double lookaheadSeconds) {
        return ball.predict(lookaheadSeconds + getFrameAgeSeconds());
    }

    // =========================================================================
    // Field-relative queries. Empty unless a robot state source was supplied via
    // withFollower() / withRobotState(); velocities have the robot's own motion
    // removed, so they are true ground speeds.
    // =========================================================================

    public boolean hasRobotState() {
        return robotStateSource != null;
    }

    public List<FieldBall> getFieldBalls() {
        return fieldBalls;
    }

    public int getFieldBallCount() {
        return fieldBalls.size();
    }

    public FieldBall getFieldBallById(int id) {
        for (FieldBall ball : fieldBalls) {
            if (ball.id == id) return ball;
        }
        return null;
    }

    /** Ball closest to the robot, or null if none are tracked. */
    public FieldBall getNearestFieldBall() {
        RobotStateHistory.Sample robot = robotHistory.newest();
        return robot == null ? null : getNearestFieldBallTo(robot.x, robot.y);
    }

    public FieldBall getNearestFieldBallTo(Pose pose) {
        return getNearestFieldBallTo(pose.x(), pose.y());
    }

    public FieldBall getNearestFieldBallTo(double fieldX, double fieldY) {
        FieldBall nearest = null;
        double bestDistance = Double.MAX_VALUE;
        for (FieldBall ball : fieldBalls) {
            double distance = ball.distanceTo(fieldX, fieldY);
            if (distance < bestDistance) {
                bestDistance = distance;
                nearest = ball;
            }
        }
        return nearest;
    }

    /** Fastest ball over the ground, or null if nothing is tracked. */
    public FieldBall getFastestFieldBall() {
        FieldBall fastest = null;
        for (FieldBall ball : fieldBalls) {
            if (fastest == null || ball.speed() > fastest.speed()) fastest = ball;
        }
        return fastest;
    }

    public List<FieldBall> getMovingFieldBalls() {
        List<FieldBall> moving = new ArrayList<>(fieldBalls.size());
        for (FieldBall ball : fieldBalls) {
            if (ball.isMoving()) moving.add(ball);
        }
        return Collections.unmodifiableList(moving);
    }

    /**
     * Where {@code ball} will be {@code defaultLookaheadSeconds} from now, including the time its
     * frame has already spent waiting to be consumed.
     */
    public Pose getPredictedFieldPosition(FieldBall ball) {
        return getPredictedFieldPosition(ball, defaultLookaheadSeconds);
    }

    public Pose getPredictedFieldPosition(FieldBall ball, double lookaheadSeconds) {
        return ball.predict(lookaheadSeconds + getFrameAgeSeconds());
    }

    /** Field position of an arbitrary camera-frame point, or null without a robot state source. */
    public Pose cameraPointToField(double cameraX, double cameraY) {
        return captureState == null
                ? null
                : BallFieldTransform.cameraPointToField(cameraX, cameraY, captureState);
    }

    /** Robot state at the instant the frame being read was captured; null without a state source. */
    public RobotStateHistory.Sample getCaptureState() {
        return captureState;
    }

    // =========================================================================
    // Camera / pipeline status
    // =========================================================================

    public BallDetectionPipeline getPipeline() {
        return pipeline;
    }

    public double getFps() {
        return frame.fps;
    }

    /** Seconds since the camera thread produced the snapshot currently being read. */
    public double getFrameAgeSeconds() {
        if (frame.timestampSeconds == 0) return 0;
        return nowSeconds() - frame.timestampSeconds;
    }

    private static double nowSeconds() {
        return System.nanoTime() * 1e-9;
    }

    public boolean isFrameStale() {
        return frame.timestampSeconds == 0 || getFrameAgeSeconds() > staleFrameSeconds;
    }

    public boolean isCalibrated() {
        return pipeline.isCalibrated();
    }

    /** Forget every tracked ball — use after the robot moves, which invalidates every velocity. */
    public void resetTracking() {
        pipeline.resetTracking();
    }

    @Override
    protected void onTelemetry() {
        if (cameraTelemetry) {
            logDashboard("Vision", getState(VisionState.class));
            logDashboard("FPS", "%.1f", frame.fps);
            logDashboard("Balls", frame.balls.size());
            log("ROIs searched", frame.roiCount);
            log("Rejected (color)", frame.rejectedColor);
            log("Rejected (overlap)", frame.rejectedOverlap);
            if (isFrameStale()) log("Frame", "STALE (%.2fs)", getFrameAgeSeconds());
        }
        if (ballTelemetry) {
            if (hasRobotState()) {
                for (FieldBall ball : fieldBalls) {
                    logDashboard("Ball " + ball.id, "%s field (%.1f, %.1f)in  %.1fin/s @ %.0fdeg%s",
                            ball.type.label, ball.x, ball.y, ball.speed(), ball.headingDeg(),
                            ball.visible() ? "" : " [coasting]");
                }
            } else {
                for (TrackedBall ball : frame.balls) {
                    logDashboard("Ball " + ball.id, "%s cam (%.1f, %.1f)in  %.1fin/s @ %.0fdeg%s",
                            ball.type.label, ball.x, ball.y, ball.speed(), ball.headingDeg(),
                            ball.visible ? "" : " [coasting]");
                }
            }
        }
    }
}
