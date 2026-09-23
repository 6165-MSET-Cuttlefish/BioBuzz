package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetection;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetectionPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.BallFieldTransform;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBallTracker;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateHistory;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateSource;
import org.firstinspires.ftc.teamcode.modules.vision.TrackedBall;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * {@link #getBalls()} is camera-relative (robot motion included); {@link #getFieldBalls()} is
 * field-relative and stays empty until {@link #withFollower} is called.
 */
@Config
public class Camera extends Module {

    public static boolean cameraTelemetry = true;
    public static boolean ballTelemetry = true;

    public static double defaultLookaheadSeconds = 0.25;
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
    private final FieldBallTracker fieldBallTracker = new FieldBallTracker();

    private WebcamSession session;
    private RobotStateSource robotStateSource;
    private BallDetectionPipeline.Frame frame = BallDetectionPipeline.Frame.EMPTY;
    private List<FieldBall> fieldBalls = Collections.emptyList();
    private RobotStateHistory.Sample captureState;
    private double lastFieldBallFrameTimestamp = -1;

    public Camera(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_WEBCAM_NAME);
    }

    public Camera(HardwareMap hardwareMap, String webcamName) {
        super();
        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    public Camera withFollower(Follower follower) {
        return withRobotState(RobotStateSource.fromFollower(follower));
    }

    public Camera withRobotState(RobotStateSource source) {
        this.robotStateSource = source;
        robotHistory.clear();
        fieldBallTracker.reset();
        return this;
    }

    @Override
    protected void initStates() {
        setStates(VisionState.ENABLED);
    }

    @Override
    public void init() {
        // Not in the constructor: open failures report through telemetry, which a Module only gets at init.
        session = new WebcamSession(hardwareMap, getTelemetry(), webcamName, pipeline);
    }

    @Override
    protected void read() {
        frame = pipeline.latest();
        // In read(), not write(): exposure/gain tuning must apply during init, when writes are held back.
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
        if (frame.timestampSeconds == lastFieldBallFrameTimestamp) return;
        lastFieldBallFrameTimestamp = frame.timestampSeconds;

        // The frame is tens of ms old: transform with the robot state at capture time, not now.
        captureState = robotHistory.sampleAt(frame.timestampSeconds);
        List<FieldBall> freshFieldBalls = BallFieldTransform.toField(frame.balls, captureState);
        fieldBalls = fieldBallTracker.update(freshFieldBalls, nowSeconds());
    }

    @Override
    protected void write() {
        pipeline.setDetectionEnabled(isInAny(VisionState.ENABLED));
    }

    @Override
    public void stop() {
        if (session != null) session.close();
    }

    public List<TrackedBall> getBalls() {
        return frame.balls;
    }

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

    /** Nearest to the camera frame's origin, not the robot center. */
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

    public Point getPredictedBallPosition(TrackedBall ball) {
        return getPredictedBallPosition(ball, defaultLookaheadSeconds);
    }

    public Point getPredictedBallPosition(TrackedBall ball, double lookaheadSeconds) {
        return ball.predict(lookaheadSeconds + getFrameAgeSeconds());
    }

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

    public Pose getPredictedFieldPosition(FieldBall ball) {
        return getPredictedFieldPosition(ball, defaultLookaheadSeconds);
    }

    public Pose getPredictedFieldPosition(FieldBall ball, double lookaheadSeconds) {
        return ball.predict(lookaheadSeconds + getFrameAgeSeconds());
    }

    /** Null without a robot state source. */
    public Pose cameraPointToField(double cameraX, double cameraY) {
        return captureState == null
                ? null
                : BallFieldTransform.cameraPointToField(cameraX, cameraY, captureState);
    }

    /** Robot state when the current frame was captured; null without a robot state source. */
    public RobotStateHistory.Sample getCaptureState() {
        return captureState;
    }

    public BallDetectionPipeline getPipeline() {
        return pipeline;
    }

    public double getFps() {
        return frame.fps;
    }

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
