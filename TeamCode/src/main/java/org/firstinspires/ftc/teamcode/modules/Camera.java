package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetectionPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.BallFieldTransform;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBallTracker;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateHistory;
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

    public static final String WEBCAM_NAME = "nerdDetector";

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final HardwareMap hardwareMap;
    private final BallDetectionPipeline pipeline = new BallDetectionPipeline();
    private final RobotStateHistory robotHistory = new RobotStateHistory();
    private final FieldBallTracker fieldBallTracker = new FieldBallTracker();

    private WebcamSession session;
    private Follower follower;
    private BallDetectionPipeline.Frame frame = BallDetectionPipeline.Frame.EMPTY;
    private List<FieldBall> fieldBalls = Collections.emptyList();
    private double lastFieldBallFrameTimestamp = -1;
    private double previousReadSeconds = Double.NaN;

    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Camera withFollower(Follower follower) {
        this.follower = follower;
        robotHistory.clear();
        previousReadSeconds = Double.NaN;
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
        session = new WebcamSession(hardwareMap, getTelemetry(), WEBCAM_NAME, pipeline);
    }

    @Override
    protected void read() {
        frame = pipeline.latest();
        // In read(), not write(): exposure/gain tuning must apply during init, when writes are held back.
        session.update();
        updateFieldBalls();
    }

    private void updateFieldBalls() {
        if (follower == null) {
            fieldBalls = Collections.emptyList();
            return;
        }
        // read() runs before follower.update(): this pose is the previous loop's, so it gets that loop's time.
        double now = nowSeconds();
        if (!Double.isNaN(previousReadSeconds)) {
            Pose pose = follower.pose();
            Velocity velocity = follower.velocity();
            robotHistory.record(new RobotStateHistory.Sample(
                    pose.x(), pose.y(), pose.heading(),
                    velocity.vx, velocity.vy, velocity.omega, previousReadSeconds));
        }
        previousReadSeconds = now;
        if (frame.timestampSeconds == lastFieldBallFrameTimestamp) return;
        lastFieldBallFrameTimestamp = frame.timestampSeconds;

        // The frame is tens of ms old: transform with the robot state at capture time, not now.
        RobotStateHistory.Sample captureState = robotHistory.sampleAt(frame.timestampSeconds);
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

    public int getBallCount() {
        return frame.balls.size();
    }

    /** Nearest to the camera frame's origin, not the robot center. */
    public TrackedBall getNearestBall() {
        TrackedBall nearest = null;
        double bestDistance = Double.MAX_VALUE;
        for (TrackedBall ball : frame.balls) {
            double distance = ball.distanceTo(0, 0);
            if (distance < bestDistance) {
                bestDistance = distance;
                nearest = ball;
            }
        }
        return nearest;
    }

    public Point getPredictedBallPosition(TrackedBall ball) {
        return getPredictedBallPosition(ball, defaultLookaheadSeconds);
    }

    public Point getPredictedBallPosition(TrackedBall ball, double lookaheadSeconds) {
        return ball.predict(lookaheadSeconds + getFrameAgeSeconds());
    }

    public boolean hasRobotState() {
        return follower != null;
    }

    public List<FieldBall> getFieldBalls() {
        return fieldBalls;
    }

    public FieldBall getNearestFieldBall() {
        RobotStateHistory.Sample robot = robotHistory.newest();
        if (robot == null) return null;
        FieldBall nearest = null;
        double bestDistance = Double.MAX_VALUE;
        for (FieldBall ball : fieldBalls) {
            double distance = ball.distanceTo(robot.x, robot.y);
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
                            ball.visible() ? "" : " [last seen]");
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
