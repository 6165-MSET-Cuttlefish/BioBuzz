package org.firstinspires.ftc.teamcode.biobuzz.modules;

import static org.firstinspires.ftc.teamcode.biobuzz.modules.vision.RobotStateHistory.nowSeconds;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.BallFieldTransform;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.FieldBallTracker;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.RobotStateHistory;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.TrackedBall;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config("Camera")
public class Camera extends Module {

    public static boolean cameraTelemetry = true;
    public static boolean ballTelemetry = true;

    public static double defaultLookaheadSeconds = 0.25;

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final LimelightBallSource source;
    private final RobotStateHistory robotHistory = new RobotStateHistory();
    private final FieldBallTracker fieldBallTracker = new FieldBallTracker();

    private final Follower follower;
    private LimelightBallSource.Frame frame = LimelightBallSource.Frame.EMPTY;
    private List<FieldBall> fieldBalls = Collections.emptyList();
    private double previousReadSeconds = Double.NaN;

    public Camera(HardwareMap hardwareMap, Follower follower) {
        source = new LimelightBallSource(hardwareMap);
        this.follower = follower;
    }

    @Override
    protected void initStates() {
        setStates(VisionState.ENABLED);
    }

    @Override
    protected void read() {
        boolean newFrame = false;
        if (isInAny(VisionState.ENABLED)) {
            newFrame = source.update();
        } else {
            source.idle();
        }
        frame = source.latest();
        updateFieldBalls(newFrame);
    }

    private void updateFieldBalls(boolean newFrame) {
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
        if (newFrame) {
            // The frame is tens of ms old: transform with the robot state at capture time, not now.
            RobotStateHistory.Sample captureState = robotHistory.sampleAt(frame.timestampSeconds);
            fieldBalls = fieldBallTracker.update(BallFieldTransform.toField(frame.balls, captureState), now);
        } else if (frame.stale) {
            fieldBalls = fieldBallTracker.update(Collections.<FieldBall>emptyList(), now);
        }
    }

    @Override
    protected void write() {}

    @Override
    public void stop() {
        source.stop();
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
        return ball.predict(lookaheadSeconds + predictionAgeSeconds());
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
        return ball.predict(lookaheadSeconds + predictionAgeSeconds());
    }

    /** Infinite before the first frame and after vision is disabled. */
    public double getFrameAgeSeconds() {
        return frame.ageSeconds();
    }

    private double predictionAgeSeconds() {
        double age = getFrameAgeSeconds();
        if (Double.isInfinite(age)) {
            throw new IllegalStateException("Camera has no Limelight frame to predict from (none yet, or vision disabled)");
        }
        return age;
    }

    /** No live Limelight frame: none yet, vision disabled, a Limelight fault, or none new for {@code Limelight Balls → staleFrameSeconds}. */
    public boolean isFrameStale() {
        return frame.stale;
    }

    /** True while the ball pipeline is still going onto the Limelight after INIT: the frame is stale with no fault. */
    public boolean isSyncing() {
        return source.isSyncing();
    }

    /** Why the Limelight is unusable (also shown as the {@code Limelight} fault), or null while healthy or DISABLED. */
    public String limelightProblem() {
        return source.problem();
    }

    @Override
    protected void onTelemetry() {
        if (cameraTelemetry) {
            logDashboard("Vision", getState(VisionState.class));
            logDashboard("FPS", "%.1f", frame.fps);
            logDashboard("Balls", frame.balls.size());
            log("Detections", frame.detectionCount);
            log("Latency (ms)", "%.0f", frame.latencyMs);
            if (limelightProblem() != null) {
                log("Frame", "STALE (Limelight fault)");
            } else if (isSyncing()) {
                log("Frame", "STALE (putting the ball pipeline on the Limelight)");
            } else if (isFrameStale()) {
                double age = getFrameAgeSeconds();
                log("Frame", Double.isInfinite(age) ? "STALE (no frame)" : String.format("STALE (%.2fs)", age));
            }
        }
        if (ballTelemetry) {
            for (FieldBall ball : fieldBalls) {
                logDashboard("Ball " + ball.id, "%s field (%.1f, %.1f)in  %.1fin/s @ %.0fdeg%s",
                        ball.type.label, ball.x, ball.y, ball.speed(), ball.headingDeg(),
                        ball.visible() ? "" : " [last seen]");
            }
        }
    }
}
