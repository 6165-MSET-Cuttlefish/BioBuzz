package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.Ball;
import org.firstinspires.ftc.teamcode.architecture.auto.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.auto.Region;
import org.firstinspires.ftc.teamcode.architecture.auto.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.auto.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.auto.RouteRun;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.vision.BallType;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Autonomous ball collection as one Ivy command, built by {@link RobotActions#collectBalls}. When it starts it
 * waits for a Limelight frame captured after that moment, plans a drive that runs the intake over the nearest balls
 * in view on this alliance's half ({@link BioBuzzField#ownHalf}) within range, with the robot's footprint
 * ({@link RobotGeometry}) clear of the walls and {@link BioBuzzField#hiveRails}, drives it with {@link RouteRun} and
 * returns to where it planned from. It requires the follower and the Drivetrain, whose writes it disables while Pedro
 * drives and restores afterwards. Read {@link #status()} and {@link #detail()} for how it went. With the Limelight
 * faulted when it starts, it ends SKIPPED; a route already driving finishes, since its plan needs no live vision.
 */
public final class BallCollection extends CommandBuilder {

    /** Read when the command starts, so dashboard edits apply to the next run. */
    public static final class Settings {
        public boolean collectPollen = true;
        public boolean collectRedNectar = true;
        public boolean collectBlueNectar = true;
        /** The nearest this many usable balls are planned; the order search is n!, so at most {@link RouteOptimizer#MAX_BALLS}. */
        public int maxBalls = 4;
        public double maxRangeIn = 60;
        /** Least room between the robot's footprint and the walls or the centre line. */
        public double wallGapIn = 1;
        /** Least room between the robot's footprint and a HIVE rail. */
        public double railGapIn = 1;
        /** With no frame captured after the start by then, the command ends SKIPPED without driving. */
        public double visionWaitMs = 1000;
        public double timeoutMinAvgSpeedIps = 10;
        public double timeoutMinSec = 4;
        public double timeoutMaxSec = 15;

        Settings copy() {
            Settings s = new Settings();
            s.collectPollen = collectPollen;
            s.collectRedNectar = collectRedNectar;
            s.collectBlueNectar = collectBlueNectar;
            s.maxBalls = maxBalls;
            s.maxRangeIn = maxRangeIn;
            s.wallGapIn = wallGapIn;
            s.railGapIn = railGapIn;
            s.visionWaitMs = visionWaitMs;
            s.timeoutMinAvgSpeedIps = timeoutMinAvgSpeedIps;
            s.timeoutMinSec = timeoutMinSec;
            s.timeoutMaxSec = timeoutMaxSec;
            return s;
        }

        void validate() {
            if (maxBalls < 0 || maxBalls > RouteOptimizer.MAX_BALLS) {
                throw new IllegalArgumentException(String.format(
                        "maxBalls %d: must be 0 to %d", maxBalls, RouteOptimizer.MAX_BALLS));
            }
            if (!(wallGapIn >= 0) || !(railGapIn >= 0)) {
                throw new IllegalArgumentException(String.format(
                        "wallGapIn (%.1f) and railGapIn (%.1f) must be >= 0", wallGapIn, railGapIn));
            }
            if (!(maxRangeIn > 0) || !(visionWaitMs > 0)) {
                throw new IllegalArgumentException(String.format(
                        "maxRangeIn (%.1f) and visionWaitMs (%.0f) must be > 0", maxRangeIn, visionWaitMs));
            }
        }

        boolean wants(BallType type) {
            switch (type) {
                case POLLEN: return collectPollen;
                case NECTAR_RED: return collectRedNectar;
                case NECTAR_BLUE: return collectBlueNectar;
                default: throw new IllegalArgumentException("unhandled ball type " + type);
            }
        }
    }

    public enum Status { NOT_STARTED, WAITING_FOR_VISION, DRIVING, DONE, ABORTED, SKIPPED }

    /** A ball seen but not planned, with why. */
    public static final class Skipped {
        public final FieldBall ball;
        public final String reason;

        Skipped(FieldBall ball, String reason) {
            this.ball = ball;
            this.reason = reason;
        }
    }

    private final Follower follower;
    private final Drivetrain drivetrain;
    private final Camera camera;
    private final Settings settings;

    private Settings used;
    private Status status = Status.NOT_STARTED;
    private String detail = "";
    private double startedSeconds;
    private double visionFromSeconds;
    private boolean restoreWrites;
    private Region area;
    private List<Obstacle> obstacles = Collections.emptyList();
    private RobotShape robot;
    private Pose planStart;
    private final List<FieldBall> planned = new ArrayList<>();
    private final List<Skipped> skipped = new ArrayList<>();
    private Ball[] balls = new Ball[0];
    private RouteOptimizer.Route route;
    private RoutePathBuilder.Plan plan;
    private RouteRun run;
    private Command runCommand;

    /** {@link RobotActions#collectBalls} builds it for a BioBuzzRobot. */
    public BallCollection(Follower follower, Drivetrain drivetrain, Camera camera, Settings settings) {
        if (!camera.hasRobotState()) {
            throw new IllegalArgumentException("ball collection needs field balls: build the Camera withFollower(...)");
        }
        this.follower = follower;
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.settings = settings;
        setStart(this::begin);
        setExecute(this::step);
        setDone(this::isDone);
        setEnd(this::finish);
        requiring(follower, drivetrain);
    }

    private void begin() {
        used = settings.copy();
        used.validate();
        status = Status.WAITING_FOR_VISION;
        detail = "";
        startedSeconds = nowSeconds();
        visionFromSeconds = Double.NaN;
        area = BioBuzzField.ownHalf(used.wallGapIn);
        obstacles = BioBuzzField.hiveRails();
        robot = RobotGeometry.shape();
        planStart = null;
        planned.clear();
        skipped.clear();
        balls = new Ball[0];
        route = null;
        plan = null;
        run = null;
        runCommand = null;
        // Drivetrain.write() throws while Pedro drives the same motors.
        restoreWrites = drivetrain.isWriteEnabled();
        drivetrain.setWriteEnabled(false);
        String limelight = camera.limelightProblem();
        if (limelight != null) {
            status = Status.SKIPPED;
            detail = "Limelight: " + limelight;
        }
    }

    private void step() {
        if (status == Status.WAITING_FOR_VISION) {
            // A pose set in onStart() is in Camera's pose history only from this loop on; earlier frames may be placed with the old pose.
            if (Double.isNaN(visionFromSeconds)) visionFromSeconds = nowSeconds();
            if (freshFrame()) {
                planAndDrive();
            } else if ((nowSeconds() - startedSeconds) * 1000 > used.visionWaitMs) {
                String limelight = camera.limelightProblem();
                status = Status.SKIPPED;
                detail = String.format("no Limelight frame captured after the start within %.0f ms%s",
                        used.visionWaitMs, limelight != null ? " (Limelight: " + limelight + ")"
                                : camera.isSyncing() ? " (the ball pipeline was still going onto the Limelight; START later)"
                                : camera.isFrameStale() ? " (vision is stale)" : "");
            }
            return;
        }
        if (runCommand != null && !runCommand.done()) runCommand.execute();
    }

    private boolean freshFrame() {
        if (camera.isFrameStale()) return false;
        double captureSeconds = nowSeconds() - camera.getFrameAgeSeconds();
        return captureSeconds >= visionFromSeconds;
    }

    private void planAndDrive() {
        Pose start = follower.pose();
        String problem = RouteOptimizer.poseProblem(start, robot, area, obstacles, used.railGapIn);
        if (problem != null) {
            status = Status.SKIPPED;
            detail = "the start pose " + problem;
            return;
        }
        planStart = start;
        select(start);
        balls = new Ball[planned.size()];
        for (int i = 0; i < balls.length; i++) {
            FieldBall b = planned.get(i);
            balls[i] = new Ball(b.x, b.y, b.type.diameterIn / 2);
        }
        route = RouteOptimizer.findOptimalRoute(start, balls, start, area, obstacles, robot, used.railGapIn, false);
        plan = RoutePathBuilder.build(route);
        run = new RouteRun(follower, plan, used.timeoutMinAvgSpeedIps, used.timeoutMinSec, used.timeoutMaxSec);
        runCommand = run.command();
        status = Status.DRIVING;
        runCommand.start();
    }

    private void select(final Pose from) {
        List<FieldBall> usable = new ArrayList<>();
        for (FieldBall ball : camera.getFieldBalls()) {
            if (!ball.visible()) {
                skipped.add(new Skipped(ball, "not in view"));
            } else if (!used.wants(ball.type)) {
                skipped.add(new Skipped(ball, ball.type.label + " not collected"));
            } else if (!area.contains(ball.x, ball.y)) {
                skipped.add(new Skipped(ball, "off our half " + area));
            } else if (ball.distanceTo(from.x(), from.y()) > used.maxRangeIn) {
                skipped.add(new Skipped(ball, String.format("beyond %.0f in", used.maxRangeIn)));
            } else {
                // The planner would drop it anyway, but only after it had taken one of the nearest maxBalls places.
                String unreachable = RouteOptimizer.ballProblem(ball.x, ball.y, ball.type.diameterIn / 2, robot, area,
                        obstacles, used.railGapIn);
                if (unreachable == null) usable.add(ball);
                else skipped.add(new Skipped(ball, unreachable));
            }
        }
        Collections.sort(usable, new Comparator<FieldBall>() {
            @Override public int compare(FieldBall a, FieldBall b) {
                return Double.compare(a.distanceTo(from.x(), from.y()), b.distanceTo(from.x(), from.y()));
            }
        });
        for (int i = 0; i < usable.size(); i++) {
            if (i < used.maxBalls) planned.add(usable.get(i));
            else skipped.add(new Skipped(usable.get(i), "past the nearest " + used.maxBalls));
        }
    }

    private boolean isDone() {
        switch (status) {
            case DONE:
            case ABORTED:
            case SKIPPED:
                return true;
            case DRIVING:
                return runCommand.done();
            default:
                return false;
        }
    }

    private void finish(EndCondition end) {
        if (status == Status.DRIVING) {
            runCommand.end(end);
            if (run.finished()) {
                status = Status.DONE;
            } else {
                status = Status.ABORTED;
                detail = run.abortReason();
            }
        } else if (status == Status.WAITING_FOR_VISION) {
            status = Status.ABORTED;
            detail = "interrupted while waiting for vision";
        }
        if (restoreWrites) {
            // Pedro drove these motors through its own objects, so resync Drivetrain's write cache at zero.
            drivetrain.setTargets(0, 0, 0, 0);
            drivetrain.stop();
            drivetrain.setWriteEnabled(true);
        }
    }

    private static double nowSeconds() {
        return System.nanoTime() * 1e-9;
    }

    public Status status() {
        return status;
    }

    /** Why it was SKIPPED or ABORTED; empty otherwise. */
    public String detail() {
        return detail;
    }

    /** The current leg while DRIVING. */
    public String phase() {
        return run == null ? "" : run.phase();
    }

    /** The HIVE rails the last plan avoided; empty before the first start. */
    public List<Obstacle> obstacles() {
        return obstacles;
    }

    /** Where the robot's footprint must stay; null before the command starts. */
    public Region area() {
        return area;
    }

    /** The robot the last plan was made for; null before the command starts. */
    public RobotShape robot() {
        return robot;
    }

    /** Where the plan starts and returns to; null until planned. */
    public Pose planStart() {
        return planStart;
    }

    /** Null until planned. */
    public RouteOptimizer.Route route() {
        return route;
    }

    /** Null until planned. */
    public RoutePathBuilder.Plan plan() {
        return plan;
    }

    /** The balls handed to the planner, in the same order as {@link #plannedBalls()}. */
    public Ball[] balls() {
        return balls;
    }

    /** The vision ball each of {@link #balls()} came from. */
    public List<FieldBall> plannedBalls() {
        return Collections.unmodifiableList(planned);
    }

    public List<Skipped> skipped() {
        return Collections.unmodifiableList(skipped);
    }

    /** The vision ball behind one of {@link #balls()}, such as one in the route's order or dropped list. */
    public FieldBall sourceOf(Ball ball) {
        for (int i = 0; i < balls.length; i++) {
            if (balls[i] == ball) return planned.get(i);
        }
        throw new IllegalArgumentException(ball + " is not one of this plan's balls");
    }
}
