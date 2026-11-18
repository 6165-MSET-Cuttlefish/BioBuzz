package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.planner.Ball;
import org.firstinspires.ftc.teamcode.architecture.planner.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.planner.Region;
import org.firstinspires.ftc.teamcode.architecture.planner.RobotShape;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.planner.RoutePathBuilder;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteRun;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;

import java.util.List;

/** A route search with no balls: {@link RoutePathBuilder}'s return leg from where the robot is at the start to {@code target}. */
public final class WaypointDrive extends CommandBuilder {

    /** Read when the drive starts, so dashboard edits apply to the next one. */
    public static final class Settings {
        /** Least room between the robot's footprint and the walls or the centre line. */
        public double wallGapIn = 1;
        /** Least room between the robot's footprint and a HIVE rail. */
        public double railGapIn = 1;
        /** Off for testing away from a field: no walls, centre line or HIVE rails. */
        public boolean onField = true;
        public double timeoutMinAvgSpeedIps = 10;
        public double timeoutMinSec = 3;
        public double timeoutMaxSec = 10;

        Settings copy() {
            Settings s = new Settings();
            s.wallGapIn = wallGapIn;
            s.railGapIn = railGapIn;
            s.onField = onField;
            s.timeoutMinAvgSpeedIps = timeoutMinAvgSpeedIps;
            s.timeoutMinSec = timeoutMinSec;
            s.timeoutMaxSec = timeoutMaxSec;
            return s;
        }

        void validate() {
            if (!(wallGapIn >= 0) || !(railGapIn >= 0)) {
                throw new IllegalArgumentException(String.format(
                        "wallGapIn (%.1f) and railGapIn (%.1f) must be >= 0", wallGapIn, railGapIn));
            }
        }
    }

    public enum Status { NOT_STARTED, PLANNING, DRIVING, DONE, ABORTED, REFUSED }

    /** Null if the robot may stand at {@code pose} for a drive under {@code settings}, otherwise why it may not. */
    public static String poseProblem(Pose pose, Settings settings) {
        return new Layout(settings).problem(pose);
    }

    /** Both poses must pass {@link #poseProblem} first. */
    static RouteOptimizer.Search search(Pose start, Pose target, Settings settings) {
        return new Layout(settings).search(start, target);
    }

    private static final class Layout {
        private static final Ball[] NO_BALLS = new Ball[0];

        final Region area;
        final List<Obstacle> obstacles;
        final RobotShape robot = RobotGeometry.shape();
        final double gapIn;

        Layout(Settings settings) {
            area = BallCollection.keepIn(settings.onField, settings.wallGapIn);
            obstacles = BallCollection.obstacles(settings.onField);
            gapIn = settings.railGapIn;
        }

        String problem(Pose pose) {
            if (!(Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading()))) {
                return String.format("(%.1f, %.1f) facing %.0f deg isn't finite",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            }
            return RouteOptimizer.poseProblem(pose, robot, area, obstacles, gapIn);
        }

        RouteOptimizer.Search search(Pose start, Pose target) {
            return RouteOptimizer.search(start, NO_BALLS, target, area, obstacles, robot, gapIn, false);
        }
    }

    private final Follower follower;
    private final Drivetrain drivetrain;
    private final Pose target;
    private final Settings settings;

    private Settings used;
    private Status status = Status.NOT_STARTED;
    private String detail = "";
    private String cancelReason;
    private boolean restoreWrites;
    private RouteOptimizer.Search search;
    private RoutePathBuilder.Plan plan;
    private RouteRun run;
    private Command runCommand;

    /** {@link RobotActions#driveTo} builds it for a BioBuzzRobot; build a new one for each drive. */
    public WaypointDrive(Follower follower, Drivetrain drivetrain, Pose target, Settings settings) {
        this.follower = follower;
        this.drivetrain = drivetrain;
        this.target = target;
        this.settings = settings;
        setStart(this::begin);
        setExecute(this::step);
        setDone(this::isDone);
        setEnd(this::finish);
        requiring(follower, drivetrain);
    }

    private void begin() {
        if (status != Status.NOT_STARTED) throw new IllegalStateException("a WaypointDrive runs once; build a new one to drive again");
        used = settings.copy();
        used.validate();
        status = Status.PLANNING;
        // Drivetrain.write() throws while Pedro drives the same motors.
        restoreWrites = drivetrain.isWriteEnabled();
        drivetrain.setWriteEnabled(false);
        Layout layout = new Layout(used);
        Pose start = follower.pose();
        String problem = layout.problem(start);
        if (problem != null) {
            refuse("start " + problem);
            return;
        }
        problem = layout.problem(target);
        if (problem != null) {
            refuse("waypoint " + problem);
            return;
        }
        search = layout.search(start, target);
    }

    private void refuse(String why) {
        status = Status.REFUSED;
        detail = why;
    }

    private void step() {
        if (status == Status.PLANNING) {
            if (search.step()) {
                if (search.noRoute() != null) refuse(search.noRoute());
                else drive();
            }
            return;
        }
        if (status == Status.DRIVING && !runCommand.done()) runCommand.execute();
    }

    private void drive() {
        plan = search.plan();
        run = new RouteRun(follower, plan, used.timeoutMinAvgSpeedIps, used.timeoutMinSec, used.timeoutMaxSec);
        runCommand = run.command();
        status = Status.DRIVING;
        runCommand.start();
    }

    private boolean isDone() {
        switch (status) {
            case DONE:
            case ABORTED:
            case REFUSED:
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
                detail = cancelReason != null ? cancelReason : run.abortReason();
            }
        } else if (status == Status.PLANNING) {
            status = Status.ABORTED;
            detail = cancelReason != null ? cancelReason : "interrupted while planning";
        }
        if (restoreWrites) {
            // Pedro drove these motors through its own objects, so resync Drivetrain's write cache at zero.
            drivetrain.setTargets(0, 0, 0, 0);
            drivetrain.stop();
            drivetrain.setWriteEnabled(true);
        }
    }

    /** Ends the drive at once from OpMode code, with {@code reason} as its detail; nothing if it isn't scheduled. */
    public void cancel(String reason) {
        if (!Scheduler.isScheduled(this)) return;
        cancelReason = reason;
        Scheduler.cancel(this);
    }

    public Status status() {
        return status;
    }

    /** Why it was REFUSED or ABORTED; empty otherwise. */
    public String detail() {
        return detail;
    }

    /** The current leg while DRIVING. */
    public String phase() {
        return run == null ? "" : run.phase();
    }

    public Pose target() {
        return target;
    }

    /** Null until planned. */
    public RoutePathBuilder.Plan plan() {
        return plan;
    }
}
