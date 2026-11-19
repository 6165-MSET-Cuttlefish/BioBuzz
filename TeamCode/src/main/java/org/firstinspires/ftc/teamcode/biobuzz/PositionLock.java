package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;

/**
 * Holds the robot on one pose with Pedro's hold, which drives it back there whenever it is pushed off, until cancelled.
 * It ends on its own only if the robot's pose stops being finite or something else takes the follower.
 */
public final class PositionLock extends CommandBuilder {
    private final Follower follower;
    private final Drivetrain drivetrain;
    private final Pose pose;

    private boolean started;
    private boolean holding;
    private boolean restoreWrites;
    private boolean released;
    private String stopReason;

    /** {@link RobotActions#lockPosition} builds it for a BioBuzzRobot; build a new one for each lock. */
    public PositionLock(Follower follower, Drivetrain drivetrain, Pose pose) {
        this.follower = follower;
        this.drivetrain = drivetrain;
        this.pose = pose;
        setStart(this::begin);
        setDone(this::lost);
        setEnd(this::finish);
        requiring(follower, drivetrain);
    }

    private void begin() {
        if (started) throw new IllegalStateException("a PositionLock runs once; build a new one to lock again");
        started = true;
        if (!finite(pose)) {
            stopReason = String.format("(%.1f, %.1f) facing %.0f deg isn't finite", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            return;
        }
        restoreWrites = drivetrain.handToFollower();
        follower.hold(pose);
        holding = true;
    }

    // Pedro keeps each motor's last power when its powers go NaN, so a pose gone non-finite would drive the robot off.
    private boolean lost() {
        if (stopReason != null) return true;
        if (!finite(follower.pose())) {
            stopReason = "the robot's pose isn't finite";
        } else if (!follower.holding()) {
            stopReason = "something else took the follower";
        }
        return stopReason != null;
    }

    private void finish(EndCondition end) {
        if (stopReason == null && !released) stopReason = "interrupted";
        if (!holding) return;
        holding = false;
        follower.stop();
        drivetrain.takeBackFromFollower(restoreWrites);
    }

    /** Ends it, leaving {@link #stopReason()} null: the caller knows why. */
    public void release() {
        if (!Scheduler.isScheduled(this)) return;
        released = true;
        Scheduler.cancel(this);
    }

    public Pose pose() {
        return pose;
    }

    /** Why it stopped without {@link #release()}: null while it holds or after a release. */
    public String stopReason() {
        return stopReason;
    }

    private static boolean finite(Pose p) {
        return Double.isFinite(p.x()) && Double.isFinite(p.y()) && Double.isFinite(p.heading());
    }
}
