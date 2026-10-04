package org.firstinspires.ftc.teamcode.architecture.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;

import java.util.ArrayList;
import java.util.List;

/**
 * Drives a {@link RoutePathBuilder.Plan} step by step, then stops: each drive to a settled stop, each turn in place
 * until the heading settles. A drive gets {@code length / minAvgSpeedIps} seconds, clamped to [minLegSec,
 * maxLegSec], and a turn minLegSec; a step that runs out of time ends the whole route with the follower stopped,
 * since the next step was planned from where this one should have ended. {@link #abort} does the same from user
 * code. Run it once, either with {@link #start()} or as {@link #command()} inside a group.
 */
public final class RouteRun {
    // Pedro's own hold reports settled 100 ms in, before a long turn is done.
    private static final double TURN_SETTLED_RAD = Math.toRadians(2);
    private static final double TURN_STILL_RAD_PER_SEC = Math.toRadians(15);

    private final Follower follower;
    private final Command command;
    private String phase = "not started";
    private String abortReason;
    private boolean abortRequested;
    private long legDeadlineNs;
    private double legBudgetSec;
    private boolean started;
    private boolean running;
    private boolean finished;

    public RouteRun(Follower follower, RoutePathBuilder.Plan plan,
                    double minAvgSpeedIps, double minLegSec, double maxLegSec) {
        if (!(minAvgSpeedIps > 0) || !(minLegSec > 0) || !(maxLegSec >= minLegSec)) {
            throw new IllegalArgumentException(String.format(
                    "leg timeouts need minAvgSpeedIps > 0 and 0 < minLegSec <= maxLegSec, got %.1f, %.1f, %.1f",
                    minAvgSpeedIps, minLegSec, maxLegSec));
        }
        this.follower = follower;
        List<Command> steps = new ArrayList<>();
        int at = 0;
        for (RoutePathBuilder.Step step : plan.steps) {
            String name = String.format("step %d/%d, %s", ++at, plan.steps.size(),
                    step.intake && step.back ? "intake and return" : step.intake ? "intake" : "return");
            steps.add(step.path != null
                    ? leg(name, PathCommands.follow(follower, step.path),
                            budget(step.length, minAvgSpeedIps, minLegSec, maxLegSec))
                    : leg(name + " turn", turn(step.turnTo), minLegSec));
        }
        steps.add(PathCommands.stop(follower));
        steps.add(instant(() -> {
            phase = "done";
            finished = true;
        }));
        final Command route = race(sequential(steps.toArray(new Command[0])), waitUntil(this::shouldEnd));
        command = Command.build()
                .setStart(() -> {
                    if (started) throw new IllegalStateException("a RouteRun runs once; build a new one to drive again");
                    started = true;
                    running = true;
                    route.start();
                })
                .setExecute(route::execute)
                .setDone(route::done)
                .setEnd(end -> {
                    running = false;
                    // Ending the race interrupts the sequence, and an interrupted follow stops the follower.
                    route.end(end);
                    if (end == EndCondition.INTERRUPTED && abortReason == null) {
                        abortReason = "interrupted by another command or a timeout";
                    }
                })
                .requiring(route.requirements())
                .setPriority(route.priority());
    }

    private static double budget(double lengthIn, double minAvgSpeedIps, double minLegSec, double maxLegSec) {
        return Math.min(maxLegSec, Math.max(minLegSec, lengthIn / minAvgSpeedIps));
    }

    private Command leg(String name, Command action, double seconds) {
        return sequential(
                instant(() -> {
                    phase = name;
                    legBudgetSec = seconds;
                    legDeadlineNs = System.nanoTime() + (long) (seconds * 1e9);
                }),
                action,
                instant(() -> legDeadlineNs = 0));
    }

    private Command turn(Pose to) {
        return sequential(
                PathCommands.hold(follower, to),
                waitUntil(() -> Math.abs(Angle.normalizeSigned(follower.pose().heading() - to.heading())) < TURN_SETTLED_RAD
                        && Math.abs(follower.velocity().omega) < TURN_STILL_RAD_PER_SEC));
    }

    private boolean shouldEnd() {
        if (abortRequested) return true;
        if (legDeadlineNs == 0 || System.nanoTime() < legDeadlineNs) return false;
        abortReason = String.format("%s timed out after %.1f s", phase, legBudgetSec);
        return true;
    }

    /** Schedules the route on its own. */
    public void start() {
        Scheduler.schedule(command);
        if (!Scheduler.isScheduled(command)) {
            throw new IllegalStateException("route was not scheduled: another command of different priority holds the follower");
        }
    }

    /** The route as a command requiring the follower, for a group; don't also {@link #start()} it. */
    public Command command() {
        return command;
    }

    /**
     * Ends the route and stops the follower. Call from OpMode code, not from inside a command; inside a group
     * the route ends on the group's next tick.
     */
    public void abort(String reason) {
        if (!running) return;
        abortReason = reason;
        abortRequested = true;
        follower.stop();
        if (Scheduler.isScheduled(command)) Scheduler.cancel(command);
    }

    public boolean isRunning() {
        return running;
    }

    /** True only when every leg finished in time. */
    public boolean finished() {
        return finished;
    }

    /** Null unless the route was aborted, interrupted, or a leg timed out. */
    public String abortReason() {
        return abortReason;
    }

    public String phase() {
        return phase;
    }
}
