package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Ivy commands wrapping Pedro path following. Each one requires the {@link Follower}, so scheduling one interrupts whatever path is in flight. */
public final class PathCommands {
    private PathCommands() {}

    /** Follows {@code path} and finishes once Pedro's end-hold has settled. */
    public static CommandBuilder follow(Follower f, Path path) {
        return Command.build()
                .setStart(() -> {
                    f.holdEnd.set(true);
                    f.follow(path);
                })
                // isBusy() only clears from calculateHold, which runs only while holdEnd puts the follower in HOLD.
                .setDone(() -> f.atParametricEnd() && !f.isBusy())
                .setEnd(stopIfInterrupted(f))
                .requiring(f)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    /** Follows {@code path} and finishes at the parametric end, with whatever settling error remains. */
    public static CommandBuilder followThrough(Follower f, Path path) {
        return Command.build()
                .setStart(() -> {
                    f.holdEnd.set(false);
                    f.follow(path);
                })
                .setDone(f::atParametricEnd)
                .setEnd(stopIfInterrupted(f))
                .requiring(f)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    /**
     * Follows {@code path} but finishes {@code inches} short of the end of the whole path, leaving Pedro
     * driving the tail so the next command overlaps it.
     */
    public static CommandBuilder followUntilRemaining(Follower f, Path path, double inches) {
        List<PathSegment> segments = path.getSegments();
        double[] tails = new double[segments.size()];
        for (int i = segments.size() - 2; i >= 0; i--) tails[i] = tails[i + 1] + segments.get(i + 1).curve.length();
        // remainingDistance() is the current leg's, and it is stale until Pedro has updated on this path.
        DoubleSupplier remainingOnPath = () -> {
            int leg = f.pathIndex();
            return leg < 0 || leg >= tails.length ? Double.MAX_VALUE : f.remainingDistance() + tails[leg];
        };
        AtomicBoolean armed = new AtomicBoolean();
        return Command.build()
                .setStart(() -> {
                    armed.set(false);
                    f.holdEnd.set(true);
                    f.follow(path);
                })
                .setExecute(() -> {
                    if (remainingOnPath.getAsDouble() > inches) armed.set(true);
                })
                .setDone(() -> f.atParametricEnd() || (armed.get() && remainingOnPath.getAsDouble() <= inches))
                .setEnd(stopIfInterrupted(f))
                .requiring(f)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    /** Holds wherever the robot is when the command starts. */
    public static CommandBuilder hold(Follower f) {
        // Not PedroCommands.hold(f): that reads the pose when the command is built, which for a
        // sequence built in initialize() is the pose at INIT rather than the one reached by then.
        return Commands.instant(() -> f.hold(f.pose()))
                .requiring(f)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    public static CommandBuilder hold(Follower f, Pose pose) {
        return PedroCommands.hold(f, pose)
                .requiring(f)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    public static CommandBuilder stop(Follower f) {
        return Commands.instant(f::stop)
                .requiring(f)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    public static CommandBuilder timeout(Command command, double ms) {
        return Groups.race(command, Commands.waitMs(ms));
    }

    // Interrupted by a timeout, a race, or a newer path command: Pedro would otherwise finish the abandoned path and hold at its end.
    private static java.util.function.Consumer<EndCondition> stopIfInterrupted(Follower f) {
        return end -> {
            if (end == EndCondition.INTERRUPTED) f.stop();
        };
    }

    /** Trigger for {@code waitUntil}, e.g. {@code parallel(follow(f, p), sequential(waitUntil(remainingBelow(f, 8)), instant(...)))}. */
    public static BooleanSupplier remainingBelow(Follower f, double inches) {
        return () -> f.remainingDistance() <= inches;
    }
}
