package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

/** Game actions composed from {@link BioBuzzRobot}'s modules, as Ivy commands. */
public class RobotActions {

    private final BioBuzzRobot robot;

    public RobotActions(BioBuzzRobot robot) {
        this.robot = robot;
    }

    /** {@link #checkTip(double)} with {@link LimelightCamera#checkTipTimeoutMs}. */
    public CommandBuilder checkTip() {
        return checkTip(LimelightCamera.checkTipTimeoutMs);
    }

    /**
     * Finishes once the HIVE cell has tipped down — its AprilTag cluster has read upside-down
     * ({@code |roll| >= 90}), or has been out of frame entirely, for the hold time baked into the
     * pipeline's script. An upright sighting clears the window, so this does not finish on a single
     * noisy frame near the 90° boundary.
     *
     * <p>Only clusters of {@link Context#allianceColor} count, and which alliance that is was fixed
     * when the OpMode initialized and the pipeline was selected; this command cannot retarget it.
     *
     * <p>It gives up after {@code timeoutMs} either way, so it cannot stall an auto on a cell that
     * never tips or a Limelight that never answers. Finishing therefore does <em>not</em> mean the
     * cell tipped — read {@link LimelightCamera#isTipped()} afterwards to tell the two apart.
     */
    public CommandBuilder checkTip(double timeoutMs) {
        Command watch = Command.build()
                .setStart(() -> robot.limelight.requireDevice())
                .setDone(() -> robot.limelight.isTipped())
                .requiring(robot.limelight)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
        return Groups.race(watch, Commands.waitMs(timeoutMs));
    }
}
