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
     * Finishes once {@link Context#cell} has tipped onto its four AprilTags — that is, once all four
     * have been out of the Limelight's view for the hold time baked into that cell's pipeline. Any
     * one of them coming back into view restarts the window, so this does not finish on a single
     * dropped frame.
     *
     * <p>Which cell that is was fixed when the OpMode initialized and the pipeline was selected;
     * this command cannot retarget it.
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
