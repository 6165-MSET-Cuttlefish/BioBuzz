package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

/** Game actions composed from {@link BioBuzzRobot}'s modules, as Ivy commands. */
public class RobotActions {

    private final BioBuzzRobot robot;

    public RobotActions(BioBuzzRobot robot) {
        this.robot = robot;
    }

    /** {@link #checkTip(LimelightCamera.Cell, double)} with {@link LimelightCamera#checkTipTimeoutMs}. */
    public CommandBuilder checkTip(LimelightCamera.Cell cell) {
        return checkTip(cell, LimelightCamera.checkTipTimeoutMs);
    }

    /**
     * Watches {@code cell}'s four AprilTags and finishes once the cell has tipped onto them — that
     * is, once all four have been out of the Limelight's view for
     * {@link LimelightCamera#hiddenHoldSeconds}. Any one of them coming back into view restarts the
     * window, so this does not finish on a single dropped frame.
     *
     * <p>It gives up after {@code timeoutMs} either way, so it cannot stall an auto on a cell that
     * never tips or a Limelight that never answers. Finishing therefore does <em>not</em> mean the
     * cell tipped — read {@link LimelightCamera#isTipped()} afterwards to tell the two apart.
     */
    public CommandBuilder checkTip(LimelightCamera.Cell cell, double timeoutMs) {
        Command watch = Command.build()
                .setStart(() -> {
                    robot.limelight.requireDevice();
                    robot.limelight.watch(cell);
                })
                .setDone(() -> robot.limelight.isTipped())
                .requiring(robot.limelight)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
        return Groups.race(watch, Commands.waitMs(timeoutMs));
    }
}
