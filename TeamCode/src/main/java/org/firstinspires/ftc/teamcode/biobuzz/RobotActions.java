package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;

import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

/** Game actions composed from {@link BioBuzzRobot}'s modules, as Ivy commands. */
public class RobotActions {

    private final BioBuzzRobot robot;

    public RobotActions(BioBuzzRobot robot) {
        this.robot = robot;
    }

    /**
     * Watches {@code cell}'s four AprilTags and finishes once the cell has tipped onto them — that
     * is, once all four have been out of the Limelight's view for
     * {@link LimelightCamera#hiddenHoldSeconds}. Any one of them coming back into view restarts the
     * window, so this does not finish on a single dropped frame.
     *
     * <p>It never finishes on its own if the cell is never in view to begin with and
     * {@link LimelightCamera#requireSeenBeforeTip} is set, and it cannot finish while the Limelight
     * is unreachable, so wrap it in a timeout like any other auto step.
     */
    public CommandBuilder checkTip(LimelightCamera.Cell cell) {
        return Command.build()
                .setStart(() -> {
                    robot.limelight.requireDevice();
                    robot.limelight.watch(cell);
                })
                .setDone(() -> robot.limelight.isTipped())
                .requiring(robot.limelight)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }
}
