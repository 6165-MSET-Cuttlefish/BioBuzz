package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.modules.CellTipCamera;

public class RobotActions {

    private final BioBuzzRobot robot;

    public RobotActions(BioBuzzRobot robot) {
        this.robot = robot;
    }

    public CommandBuilder checkTip() {
        return checkTip(CellTipCamera.checkTipTimeoutMs);
    }

    /**
     * Finishes when the cell tips or after {@code timeoutMs}, whichever comes first; check
     * {@link CellTipCamera#isTipped()} to tell which.
     */
    public CommandBuilder checkTip(double timeoutMs) {
        CellTipCamera cellTip = robot.cellTip;
        Command watch = Command.build()
                .setDone(cellTip::isTipped)
                .requiring(cellTip);
        return Groups.race(watch, Commands.waitMs(timeoutMs));
    }
}
