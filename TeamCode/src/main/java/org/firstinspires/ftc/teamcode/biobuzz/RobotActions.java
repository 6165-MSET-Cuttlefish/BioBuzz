package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

public class RobotActions {

    private final BioBuzzRobot robot;

    public RobotActions(BioBuzzRobot robot) {
        this.robot = robot;
    }

    public CommandBuilder checkTip() {
        return checkTip(LimelightCamera.checkTipTimeoutMs);
    }

    /**
     * Finishes when the cell tips or after {@code timeoutMs}, whichever comes first; check
     * {@link LimelightCamera#isTipped()} to tell which. Throws at build time if there is no Limelight.
     */
    public CommandBuilder checkTip(double timeoutMs) {
        LimelightCamera limelight = robot.limelight.requireDevice();
        Command watch = Command.build()
                .setDone(limelight::isTipped)
                .requiring(limelight);
        return Groups.race(watch, Commands.waitMs(timeoutMs));
    }
}
