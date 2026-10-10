package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

final class HubRegisters {
    private HubRegisters() {}

    static LynxModule controlHub(HardwareMap hardwareMap, String bench) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        if (hubs.size() != 1 || !hubs.get(0).isParent()) {
            throw new IllegalStateException(bench + " expects the Control Hub alone in the config, found " + hubs.size()
                    + " REV hubs; " + BenchIO.RUNNER_HINT);
        }
        return hubs.get(0);
    }

    static boolean responding(LynxModule hub) {
        hub.getInputVoltage(VoltageUnit.VOLTS);
        return !hub.isNotResponding();
    }

    static double motorPower(LynxModule hub, int port) {
        return send(new LynxGetMotorConstantPowerCommand(hub, port)).getPower()
                / (double) LynxSetMotorConstantPowerCommand.apiPowerLast;
    }

    static boolean motorEnabled(LynxModule hub, int port) {
        return send(new LynxGetMotorChannelEnableCommand(hub, port)).isEnabled();
    }

    static int servoPulseWidth(LynxModule hub, int port) {
        return send(new LynxGetServoPulseWidthCommand(hub, port)).getPulseWidth();
    }

    static boolean servoEnabled(LynxModule hub, int port) {
        return send(new LynxGetServoEnableCommand(hub, port)).isEnabled();
    }

    static void failSafe(LynxModule hub) {
        try {
            hub.failSafe();
        } catch (RobotCoreException | InterruptedException | LynxNackException e) {
            throw new IllegalStateException("failSafe() on the Control Hub failed", e);
        }
    }

    private static <R extends LynxMessage> R send(LynxRespondable<R> command) {
        try {
            return command.sendReceive();
        } catch (InterruptedException | LynxNackException e) {
            throw new IllegalStateException(command.getClass().getSimpleName() + " failed", e);
        }
    }
}
