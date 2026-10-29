package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.LinkedHashMap;
import java.util.Map;

// Bypasses STOP's gate on purpose: the negative controls that prove the STOP checks can fail.
final class StopGateViolation {
    private static final double POWER = 0.2;
    private static final long WAIT_FOR_SAMPLER_NS = 400_000_000L;
    private static final long HOLD_NS = 150_000_000L;
    private static final Field UNFINISHED = PhotonProbe.field(LynxModule.class, "unfinishedCommands");
    private static final Method NEW_MESSAGE_NUMBER = method(LynxModule.class, "getNewMessageNumber");

    private StopGateViolation() {}

    /** One late power write and enable per hub, right after the fail-safe: the shape of a real Photon STOP failure. */
    static Map<String, Object> reEnergizeOnce(PhotonRig rig) {
        long t0 = System.nanoTime();
        int sends = 0;
        for (LynxModule hub : rig.hubs) {
            PhotonChannel c = rig.on(hub).get(0);
            rawPower(hub, c.port, PhotonChannel.motorRegister(c.motor, POWER));
            enable(hub, c.port);
            sends++;
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("variant", "once");
        m.put("sends", sends);
        m.put("msTaken", (System.nanoTime() - t0) / 1e6);
        return m;
    }

    static Map<String, Object> reEnergize(PhotonRig rig, StopSampler sampler) {
        long t0 = System.nanoTime();
        while (!sampler.failSafeSeenOnEveryHub() && System.nanoTime() - t0 < WAIT_FOR_SAMPLER_NS) Thread.yield();
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("variant", "held");
        m.put("samplerSawFailSafeFirst", sampler.failSafeSeenOnEveryHub());
        m.put("msWaitedForSampler", (System.nanoTime() - t0) / 1e6);
        int sends = 0;
        long start = System.nanoTime();
        while (System.nanoTime() - start < HOLD_NS) {
            for (LynxModule hub : rig.hubs) {
                PhotonChannel c = rig.on(hub).get(0);
                rawPower(hub, c.port, PhotonChannel.motorRegister(c.motor, POWER));
                enable(hub, c.port);
                sends++;
            }
            sleep(10);
        }
        m.put("sends", sends);
        m.put("msHeld", (System.nanoTime() - start) / 1e6);
        return m;
    }

    @SuppressWarnings({"unchecked", "rawtypes"})
    private static void rawPower(LynxModule hub, int port, int power) {
        LynxSetMotorConstantPowerCommand command = new LynxSetMotorConstantPowerCommand(hub, port, power);
        try {
            command.setMessageNumber((Byte) NEW_MESSAGE_NUMBER.invoke(hub));
            LynxDatagram datagram = new LynxDatagram(command);
            command.setSerialization(datagram);
            ((Map) PhotonProbe.get(UNFINISHED, hub)).put(command.getMessageNumber(), (LynxRespondable) command);
            LynxUsbDeviceImpl impl = PhotonProbe.impl(hub);
            synchronized (PhotonProbe.engageLock(impl)) {
                RobotUsbDevice device = impl.getRobotUsbDevice();
                if (device == null) throw new IllegalStateException("REV Hub #" + hub.getModuleAddress() + " has no USB device");
                device.write(datagram.toByteArray());
            }
        } catch (IllegalAccessException | InvocationTargetException | LynxUnsupportedCommandException | RobotUsbException e) {
            throw new IllegalStateException("the negative control couldn't write to REV Hub #" + hub.getModuleAddress(), e);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted writing the negative control", e);
        }
    }

    private static void enable(LynxModule hub, int port) {
        try {
            new LynxSetMotorChannelEnableCommand(hub, port, true).send();
        } catch (LynxNackException e) {
            throw new IllegalStateException("REV Hub #" + hub.getModuleAddress() + " refused the control's enable", e);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted enabling the negative control's motor", e);
        }
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted in the negative control", e);
        }
    }

    private static Method method(Class<?> owner, String name) {
        try {
            Method m = owner.getDeclaredMethod(name);
            m.setAccessible(true);
            return m;
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException(owner.getName() + "." + name + "() is gone", e);
        }
    }
}
