package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import org.firstinspires.ftc.robotcore.internal.hardware.usb.ArmableUsbDevice;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbUnspecifiedException;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

final class LinkTap implements InvocationHandler {
    interface AfterWrite {
        void run(int messageNumber);
    }

    static final int MOTORS = 1;
    static final int SERVOS = 2;
    private static final Field DEVICE = PhotonProbe.field(ArmableUsbDevice.class, "robotUsbDevice");

    private final LynxUsbDeviceImpl impl;
    private final Object engageLock;
    private final RobotUsbDevice real;
    private final RobotUsbDevice proxy;
    private final int address;
    private final int motorCommand;
    private final int servoCommand;
    private final Set<Integer> swallowing = ConcurrentHashMap.newKeySet();
    private final Set<Integer> channels = ConcurrentHashMap.newKeySet();
    private final List<Map<String, Object>> log = new ArrayList<>();
    private volatile Thread armedThread;
    private volatile int kinds;
    private volatile int toSwallow;
    private volatile int toThrow;
    private volatile AfterWrite afterWrite;
    private volatile long armedNs;

    private LinkTap(LynxModule hub) {
        impl = PhotonProbe.impl(hub);
        engageLock = PhotonProbe.engageLock(impl);
        address = hub.getModuleAddress();
        motorCommand = new LynxSetMotorConstantPowerCommand(hub, 0, 0).getCommandNumber();
        servoCommand = new LynxSetServoPulseWidthCommand(hub, 0, 1500).getCommandNumber();
        synchronized (engageLock) {
            real = (RobotUsbDevice) PhotonProbe.get(DEVICE, impl);
            if (real == null) throw new IllegalStateException("REV Hub #" + address + " has no USB device to tap");
            proxy = (RobotUsbDevice) Proxy.newProxyInstance(RobotUsbDevice.class.getClassLoader(),
                    new Class<?>[] {RobotUsbDevice.class}, this);
            set(proxy);
        }
    }

    static LinkTap install(LynxModule hub) {
        return new LinkTap(hub);
    }

    void remove() {
        synchronized (engageLock) {
            if (PhotonProbe.get(DEVICE, impl) == proxy) set(real);
        }
    }

    private void set(RobotUsbDevice device) {
        try {
            DEVICE.set(impl, device);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("can't swap REV Hub #" + address + "'s USB device", e);
        }
    }

    void swallowNext(int count, int kinds, int... ports) {
        this.kinds = kinds;
        channels.clear();
        for (int p : ports) channels.add(p);
        swallowing.clear();
        synchronized (log) {
            log.clear();
        }
        armedNs = System.nanoTime();
        toSwallow = count;
        armedThread = Thread.currentThread();
    }

    void throwOnNext(int kinds, int port) {
        this.kinds = kinds;
        channels.clear();
        channels.add(port);
        armedNs = System.nanoTime();
        toThrow = 1;
        armedThread = Thread.currentThread();
    }

    void afterNextMotorWrite(int port, AfterWrite action) {
        kinds = MOTORS;
        channels.clear();
        channels.add(port);
        afterWrite = action;
        armedNs = System.nanoTime();
        armedThread = Thread.currentThread();
    }

    // Photon reuses a message number once it gives up on the write: disarm before writing that channel again.
    void disarm() {
        armedThread = null;
        kinds = 0;
        toSwallow = 0;
        toThrow = 0;
        afterWrite = null;
        swallowing.clear();
    }

    int swallowedCount() {
        synchronized (log) {
            return log.size();
        }
    }

    int firstTransmissionsSwallowed() {
        int n = 0;
        synchronized (log) {
            for (Map<String, Object> e : log) if (!(Boolean) e.get("retransmit")) n++;
        }
        return n;
    }

    List<Map<String, Object>> log() {
        synchronized (log) {
            return new ArrayList<>(log);
        }
    }

    @Override
    public Object invoke(Object p, Method method, Object[] args) throws Throwable {
        if (method.getName().equals("write") && args != null && args.length == 1 && args[0] instanceof byte[]) {
            byte[] bytes = (byte[]) args[0];
            if (bytes.length >= 14 && (bytes[4] & 0xFF) == address) {
                int message = bytes[6] & 0xFF;
                int packet = (bytes[8] & 0xFF) | (bytes[9] & 0xFF) << 8;
                int channel = bytes[10] & 0xFF;
                boolean output = packet == motorCommand || packet == servoCommand;
                int key = message | channel << 8 | (packet == servoCommand ? 1 << 16 : 0);
                if (output && swallowing.contains(key)) {
                    note(message, packet, channel, true);
                    return null;
                }
                int kind = packet == motorCommand ? MOTORS : packet == servoCommand ? SERVOS : 0;
                boolean mine = (kind & kinds) != 0 && armedThread == Thread.currentThread() && channels.contains(channel);
                if (mine && toSwallow > 0) {
                    toSwallow--;
                    swallowing.add(key);
                    note(message, packet, channel, false);
                    return null;
                }
                if (mine && toThrow > 0) {
                    toThrow--;
                    note(message, packet, channel, false);
                    throw new RobotUsbUnspecifiedException("bench: injected USB write failure");
                }
                Object result = forward(method, args);
                AfterWrite action = afterWrite;
                if (mine && packet == motorCommand && action != null) {
                    afterWrite = null;
                    armedThread = null;
                    action.run(message);
                }
                return result;
            }
        }
        return forward(method, args);
    }

    private Object forward(Method method, Object[] args) throws Throwable {
        try {
            return method.invoke(real, args);
        } catch (InvocationTargetException e) {
            throw e.getCause();
        }
    }

    private void note(int message, int packet, int channel, boolean retransmit) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("msAfterArming", (System.nanoTime() - armedNs) / 1e6);
        m.put("messageNumber", message);
        m.put("kind", packet == motorCommand ? "motor" : "servo");
        m.put("port", channel);
        m.put("retransmit", retransmit);
        synchronized (log) {
            log.add(m);
        }
    }
}
