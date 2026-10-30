package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

final class PhotonChannel {
    static final double POWER_A = 0.2;
    static final double POWER_B = 0.25;
    static final double POSITION_A = 0.4;
    static final double POSITION_B = 0.6;
    static final int POWER_VIOLATION_RAW = (int) (0.01 * 32767);

    final String name;
    final LynxModule hub;
    final DcMotorEx motor;
    final ServoImplEx servo;
    final int port;
    final boolean wrapped;
    private final DcMotorEx motorOut;
    private final Servo servoOut;
    private long step;
    private double commanded = Double.NaN;
    private int expected;
    private volatile long lastWriteNs;

    PhotonChannel(String name, LynxModule hub, DcMotorEx motor, ServoImplEx servo) {
        this(name, hub, motor, servo, motor, servo, false);
    }

    private PhotonChannel(String name, LynxModule hub, DcMotorEx motor, ServoImplEx servo, DcMotorEx motorOut,
                          Servo servoOut, boolean wrapped) {
        this.name = name;
        this.hub = hub;
        this.motor = motor;
        this.servo = servo;
        this.motorOut = motorOut;
        this.servoOut = servoOut;
        this.wrapped = wrapped;
        port = motor != null ? motor.getPortNumber() : servo.getPortNumber();
    }

    PhotonChannel throughWriteCache(HardwareMap map) {
        return isMotor() ? new PhotonChannel(name, hub, motor, null, new EnhancedMotor(map, name), null, true)
                : new PhotonChannel(name, hub, null, servo, null, new EnhancedServo(map, name), true);
    }

    boolean isMotor() {
        return motor != null;
    }

    void write() {
        prepare();
        send();
    }

    // Always a new value, so the SDK never skips it as unchanged and every write is counted.
    void prepare() {
        double value = isMotor() ? (step % 2 == 0 ? POWER_A : POWER_B) : (step % 2 == 0 ? POSITION_A : POSITION_B);
        step++;
        commanded = value;
        expected = isMotor() ? motorRegister(motor, value) : servoPulse(servo, value);
    }

    void send() {
        if (isMotor()) motorOut.setPower(commanded);
        else servoOut.setPosition(commanded);
        lastWriteNs = System.nanoTime();
    }

    void set(double value) {
        commanded = value;
        expected = isMotor() ? motorRegister(motor, value) : servoPulse(servo, value);
        send();
    }

    void zero() {
        if (isMotor()) set(0);
    }

    double commanded() {
        return commanded;
    }

    int expected() {
        return expected;
    }

    int read() {
        return isMotor() ? HubRegisters.motorPowerRaw(hub, port) : HubRegisters.servoPulseWidth(hub, port);
    }

    boolean enabled() {
        return isMotor() ? HubRegisters.motorEnabled(hub, port) : HubRegisters.servoEnabled(hub, port);
    }

    double msSinceWrite() {
        long t = lastWriteNs;
        return t == 0 ? Double.NaN : (System.nanoTime() - t) / 1e6;
    }

    // DcMotorImpl.getOperationalDirection(): a CCW motor type (goBILDA 5202) inverts the configured direction.
    static int motorRegister(DcMotorEx motor, double power) {
        DcMotorSimple.Direction direction = motor.getDirection();
        if (motor.getMotorType().getOrientation() == Rotation.CCW) direction = direction.inverted();
        double signed = direction == DcMotorSimple.Direction.REVERSE ? -power : power;
        return (int) Range.scale(Range.clip(signed, -1, 1), -1, 1, -32767, 32767);
    }

    static int servoPulse(ServoImplEx servo, double position) {
        double p = servo.getDirection() == Servo.Direction.REVERSE ? 1 - position : position;
        PwmControl.PwmRange r = servo.getPwmRange();
        return (int) Range.clip(Range.scale(p, 0, 1, r.usPulseLower, r.usPulseUpper), 1, 65535);
    }

    static final class Mismatches {
        private static final int KEEP = 60;
        int checks;
        int count;
        final List<Map<String, Object>> events = new ArrayList<>();

        boolean check(String phase, long step, PhotonChannel c, boolean wantEnabled) {
            checks++;
            int got = c.read();
            boolean enabledNow = wantEnabled ? c.enabled() : true;
            if (got == c.expected() && enabledNow) return true;
            count++;
            if (events.size() < KEEP) {
                Map<String, Object> e = new LinkedHashMap<>();
                e.put("phase", phase);
                e.put("step", step);
                e.put("hub", c.hub.getModuleAddress());
                e.put("channel", c.name);
                e.put("expected", c.expected());
                e.put("got", got);
                e.put("commanded", c.commanded());
                e.put("hubEnabled", wantEnabled ? enabledNow : null);
                e.put("photonEnabled", PhotonCore.isEnabled());
                e.put("msSinceWrite", c.msSinceWrite());
                e.put("inFlightOnHub", PhotonProbe.inFlight(c.hub));
                events.add(e);
            }
            return false;
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("checks", checks);
            m.put("mismatches", count);
            m.put("events", events);
            return m;
        }
    }
}
