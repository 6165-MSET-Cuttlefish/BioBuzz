package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

final class PhotonRig {
    static final String[] CH_MOTORS = {"fl", "fr", "bl", "br"};
    static final String[] CH_SERVOS = {"benchServo", "benchServo2"};
    static final String[] EH_MOTORS = {"efl", "efr", "ebl", "ebr"};
    static final String[] EH_SERVOS = {"eServo", "eServo2"};
    static final int NONE = 0;
    static final int USB = 1;
    static final int RS485 = 2;

    final HardwareMap map;
    final int topology;
    final LynxModule controlHub;
    final LynxModule expansionHub;
    final List<LynxModule> hubs = new ArrayList<>();
    final List<PhotonChannel> channels = new ArrayList<>();

    PhotonRig(HardwareMap map, Map<String, Object> params, String bench) {
        this.map = map;
        topology = (int) BenchIO.param(params, "expansion", NONE);
        LynxModule ch = null;
        LynxModule eh = null;
        for (LynxModule m : map.getAll(LynxModule.class)) {
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) ch = m;
            else eh = m;
            hubs.add(m);
        }
        controlHub = ch;
        expansionHub = eh;
        if (controlHub == null) throw new IllegalStateException(bench + " found no Control Hub; " + BenchIO.RUNNER_HINT);
        int want = topology == NONE ? 1 : 2;
        if (hubs.size() != want || (topology != NONE && expansionHub == null)) {
            throw new IllegalStateException(bench + " expected " + want + " REV hub(s) for expansion=" + topology + ", found "
                    + hubs.size() + "; " + BenchIO.RUNNER_HINT);
        }
        if (topology == USB && !expansionHub.isParent()) {
            throw new IllegalStateException("the Expansion Hub answers on RS485, but the runner asked for USB: connect it by USB alone");
        }
        if (topology == RS485 && expansionHub.isParent()) {
            throw new IllegalStateException("the Expansion Hub has its own USB link, but the runner asked for RS485");
        }
        for (LynxModule m : hubs) m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        add(controlHub, CH_MOTORS, CH_SERVOS);
        if (expansionHub != null) add(expansionHub, EH_MOTORS, EH_SERVOS);
    }

    private void add(LynxModule hub, String[] motors, String[] servos) {
        for (String name : motors) {
            DcMotorEx m = BenchIO.require(map, DcMotorEx.class, name, BenchIO.RUNNER_HINT);
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            channels.add(new PhotonChannel(name, hub, m, null));
        }
        for (String name : servos) {
            ServoImplEx s = BenchIO.require(map, ServoImplEx.class, name, BenchIO.RUNNER_HINT);
            s.setDirection(Servo.Direction.FORWARD);
            channels.add(new PhotonChannel(name, hub, null, s));
        }
    }

    static boolean parallelCapable(LynxModule hub) {
        return hub.isParent();
    }

    List<PhotonChannel> on(LynxModule hub) {
        List<PhotonChannel> out = new ArrayList<>();
        for (PhotonChannel c : channels) if (c.hub == hub) out.add(c);
        return out;
    }

    void writeAll() {
        for (PhotonChannel c : channels) c.write();
    }

    void zeroMotors() {
        for (PhotonChannel c : channels) c.zero();
    }

    void disableServos() {
        for (PhotonChannel c : channels) if (!c.isMotor()) c.servo.setPwmDisable();
    }

    int inFlight() {
        int n = 0;
        for (LynxModule h : hubs) n += PhotonProbe.inFlight(h);
        return n;
    }

    double drain(double timeoutMs) {
        long t0 = System.nanoTime();
        while (inFlight() > 0) {
            if (System.nanoTime() - t0 > timeoutMs * 1e6) return Double.NaN;
            Thread.yield();
        }
        return (System.nanoTime() - t0) / 1e6;
    }

    // A field of the installed APK's BuildConfig, read reflectively so a hot-loaded bench can't carry an inlined copy.
    static String installedPhotonSourceSha256() {
        try {
            return (String) com.qualcomm.ftcrobotcontroller.BuildConfig.class.getField("PHOTON_SOURCE_SHA256").get(null);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new IllegalStateException("the installed APK predates BuildConfig.PHOTON_SOURCE_SHA256: do a full install (Run)", e);
        }
    }

    /** Every bench channel's enable and register (motor raw power, servo pulse width), read from the hubs now. */
    Map<String, Object> outputs() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("gateUp", PhotonProbe.gateUp());
        m.put("photonEnabled", PhotonCore.isEnabled());
        List<Map<String, Object>> list = new ArrayList<>();
        for (PhotonChannel c : channels) {
            Map<String, Object> e = new LinkedHashMap<>();
            e.put("channel", c.name);
            e.put("hub", key(c.hub));
            e.put("kind", c.isMotor() ? "motor" : "servo");
            e.put("enabled", c.enabled());
            e.put("raw", c.read());
            list.add(e);
        }
        m.put("outputs", list);
        return m;
    }

    List<Integer> notResponding() {
        List<Integer> out = new ArrayList<>();
        for (LynxModule h : hubs) if (h.isNotResponding()) out.add(h.getModuleAddress());
        return out;
    }

    String key(LynxModule hub) {
        return hub == controlHub ? "controlHub" : "expansionHub";
    }

    Map<String, Object> info() {
        return info(false);
    }

    // Firmware is a bus read the hub can take over 250 ms to answer right after a restart, even on the SDK's own path.
    Map<String, Object> info(boolean firmware) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("topology", topology == NONE ? "controlHubOnly" : topology == USB ? "usbExpansionHub" : "rs485ExpansionHub");
        List<Map<String, Object>> list = new ArrayList<>();
        for (LynxModule h : hubs) {
            Map<String, Object> e = new LinkedHashMap<>();
            e.put("key", key(h));
            e.put("serial", h.getSerialNumber().toString());
            e.put("address", h.getModuleAddress());
            e.put("parent", h.isParent());
            e.put("class", h.getClass().getName());
            e.put("swapped", PhotonCore.isSwapped(h));
            if (firmware) e.put("firmware", h.getNullableFirmwareVersionString());
            list.add(e);
        }
        m.put("hubs", list);
        List<Map<String, Object>> ch = new ArrayList<>();
        for (PhotonChannel c : channels) {
            Map<String, Object> e = new LinkedHashMap<>();
            e.put("name", c.name);
            e.put("hub", c.hub.getModuleAddress());
            e.put("kind", c.isMotor() ? "motor" : "servo");
            e.put("port", c.port);
            if (c.isMotor()) {
                e.put("motorType", c.motor.getMotorType().getName());
                e.put("orientation", c.motor.getMotorType().getOrientation().toString());
                e.put("expectedRegisterAtPowerA", PhotonChannel.motorRegister(c.motor, PhotonChannel.POWER_A));
            } else {
                e.put("pwmRange", new double[] {c.servo.getPwmRange().usPulseLower, c.servo.getPwmRange().usPulseUpper});
            }
            ch.add(e);
        }
        m.put("channels", ch);
        m.put("photonSwapFailures", new ArrayList<>(PhotonCore.swapFailures()));
        m.put("installedPhotonSourceSha256", installedPhotonSourceSha256());
        m.put("sdkVersion", com.qualcomm.robotcore.util.Version.getLibraryVersion());
        return m;
    }
}
