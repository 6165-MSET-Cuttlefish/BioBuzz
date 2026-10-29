package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Photon Stock", group = "Test")
public class PhotonStockBench extends PhotonWorkload {
    private static final double PROBE_POWER = 0.1;
    private static final long PROBE_MS = 200;
    private static final double PROBE_LIMIT_AMPS = 0.1;

    @Override
    public void runOpMode() {
        setUp("Photon Stock", "photon_stock_" + BenchIO.param(BenchIO.params(), "runId", "1"));
        Map<String, Object> info = rig.info(true);
        info.put("photonEnabledAtInit", PhotonCore.isEnabled());
        info.put("inFlightAtInit", rig.inFlight());
        Map<String, Object> left = new LinkedHashMap<>();
        for (LynxModule h : rig.hubs) left.put(rig.key(h), PhotonProbe.leftovers(h));
        info.put("leftoversAtInit", left);
        report.put("info", info);
        report.put("enableRefused", enableRefused());
        report.save();
        boolean probe = BenchIO.param(report.params, "portProbe", 0) != 0;
        if (!report.holdUntilStart(this)) return;
        if (probe) {
            Map<String, Object> ports = portProbe();
            report.put("portProbe", ports);
            report.save();
            if (!(Boolean) ports.get("empty")) {
                report.done("a motor port drew current: refusing to run the Photon benches");
                report.holdUntilStop(this);
                return;
            }
        }
        boolean shortRun = BenchIO.param(report.params, "short", 0) != 0;
        Map<String, Object> timing = new LinkedHashMap<>();
        report.put("timing", timing);
        timing.put("stock", timing("timing stock", -1, false));
        report.save();
        if (!shortRun && opModeIsActive()) {
            timing.put("stock+poller", timing("timing stock+poller", -1, true));
            report.put("burst", burst("burst stock", -1, Math.max(50, loops / 2)));
            report.put("interleave", interleave("interleave stock", -1, Math.max(30, loops / 5)));
            report.put("reads", reads("reads stock", -1, Math.max(100, loops / 2)));
        }
        rig.zeroMotors();
        rig.disableServos();
        report.done(String.format("stock write block p50 %s", dig(timing, "stock", "writeBlockMs", "p50")));
        report.holdUntilStop(this);
    }

    private Map<String, Object> enableRefused() {
        Map<String, Object> m = new LinkedHashMap<>();
        try {
            PhotonCore.enable();
            m.put("refused", false);
            PhotonCore.disable();
        } catch (IllegalStateException e) {
            m.put("refused", true);
            m.put("message", e.getMessage());
        }
        return m;
    }

    private Map<String, Object> portProbe() {
        report.phase("port probe");
        List<Map<String, Object>> rows = new ArrayList<>();
        boolean empty = true;
        for (PhotonChannel c : rig.channels) {
            if (!c.isMotor()) continue;
            double idle = c.motor.getCurrent(CurrentUnit.AMPS);
            c.set(PROBE_POWER);
            sleep(PROBE_MS);
            double driven = c.motor.getCurrent(CurrentUnit.AMPS);
            c.zero();
            Map<String, Object> r = new LinkedHashMap<>();
            r.put("channel", c.name);
            r.put("hub", c.hub.getModuleAddress());
            r.put("idleAmps", idle);
            r.put("drivenAmps", driven);
            boolean ok = driven <= PROBE_LIMIT_AMPS && driven - idle <= PROBE_LIMIT_AMPS;
            r.put("empty", ok);
            empty &= ok;
            rows.add(r);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("power", PROBE_POWER);
        m.put("ms", PROBE_MS);
        m.put("limitAmps", PROBE_LIMIT_AMPS);
        m.put("ports", rows);
        m.put("servoPortsProbed", false);
        m.put("empty", empty);
        return m;
    }

    private static Object dig(Map<String, Object> m, String... path) {
        Object o = m;
        for (String p : path) {
            if (!(o instanceof Map)) return null;
            o = ((Map<?, ?>) o).get(p);
        }
        return o;
    }
}
