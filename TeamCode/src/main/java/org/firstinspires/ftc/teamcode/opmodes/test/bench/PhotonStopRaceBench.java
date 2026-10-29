package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

// The raised gate stays up until the next OpMode's INIT, so one trial per run.
@Photon
@TeleOp(name = "Bench: Photon Stop Race", group = "Test")
public class PhotonStopRaceBench extends LinearOpMode {
    @Override
    public void runOpMode() {
        String trial = BenchIO.param(BenchIO.params(), "trial", "0");
        BenchReport report = new BenchReport("Photon Stop Race", "photon_stop_race_" + trial);
        PhotonRig rig = new PhotonRig(hardwareMap, report.params, "Bench: Photon Stop Race");
        int parallel = (int) BenchIO.param(report.params, "photon", 8);
        String hubKey = BenchIO.param(report.params, "hub", "controlHub");
        if (!hubKey.equals("controlHub") && !hubKey.equals("expansionHub")) throw new IllegalArgumentException("hub=" + hubKey);
        if (hubKey.equals("expansionHub") && rig.expansionHub == null) {
            throw new IllegalStateException("Bench: Photon Stop Race was asked for the Expansion Hub, but the config has none");
        }
        LynxModule hub = hubKey.equals("expansionHub") ? rig.expansionHub : rig.controlHub;
        PhotonChannel c = rig.on(hub).get(0);
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("photon", parallel);
        r.put("hub", rig.key(hub));
        r.put("channel", c.name);
        r.put("parallelCapable", PhotonRig.parallelCapable(hub));
        report.put("info", rig.info());
        report.put("result", r);
        report.save();
        if (!report.holdUntilStart(this)) return;
        PhotonWorkload.setMode(parallel);
        c.set(0.2);
        rig.drain(500);
        c.motor.setMotorDisable();
        c.motor.setMotorEnable();
        sleep(20);
        r.put("enabledBefore", c.enabled());
        Map<String, Object> before = PhotonProbe.counters();
        final List<LynxModule> hubs = rig.hubs;
        final Thread[] stopper = new Thread[1];
        final long[] gateNs = new long[1];
        LinkTap tap = LinkTap.install(hub);
        long t0;
        try {
            tap.afterNextMotorWrite(c.port, message -> {
                PhotonProbe.raiseGate();
                gateNs[0] = System.nanoTime();
                stopper[0] = new Thread(() -> {
                    for (LynxModule h : hubs) h.attemptFailSafeAndIgnoreErrors();
                }, "PhotonStopRaceFailSafe");
                stopper[0].start();
            });
            t0 = System.nanoTime();
            c.set(0.25);
            r.put("writeCallMs", (System.nanoTime() - t0) / 1e6);
            tap.disarm();
        } finally {
            tap.remove();
        }
        r.put("gateRaisedInsideWrite", gateNs[0] != 0);
        r.put("gateRaisedMsAfterWriteStart", gateNs[0] == 0 ? null : (gateNs[0] - t0) / 1e6);
        if (stopper[0] != null) PhotonBench.joinOrThrow(stopper[0], "STOP fail-safe thread");
        List<Map<String, Object>> reads = new ArrayList<>();
        boolean violation = false;
        for (long at : new long[] {30, 80, 150, 300}) {
            while ((System.nanoTime() - t0) / 1e6 < at) sleep(1);
            Map<String, Object> m = new LinkedHashMap<>();
            boolean on = c.enabled();
            int raw = c.read();
            m.put("msAfterWrite", (System.nanoTime() - t0) / 1e6);
            m.put("enabled", on);
            m.put("raw", raw);
            reads.add(m);
            violation |= on && Math.abs(raw) > PhotonChannel.POWER_VIOLATION_RAW;
        }
        r.put("reads", reads);
        r.put("violation", violation);
        r.put("photonEnabledAfter", PhotonCore.isEnabled());
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        rig.zeroMotors();
        rig.disableServos();
        report.done((violation ? "VIOLATION: " : "no violation: ") + c.name + " after STOP's gate in the write window");
        report.holdUntilStop(this);
    }
}
