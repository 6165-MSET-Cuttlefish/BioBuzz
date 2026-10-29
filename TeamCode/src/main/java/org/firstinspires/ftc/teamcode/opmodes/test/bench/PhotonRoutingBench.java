package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Photon
@TeleOp(name = "Bench: Photon Routing", group = "Test")
public class PhotonRoutingBench extends LinearOpMode {
    private static final double CLEAR_WAIT_MS = 1000;

    @Override
    public void runOpMode() {
        BenchReport report = new BenchReport("Photon Routing", "photon_routing_" + BenchIO.param(BenchIO.params(), "runId", "1"));
        Map<String, Object> init = new LinkedHashMap<>();
        init.put("photonEnabled", PhotonCore.isEnabled());
        Map<String, Object> inFlight = new LinkedHashMap<>();
        Map<String, Object> pending = new LinkedHashMap<>();
        Map<String, Object> standIns = new LinkedHashMap<>();
        Map<String, Object> routes = new LinkedHashMap<>();
        Map<String, Object> unfinished = new LinkedHashMap<>();
        List<Integer> mapOrder = new ArrayList<>();
        for (HardwareDevice d : hardwareMap.unsafeIterable()) {
            if (!(d instanceof LynxModule)) continue;
            LynxModule h = (LynxModule) d;
            mapOrder.add(h.getModuleAddress());
            String address = String.valueOf(h.getModuleAddress());
            inFlight.put(address, PhotonProbe.inFlight(h));
            pending.put(address, PhotonProbe.pendingEntries(h));
            standIns.put(address, PhotonProbe.standIns(h));
            routes.put(address, PhotonProbe.routes(h));
            unfinished.put(address, PhotonProbe.unfinished(h));
        }
        init.put("inFlight", inFlight);
        init.put("pendingEntries", pending);
        init.put("standIns", standIns);
        init.put("routes", routes);
        init.put("unfinishedCommands", unfinished);
        init.put("swapFailures", new ArrayList<>(PhotonCore.swapFailures()));
        init.put("swapOrder", PhotonProbe.swapOrder());
        init.put("mapOrder", mapOrder);
        init.put("routingProblems", PhotonProbe.routingProblems(hardwareMap));
        init.put("counters", PhotonProbe.counters());
        init.put("loaders", PhotonProbe.loaders(this, hardwareMap));
        report.put("atInit", init);
        report.save();
        PhotonRig rig = new PhotonRig(hardwareMap, report.params, "Bench: Photon Routing");
        report.put("info", rig.info());
        report.save();
        if (!report.holdUntilStart(this)) return;
        Map<String, Object> recovery = new LinkedHashMap<>();
        report.put("notRespondingRecovery", recovery);
        try {
            for (int parallel : new int[] {0, 8}) {
                report.phase("not responding " + PhotonWorkload.modeName(parallel));
                PhotonWorkload.setMode(parallel);
                Map<String, Object> byHub = new LinkedHashMap<>();
                for (LynxModule h : rig.hubs) byHub.put(rig.key(h), recover(h));
                recovery.put(PhotonWorkload.modeName(parallel), byHub);
                report.save();
            }
            report.phase("parallel check");
            PhotonWorkload.setMode(8);
            Map<String, Object> before = PhotonProbe.counters();
            long issued = 0;
            for (int i = 0; i < 20 && opModeIsActive(); i++) {
                for (PhotonChannel c : rig.channels) {
                    c.write();
                    if (PhotonRig.parallelCapable(c.hub)) issued++;
                }
            }
            Map<String, Object> check = new LinkedHashMap<>();
            check.put("drainMs", rig.drain(500));
            check.put("issuedOnParallelHubs", issued);
            check.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
            PhotonChannel.Mismatches m = new PhotonChannel.Mismatches();
            for (PhotonChannel c : rig.channels) m.check("parallel check", 20, c, true);
            check.put("readback", m.toMap());
            check.put("routingProblemsAfter", PhotonProbe.routingProblems(hardwareMap));
            Map<String, Object> left = new LinkedHashMap<>();
            for (LynxModule h : rig.hubs) left.put(rig.key(h), PhotonProbe.leftovers(h));
            check.put("leftoversAfterDrain", left);
            report.put("parallelCheck", check);
        } finally {
            PhotonCore.disable();
            rig.zeroMotors();
            rig.disableServos();
        }
        report.done("swap order " + init.get("swapOrder") + ", routing problems " + init.get("routingProblems"));
        report.holdUntilStop(this);
    }

    private Map<String, Object> recover(LynxModule hub) {
        Map<String, Object> m = new LinkedHashMap<>();
        hub.noteNotResponding();
        m.put("visible", hub.isNotResponding());
        long t0 = System.nanoTime();
        hub.getInputVoltage(VoltageUnit.VOLTS);
        double clearedMs = Double.NaN;
        while ((System.nanoTime() - t0) / 1e6 < CLEAR_WAIT_MS) {
            if (!hub.isNotResponding()) {
                clearedMs = (System.nanoTime() - t0) / 1e6;
                break;
            }
            Thread.yield();
        }
        m.put("clearedMs", clearedMs);
        m.put("voltsAfter", hub.getInputVoltage(VoltageUnit.VOLTS));
        m.put("notRespondingAtEnd", hub.isNotResponding());
        return m;
    }
}
