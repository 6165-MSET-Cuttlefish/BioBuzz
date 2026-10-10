package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.octoquad.OctoQuadLocalizerTest;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Expansion Hub", group = "Test")
public class ExpansionHubBench extends LinearOpMode {

    private static final class Timed {
        final Samples samples = new Samples();
        final Runnable read;

        Timed(Runnable read) {
            this.read = read;
        }
    }

    private BenchReport report;
    private int iterations;

    @Override
    public void runOpMode() {
        Map<String, Object> params = BenchIO.params();
        String runId = BenchIO.param(params, "runId", "ch");
        iterations = (int) BenchIO.param(params, "iterations", 300);
        report = new BenchReport("Expansion Hub", "expansion_hub_" + runId);
        while (opModeInInit()) {
            report.sendIfDue();
            sleep(20);
        }
        if (!opModeIsActive()) return;

        report.phase("hubs");
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("iterations", iterations);
        info.put("bulkCachingMode", LynxModule.BulkCachingMode.OFF.name());
        info.put("devices", BenchIO.deviceNames(hardwareMap));
        report.put("info", info);
        List<LynxModule> measured = new ArrayList<>();
        List<Map<String, Object>> hubs = new ArrayList<>();
        for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            boolean responding = HubRegisters.responding(m);
            Map<String, Object> h = new LinkedHashMap<>();
            h.put("address", m.getModuleAddress());
            h.put("parent", m.isParent());
            h.put("firmware", m.getNullableFirmwareVersionString());
            h.put("responding", responding);
            hubs.add(h);
            if (m.isParent()) measured.add(0, m);
            else if (responding) measured.add(m);
        }
        report.put("hubs", hubs);
        report.save();
        if (measured.isEmpty() || !measured.get(0).isParent()) {
            throw new IllegalStateException("Bench: Expansion Hub found no Control Hub in the config; " + BenchIO.RUNNER_HINT);
        }
        LynxModule controlHub = measured.get(0);
        boolean expansionHub = measured.size() > 1;

        Map<String, Runnable> odometry = new LinkedHashMap<>();
        Map<String, Map<String, Object>> devices = new LinkedHashMap<>();
        OctoQuad octoQuad = hardwareMap.tryGet(OctoQuad.class, OctoQuadLocalizerTest.name);
        if (octoQuad != null) {
            OctoQuad.LocalizerDataBlock block = new OctoQuad.LocalizerDataBlock();
            odometry.put(OctoQuadLocalizerTest.name, () -> octoQuad.readLocalizerData(block));
            devices.put(OctoQuadLocalizerTest.name, device(octoQuad.getConnectionInfo(), octoQuad.getChipId() & 0xFF));
        }
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, BettaConstants.pinpoint.name);
        if (pinpoint != null) {
            odometry.put(BettaConstants.pinpoint.name, pinpoint::update);
            devices.put(BettaConstants.pinpoint.name, device(pinpoint.getConnectionInfo(), pinpoint.getDeviceVersion()));
        }

        Map<String, Map<String, Timed>> commands = new LinkedHashMap<>();
        List<Timed> commandReads = new ArrayList<>();
        for (LynxModule hub : measured) {
            Map<String, Timed> perHub = new LinkedHashMap<>();
            perHub.put("inputVoltage", new Timed(() -> hub.getInputVoltage(VoltageUnit.VOLTS)));
            perHub.put("bulkData", new Timed(hub::getBulkData));
            perHub.put("current", new Timed(() -> hub.getCurrent(CurrentUnit.AMPS)));
            commandReads.addAll(perHub.values());
            commands.put(String.valueOf(hub.getModuleAddress()), perHub);
        }
        if (!roundRobin("commands", commandReads)) return;
        Map<String, Object> commandMs = new LinkedHashMap<>();
        for (Map.Entry<String, Map<String, Timed>> e : commands.entrySet()) commandMs.put(e.getKey(), summaries(e.getValue()));
        report.put("commands", commandMs);
        report.save();

        Map<String, Timed> deviceReads = new LinkedHashMap<>();
        for (Map.Entry<String, Runnable> e : odometry.entrySet()) deviceReads.put(e.getKey(), new Timed(e.getValue()));
        if (!roundRobin("devices", new ArrayList<>(deviceReads.values()))) return;
        for (Map.Entry<String, Timed> e : deviceReads.entrySet()) {
            devices.get(e.getKey()).put("readMs", e.getValue().samples.summary());
        }
        report.put("devices", devices);
        report.save();

        Map<String, Timed> passes = new LinkedHashMap<>();
        passes.put("controlHubBulk", new Timed(controlHub::getBulkData));
        if (expansionHub) {
            passes.put("bothHubsBulk", new Timed(() -> {
                for (LynxModule hub : measured) hub.getBulkData();
            }));
        }
        if (!odometry.isEmpty()) {
            passes.put("bothHubsBulkPlusOdometry", new Timed(() -> {
                for (LynxModule hub : measured) hub.getBulkData();
                for (Runnable read : odometry.values()) read.run();
            }));
        }
        if (!roundRobin("loopPass", new ArrayList<>(passes.values()))) return;
        report.put("loopPass", summaries(passes));

        report.done(String.format("%d of %d hub(s) answered, %d odometry device(s), %d iterations",
                measured.size(), hubs.size(), odometry.size(), iterations));
        while (opModeIsActive()) {
            report.sendIfDue();
            sleep(20);
        }
    }

    private static Map<String, Object> device(String connectionInfo, int chipIdOrVersion) {
        Map<String, Object> d = new LinkedHashMap<>();
        d.put("connectionInfo", connectionInfo);
        d.put("chipIdOrVersion", chipIdOrVersion);
        return d;
    }

    private boolean roundRobin(String phase, List<Timed> reads) {
        report.phase(phase);
        int n = reads.size();
        for (int i = 0; i < iterations; i++) {
            if (!opModeIsActive()) return false;
            for (int k = 0; k < n; k++) {
                Timed t = reads.get((i + k) % n);
                long t0 = System.nanoTime();
                t.read.run();
                t.samples.addNanos(System.nanoTime() - t0);
            }
            report.status("%s %d/%d", phase, i + 1, iterations);
            report.sendIfDue();
        }
        return true;
    }

    private static Map<String, Object> summaries(Map<String, Timed> timed) {
        Map<String, Object> m = new LinkedHashMap<>();
        for (Map.Entry<String, Timed> e : timed.entrySet()) m.put(e.getKey(), e.getValue().samples.summary());
        return m;
    }
}
