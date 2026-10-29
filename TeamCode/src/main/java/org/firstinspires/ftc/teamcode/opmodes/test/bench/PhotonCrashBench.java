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
import java.util.concurrent.locks.LockSupport;

@Photon
@TeleOp(name = "Bench: Photon Crash", group = "Test")
public class PhotonCrashBench extends LinearOpMode {
    private static final long TAP_RELEASE_TIMEOUT_NS = 2_000_000_000L;

    @Override
    public void runOpMode() {
        String trial = BenchIO.param(BenchIO.params(), "trial", "0");
        BenchReport report = new BenchReport("Photon Crash", "photon_crash_" + trial);
        PhotonRig rig = new PhotonRig(hardwareMap, report.params, "Bench: Photon Crash");
        double crashAfterMs = BenchIO.param(report.params, "crashAfterMs", 1200);
        int lose = (int) BenchIO.param(report.params, "lostWrites", 2);
        Map<String, Object> info = rig.info();
        info.put("variant", "crash");
        info.put("photon", 8);
        info.put("negativeControl", false);
        info.put("trial", trial);
        report.put("info", info);
        report.save();
        StopSampler sampler = new StopSampler("photon_crash_sampler_" + trial, rig, Thread.currentThread(), info);
        if (!report.holdUntilStart(this)) return;
        PhotonCore.setMaxParallelWrites(8);
        PhotonCore.enable();
        report.phase("writing");
        sampler.start();
        long t0 = System.nanoTime();
        while (opModeIsActive() && (System.nanoTime() - t0) / 1e6 < crashAfterMs) {
            sampler.noteLoop();
            rig.writeAll();
            report.sendIfDue();
        }
        report.phase("losing writes");
        report.put("drainBeforeLostWritesMs", rig.drain(500));
        Map<LynxModule, LinkTap> tapOf = new LinkedHashMap<>();
        for (LynxModule hub : rig.hubs) {
            if (!PhotonRig.parallelCapable(hub)) continue;
            List<Integer> ports = new ArrayList<>();
            for (PhotonChannel c : rig.on(hub)) if (c.isMotor() && ports.size() < lose) ports.add(c.port);
            int[] armed = new int[ports.size()];
            for (int i = 0; i < armed.length; i++) armed[i] = ports.get(i);
            LinkTap tap = LinkTap.install(hub);
            tapOf.put(hub, tap);
            tap.swallowNext(armed.length, LinkTap.MOTORS, armed);
        }
        rig.writeAll();
        Map<String, Object> lost = new LinkedHashMap<>();
        for (Map.Entry<LynxModule, LinkTap> e : tapOf.entrySet()) {
            lost.put(rig.key(e.getKey()), e.getValue().firstTransmissionsSwallowed());
        }
        startTapRelease(new ArrayList<>(tapOf.values()), rig.hubs, Thread.currentThread());
        int inFlight = rig.inFlight();
        report.put("inFlightAtThrow", inFlight);
        report.put("lostAtThrow", lost);
        report.save();
        sampler.put("inFlightAtThrow", inFlight);
        throw new IllegalStateException("BENCH injected crash with " + inFlight + " Photon writes in flight, " + lost
                + " of them never to be acknowledged");
    }

    // Outlives the throw so those writes never get an ACK; removed once Photon finished them, as freed numbers get reused.
    private static void startTapRelease(List<LinkTap> taps, List<LynxModule> hubs, Thread opModeThread) {
        Thread release = new Thread(() -> {
            long deadline = System.nanoTime() + TAP_RELEASE_TIMEOUT_NS;
            while (System.nanoTime() < deadline && (opModeThread.isAlive() || anyInFlight(hubs))) {
                LockSupport.parkNanos(2_000_000L);
            }
            for (LinkTap tap : taps) tap.remove();
            BenchIO.log("crash bench removed its USB taps; in flight then %s", anyInFlight(hubs));
        }, "PhotonCrashTapRelease");
        release.setDaemon(true);
        release.start();
    }

    private static boolean anyInFlight(List<LynxModule> hubs) {
        for (LynxModule h : hubs) if (PhotonProbe.inFlight(h) > 0) return true;
        return false;
    }
}
