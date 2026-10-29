package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.LockSupport;

@Photon
@TeleOp(name = "Bench: Photon Replug", group = "Test")
public class PhotonReplugBench extends LinearOpMode {
    private static final double HEALTHY_FOR_MS = 3000;

    @Override
    public void runOpMode() {
        Map<String, Object> params = BenchIO.params();
        String mode = BenchIO.param(params, "mode", "usb");
        int parallel = (int) BenchIO.param(params, "photon", 8);
        BenchReport report = new BenchReport("Photon Replug", "photon_replug_" + mode + "_" + PhotonWorkload.modeName(parallel));
        PhotonRig rig = new PhotonRig(hardwareMap, report.params, "Bench: Photon Replug");
        if (rig.topology != PhotonRig.USB) throw new IllegalStateException("Bench: Photon Replug needs the USB Expansion Hub");
        double maxSeconds = BenchIO.param(report.params, "maxSeconds", 90);
        report.put("info", rig.info());
        report.put("mode", mode);
        report.put("photon", parallel);
        report.save();
        if (!report.holdUntilStart(this)) return;
        PhotonWorkload.setMode(parallel);
        report.phase("waiting for the " + mode + " pull");
        LynxModule eh = rig.expansionHub;
        List<PhotonChannel> ehChannels = rig.on(eh);
        List<PhotonChannel> chChannels = rig.on(rig.controlHub);
        PhotonChannel.Mismatches chReadback = new PhotonChannel.Mismatches();
        Map<String, Object> before = PhotonProbe.counters();
        Gap gap = new Gap(Thread.currentThread());
        gap.start();
        List<Map<String, Object>> transitions = new ArrayList<>();
        long t0 = System.nanoTime();
        double faultStartMs = Double.NaN;
        double lastUnhealthyMs = Double.NaN;
        double recoveredMs = Double.NaN;
        double lastVolts = Double.NaN;
        double maxSingleWriteMs = 0;
        double backMs = Double.NaN;
        double backToParallelMs = Double.NaN;
        boolean wasHealthy = true;
        long loop = 0;
        Map<String, Object> atFault = null;
        Map<String, Object> faultWindow = null;
        try {
            while (opModeIsActive() && (System.nanoTime() - t0) / 1e9 < maxSeconds) {
                double now = (System.nanoTime() - t0) / 1e6;
                gap.enter();
                for (PhotonChannel c : rig.channels) {
                    long w = System.nanoTime();
                    c.write();
                    maxSingleWriteMs = Math.max(maxSingleWriteMs, (System.nanoTime() - w) / 1e6);
                }
                gap.exit();
                if (!Double.isNaN(backMs) && Double.isNaN(backToParallelMs) && PhotonProbe.inFlight(eh) > 0) {
                    backToParallelMs = now - backMs;
                }
                LynxUsbDeviceImpl impl = PhotonProbe.impl(eh);
                RobotArmingStateNotifier.ARMINGSTATE arming = impl.getArmingState();
                SyncdDevice.ShutdownReason shutdown = impl.getShutdownReason();
                if (loop % 10 == 0 && arming == RobotArmingStateNotifier.ARMINGSTATE.ARMED) lastVolts = eh.getInputVoltage(VoltageUnit.VOLTS);
                boolean healthy = arming == RobotArmingStateNotifier.ARMINGSTATE.ARMED
                        && shutdown == SyncdDevice.ShutdownReason.NORMAL && !eh.isNotResponding()
                        && !(lastVolts < 9);
                if (healthy != wasHealthy) {
                    Map<String, Object> t = new LinkedHashMap<>();
                    t.put("ms", now);
                    t.put("healthy", healthy);
                    t.put("arming", arming.toString());
                    t.put("shutdown", shutdown.toString());
                    t.put("notResponding", eh.isNotResponding());
                    t.put("volts", lastVolts);
                    t.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
                    if (transitions.size() < 40) transitions.add(t);
                    wasHealthy = healthy;
                    if (healthy && atFault != null) {
                        backMs = now;
                        backToParallelMs = Double.NaN;
                        faultWindow = PhotonProbe.delta(atFault, PhotonProbe.counters());
                    }
                }
                if (!healthy) {
                    if (Double.isNaN(faultStartMs)) {
                        faultStartMs = now;
                        atFault = PhotonProbe.counters();
                        report.phase("expansion hub out");
                    }
                    lastUnhealthyMs = now;
                    recoveredMs = Double.NaN;
                } else if (!Double.isNaN(faultStartMs) && Double.isNaN(recoveredMs) && loop % 5 == 0) {
                    boolean all = true;
                    for (PhotonChannel c : ehChannels) all &= c.read() == c.expected() && c.enabled();
                    if (all) {
                        recoveredMs = now;
                        report.phase("expansion hub back");
                    }
                }
                if (loop % 20 == 0) for (PhotonChannel c : chChannels) chReadback.check("replug", loop, c, true);
                if (!Double.isNaN(recoveredMs) && now - lastUnhealthyMs >= HEALTHY_FOR_MS) break;
                loop++;
                report.status("%s: %.0f s, EH %s", mode, now / 1000, healthy ? "healthy" : "OUT");
                report.sendIfDue();
            }
        } finally {
            gap.halt();
        }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("faultSeen", !Double.isNaN(faultStartMs));
        r.put("faultStartMs", faultStartMs);
        r.put("lastUnhealthyMs", lastUnhealthyMs);
        r.put("recoveredMs", recoveredMs);
        r.put("recoveryMsAfterBack", recoveredMs - lastUnhealthyMs);
        r.put("maxWriteStallMs", gap.maxGapMs);
        r.put("maxSingleWriteMs", maxSingleWriteMs);
        r.put("parallelWriteMsAfterBack", backToParallelMs);
        r.put("loops", loop);
        r.put("transitions", transitions);
        r.put("countersDuringFault", faultWindow != null ? faultWindow
                : atFault == null ? null : PhotonProbe.delta(atFault, PhotonProbe.counters()));
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        r.put("controlHubReadback", chReadback.toMap());
        r.put("drainMs", rig.drain(1000));
        r.put("inFlightAtEnd", rig.inFlight());
        Map<String, Object> left = new LinkedHashMap<>();
        for (LynxModule h : rig.hubs) left.put(rig.key(h), PhotonProbe.leftovers(h));
        r.put("leftoversAtEnd", left);
        r.put("routingProblemsAfter", PhotonProbe.routingProblems(hardwareMap));
        PhotonCore.disable();
        rig.zeroMotors();
        rig.disableServos();
        report.put("result", r);
        report.done(mode + " pull, Photon " + PhotonWorkload.modeName(parallel) + ": fault " + r.get("faultSeen") + ", back at target " + r.get("recoveryMsAfterBack")
                + " ms after the hub returned, longest write stall " + gap.maxGapMs + " ms");
        report.holdUntilStop(this);
    }

    // Times only the write calls: the bench's own SDK reads of a vanished hub wait out their timeout.
    private static final class Gap extends Thread {
        private final Thread opModeThread;
        private volatile long enteredNs;
        private volatile boolean running = true;
        volatile double maxGapMs;

        Gap(Thread opModeThread) {
            super("PhotonReplugWatchdog");
            this.opModeThread = opModeThread;
            setDaemon(true);
        }

        void enter() {
            enteredNs = System.nanoTime();
        }

        void exit() {
            double g = (System.nanoTime() - enteredNs) / 1e6;
            if (g > maxGapMs) maxGapMs = g;
            enteredNs = 0;
        }

        @Override
        public void run() {
            while (running && opModeThread.isAlive()) {
                long e = enteredNs;
                if (e != 0) {
                    double g = (System.nanoTime() - e) / 1e6;
                    if (g > maxGapMs) maxGapMs = g;
                }
                LockSupport.parkNanos(2_000_000L);
            }
        }

        void halt() {
            running = false;
            PhotonBench.joinOrThrow(this, "replug watchdog");
        }
    }
}
