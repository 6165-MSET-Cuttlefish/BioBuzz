package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.Map;
import java.util.concurrent.locks.LockSupport;

@Photon
@TeleOp(name = "Bench: Photon Stop Linear", group = "Test")
public class PhotonStopLinearBench extends LinearOpMode {
    private static final long WRITE_AFTER_STOP_NS = 200_000_000L;
    private static final long INTERRUPT_WAIT_NS = 50_000_000L;

    @Override
    public void runOpMode() {
        String trial = BenchIO.param(BenchIO.params(), "trial", "0");
        BenchReport report = new BenchReport("Photon Stop Linear", "photon_stop_main_" + trial);
        int parallel = (int) BenchIO.param(report.params, "photon", 8);
        int control = (int) BenchIO.param(report.params, "control", PhotonStopBench.CONTROL_NONE);
        if (control != PhotonStopBench.CONTROL_NONE && control != PhotonStopBench.CONTROL_ONCE) {
            throw new IllegalArgumentException("control=" + control + ": the linear STOP bench has no held control");
        }
        PhotonRig rig = new PhotonRig(hardwareMap, report.params, "Bench: Photon Stop Linear");
        Map<String, Object> info = rig.info();
        info.put("variant", "linear");
        info.put("photon", parallel);
        info.put("negativeControl", control != PhotonStopBench.CONTROL_NONE);
        info.put("controlVariant", control == PhotonStopBench.CONTROL_ONCE ? "once" : "none");
        info.put("trial", trial);
        report.put("info", info);
        report.save();
        StopSampler sampler = new StopSampler("photon_stop_" + trial, rig, Thread.currentThread(), info);
        Map<String, Object> countersBefore = PhotonProbe.counters();
        if (!report.holdUntilStart(this)) return;
        if (parallel > 0) {
            PhotonCore.setMaxParallelWrites(parallel);
            PhotonCore.enable();
        } else {
            PhotonCore.disable();
        }
        report.phase("writing");
        sampler.start();
        long loops = 0;
        while (opModeIsActive()) {
            sampler.noteLoop();
            rig.writeAll();
            loops++;
            report.status("loop %d", loops);
            report.sendIfDue();
        }
        long exitNs = System.nanoTime();
        while (!Thread.currentThread().isInterrupted() && System.nanoTime() - exitNs < INTERRUPT_WAIT_NS) Thread.yield();
        long interruptNs = System.nanoTime();
        // LinearOpMode interrupts this thread at STOP; cleared so the late writes reach the STOP gates and the reads below run.
        boolean interrupted = Thread.interrupted();
        sampler.put("interruptedAtStop", interrupted);
        sampler.put("msLoopExitToInterrupt", interrupted ? (interruptNs - exitNs) / 1e6 : null);
        long late = 0;
        long t0 = System.nanoTime();
        while (System.nanoTime() - t0 < WRITE_AFTER_STOP_NS) {
            rig.writeAll();
            late++;
            LockSupport.parkNanos(5_000_000L);
        }
        Map<String, Object> once = control == PhotonStopBench.CONTROL_ONCE ? StopGateViolation.reEnergizeOnce(rig) : null;
        sampler.put("stopSawBeforeZeroing", rig.outputs());
        sampler.put("countersAtStop", PhotonProbe.delta(countersBefore, PhotonProbe.counters()));
        sampler.put("loops", loops);
        sampler.put("passesAfterStop", late);
        sampler.put("photonEnabledAfterStop", PhotonCore.isEnabled());
        if (once != null) sampler.put("negativeControl", once);
        rig.zeroMotors();
    }
}
