package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.Map;

@Photon
@TeleOp(name = "Bench: Photon Stop", group = "Test")
public class PhotonStopBench extends OpMode {
    static final int CONTROL_NONE = 0;
    static final int CONTROL_HELD = 1;
    static final int CONTROL_ONCE = 2;

    private PhotonRig rig;
    private StopSampler sampler;
    private BenchReport report;
    private int parallel;
    private int control;
    private long loopCount;
    private Map<String, Object> countersBefore;

    @Override
    public void init() {
        String trial = BenchIO.param(BenchIO.params(), "trial", "0");
        report = new BenchReport("Photon Stop", "photon_stop_main_" + trial);
        parallel = (int) BenchIO.param(report.params, "photon", 8);
        control = (int) BenchIO.param(report.params, "control", CONTROL_NONE);
        if (control < CONTROL_NONE || control > CONTROL_ONCE) throw new IllegalArgumentException("control=" + control);
        rig = new PhotonRig(hardwareMap, report.params, "Bench: Photon Stop");
        Map<String, Object> info = rig.info();
        info.put("variant", "iterative");
        info.put("photon", parallel);
        info.put("negativeControl", control != CONTROL_NONE);
        info.put("controlVariant", control == CONTROL_HELD ? "held" : control == CONTROL_ONCE ? "once" : "none");
        info.put("trial", trial);
        report.put("info", info);
        report.save();
        sampler = new StopSampler("photon_stop_" + trial, rig, Thread.currentThread(), info);
        countersBefore = PhotonProbe.counters();
    }

    @Override
    public void init_loop() {
        report.sendIfDue();
    }

    @Override
    public void start() {
        if (parallel > 0) {
            PhotonCore.setMaxParallelWrites(parallel);
            PhotonCore.enable();
        } else {
            PhotonCore.disable();
        }
        countersBefore = PhotonProbe.counters();
        report.phase("writing");
        sampler.start();
    }

    @Override
    public void loop() {
        sampler.noteLoop();
        rig.writeAll();
        loopCount++;
        report.status("loop %d", loopCount);
        report.sendIfDue();
    }

    // The SDK fail-safes every hub before stop(), and loop() has returned, so this read follows every late write.
    @Override
    public void stop() {
        Map<String, Object> once = control == CONTROL_ONCE ? StopGateViolation.reEnergizeOnce(rig) : null;
        sampler.put("stopSawBeforeZeroing", rig.outputs());
        sampler.put("countersAtStop", PhotonProbe.delta(countersBefore, PhotonProbe.counters()));
        sampler.put("loops", loopCount);
        if (control == CONTROL_HELD) sampler.put("negativeControl", StopGateViolation.reEnergize(rig, sampler));
        if (once != null) sampler.put("negativeControl", once);
        rig.zeroMotors();
    }
}
