package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;

@TeleOp(name = "Bench: OctoQuad Absent", group = "Test")
public class OctoQuadAbsentBench extends EnhancedOpMode {
    private static final int SAVE_EVERY_LOOPS = 100;

    private final Samples faultedLoopMs = new Samples();
    private BenchReport report;
    private long createdNs;
    private long loops;
    private long lastLoopNs;
    private boolean faultSeen;

    @Override
    protected Robot createRobot() {
        report = new BenchReport("OctoQuad Absent", "octoquad_absent");
        OctoQuad octoQuad = BenchIO.require(hardwareMap, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
        report.put("chipId", octoQuad.getChipId() & 0xFF);
        report.phase("constructing");
        BenchIO.log("BENCH octoquad absent: constructing GuardedOctoQuadLocalizer");
        long t0 = System.nanoTime();
        Robot octoQuadRobot = new OctoQuadSetup.OctoQuadRobot(this);
        createdNs = System.nanoTime();
        report.put("createMs", (createdNs - t0) / 1e6);
        report.phase("init");
        report.save();
        return octoQuadRobot;
    }

    @Override
    protected void onLoopStart() {
        long now = System.nanoTime();
        if (faultSeen && lastLoopNs != 0) faultedLoopMs.add((now - lastLoopNs) / 1e6);
        lastLoopNs = now;
    }

    @Override
    protected void initializeLoop() {
        loops++;
        if (faultSeen) {
            if (loops % SAVE_EVERY_LOOPS == 0) {
                report.put("initLoopFaultedMs", faultedLoopMs.summary());
                report.save();
            }
            return;
        }
        for (Faults.Fault f : Faults.active()) {
            if (!f.key.equals(GuardedOctoQuadLocalizer.FAULT_KEY)) continue;
            faultSeen = true;
            double ms = (System.nanoTime() - createdNs) / 1e6;
            report.put("faultMs", ms);
            report.put("faultMessage", f.message);
            report.put("initLoopsBeforeFault", loops);
            report.status("FAULT %s: %s", f.key, f.message);
            report.save();
            BenchIO.log("BENCH octoquad absent: fault after %.0f ms: %s", ms, f.message);
        }
    }

    @Override
    protected void telemetry() {
        telemetry.addData(BenchReport.KEY_LOOP, loops);
        report.addTo(telemetry);
    }
}
