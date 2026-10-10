package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.octoquad.OctoQuadLocalizerTest;

@TeleOp(name = "Bench: OctoQuad Absent", group = "Test")
public class OctoQuadAbsentBench extends EnhancedOpMode {
    private BenchReport report;

    @Override
    protected Robot createRobot() {
        report = new BenchReport("OctoQuad Absent", "octoquad_absent");
        OctoQuad octoQuad = BenchIO.require(hardwareMap, OctoQuad.class, OctoQuadLocalizerTest.name, BenchIO.RUNNER_HINT);
        report.put("chipId", octoQuad.getChipId() & 0xFF);
        report.put("constructing", true);
        report.save();
        report.phase("constructing");
        BenchIO.log("BENCH octoquad absent: constructing OctoQuadLocalizer");
        long t0 = System.nanoTime();
        Robot octoQuadRobot = new OctoQuadSetup.OctoQuadRobot(this);
        report.put("returned", true);
        report.put("createMs", (System.nanoTime() - t0) / 1e6);
        report.put("statusAfterCreate", octoQuad.getLocalizerStatus().name());
        report.done("OctoQuadLocalizer returned with nothing on the bus");
        return octoQuadRobot;
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
    }
}
