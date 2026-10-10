package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * The framework's INIT path (runInit through initialize) and nothing else, for run_bench.py's Sloth race: it
 * INITs this right after a deploySloth and checks whether the app restarted. The class loader shows whether the
 * hot-loaded code or the installed code ran.
 */
@TeleOp(name = "Bench: Sloth Probe", group = "Test")
public class SlothProbeBench extends EnhancedOpMode {
    private BenchReport report;

    @Override
    protected Robot createRobot() {
        return new BenchRobot(this);
    }

    @Override
    protected void initialize() {
        String runId = BenchIO.param(BenchIO.params(), "runId", "manual");
        report = new BenchReport("Sloth Probe", "sloth_probe_" + runId);
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("classLoader", String.valueOf(getClass().getClassLoader()));
        m.put("classIdentity", System.identityHashCode(getClass()));
        report.put("loaded", m);
        report.done("INIT reached initialize() in pid " + android.os.Process.myPid());
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
    }
}
