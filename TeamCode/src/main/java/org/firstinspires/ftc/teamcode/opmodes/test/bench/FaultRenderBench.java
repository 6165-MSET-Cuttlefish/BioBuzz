package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Faults for a person to judge on the Driver Station. Mode phases: the runner saves {@link #cue} through the dashboard
 * to move it through short, long, two, slow and off. Mode firstFrame: after the first Driver Station frame of INIT it
 * turns Driver Station telemetry off, so that frame stays on the screen.
 */
@Config("Bench: Fault Render")
@TeleOp(name = "Bench: Fault Render", group = "Test")
public class FaultRenderBench extends EnhancedOpMode {
    public static int cue = 0;

    static final String[] PHASES = {"waiting", "short", "long", "two", "slow", "off"};
    static final String KEY = "Bench";
    static final String SECOND_KEY = "Second";
    static final String HIDDEN_KEY = "Hidden";
    static final int LONG_CHARS = 150;
    static final int SLOW_INTERVAL_MS = 1000;
    static final int NORMAL_LINES = 10;

    public static final class RenderStub extends Module {
        enum StubState implements State {
            READY
        }

        private final FaultRenderBench bench;

        RenderStub(FaultRenderBench bench) {
            this.bench = bench;
        }

        @Override
        protected void initStates() {
            setStates(StubState.READY);
        }

        @Override protected void read() {}
        @Override protected void write() {}

        @Override
        public void stop() {
            bench.restoreToggles();
        }
    }

    public static class RenderRobot extends Robot {
        public RenderStub stub;

        public RenderRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {
            stub = new RenderStub((FaultRenderBench) opMode);
        }
    }

    private RenderRobot bench;
    private BenchReport report;
    private String mode;
    private boolean savedDsTelemetry;
    private int savedDsIntervalMs;
    private boolean restored;
    private boolean running;
    private int phase = -1;
    private long phaseStartNs;
    private long loops;
    private long dsFrames;
    private boolean firstFrameHeld;

    static String longMessage() {
        StringBuilder m = new StringBuilder("this message is " + LONG_CHARS + " characters long so you can check it wraps and "
                + "stays readable on the Driver Station");
        while (m.length() < LONG_CHARS - 4) m.append('.');
        return m.append(" END").toString();
    }

    @Override
    protected Robot createRobot() {
        mode = BenchIO.param(BenchIO.params(), "mode", "phases");
        if (!mode.equals("phases") && !mode.equals("firstFrame")) {
            throw new IllegalArgumentException("bench param mode must be phases or firstFrame, was " + mode);
        }
        report = new BenchReport("Fault Render", "fault_render_" + mode);
        savedDsTelemetry = OptimizationToggles.dsTelemetry;
        savedDsIntervalMs = OptimizationToggles.dsTransmissionIntervalMs;
        cue = 0;
        bench = new RenderRobot(this);
        return bench;
    }

    @Override
    protected void initialize() {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("mode", mode);
        info.put("dsTelemetryAtInit", savedDsTelemetry);
        info.put("dsTransmissionIntervalMsAtInit", savedDsIntervalMs);
        info.put("longMessage", longMessage());
        report.put("info", info);
        if (mode.equals("phases")) startPhase(0);
        else report.phase("first frame");
    }

    @Override
    protected void onLoopStart() {
        loops++;
        if (!mode.equals("phases")) return;
        int wanted = cue;
        if (wanted != phase && wanted >= 0 && wanted < PHASES.length) {
            endPhase();
            startPhase(wanted);
        }
        if (PHASES[phase].equals("slow")) {
            Faults.raise(KEY, "updates once a second: " + (System.nanoTime() - phaseStartNs) / 1_000_000_000L + " s");
        }
    }

    @Override
    protected void onStart() {
        running = true;
    }

    private void startPhase(int index) {
        phase = index;
        phaseStartNs = System.nanoTime();
        loops = 0;
        dsFrames = 0;
        Faults.clear(KEY);
        Faults.clear(SECOND_KEY);
        Faults.clear(HIDDEN_KEY);
        OptimizationToggles.dsTelemetry = savedDsTelemetry;
        OptimizationToggles.dsTransmissionIntervalMs = savedDsIntervalMs;
        switch (PHASES[index]) {
            case "short":
                Faults.raise(KEY, "a short test message");
                break;
            case "long":
                Faults.raise(KEY, longMessage());
                break;
            case "two":
                Faults.raise(KEY, "first of two faults");
                Faults.raise(SECOND_KEY, "second of two faults");
                break;
            case "slow":
                OptimizationToggles.dsTransmissionIntervalMs = SLOW_INTERVAL_MS;
                Faults.raise(KEY, "updates once a second: 0 s");
                break;
            case "off":
                OptimizationToggles.dsTelemetry = false;
                Faults.raise(HIDDEN_KEY, "this line must not show on the Driver Station");
                break;
            default:
                break;
        }
        report.phase(PHASES[index]);
        BenchIO.log("fault render phase %s", PHASES[index]);
    }

    private void endPhase() {
        if (phase < 0) return;
        double seconds = (System.nanoTime() - phaseStartNs) / 1e9;
        List<String> keys = new ArrayList<>();
        int htmlChars = 0;
        for (Faults.Fault f : Faults.active()) {
            keys.add(f.key);
            htmlChars += HtmlFormatter.faultLine(f.key, f.message).length();
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("seconds", seconds);
        m.put("loops", loops);
        m.put("dsFramesOpened", dsFrames);
        m.put("dsFramesPerSecond", seconds > 0 ? dsFrames / seconds : 0);
        m.put("faults", keys);
        m.put("faultLineHtmlChars", htmlChars);
        m.put("dsTelemetry", OptimizationToggles.dsTelemetry);
        m.put("dsTransmissionIntervalMs", OptimizationToggles.dsTransmissionIntervalMs);
        report.endPhase(PHASES[phase], m);
    }

    @Override
    protected void telemetry() {
        if (bench.telemetry.isDSFrame()) {
            dsFrames++;
            if (mode.equals("firstFrame") && !firstFrameHeld) {
                firstFrameHeld = true;
                OptimizationToggles.dsTelemetry = false;
                report.put("firstFrameLoop", loops);
                report.done("first Driver Station frame held");
            }
        }
        if (mode.equals("phases") && PHASES[phase].equals("two")) {
            for (int i = 1; i <= NORMAL_LINES; i++) telemetry.addData("Line " + i, "normal text");
        }
        report.addTo(telemetry);
        // The runner times the phases and ends with STOP, so it needs BENCH DONE as soon as it runs.
        if (running && !report.isDone()) telemetry.addData(BenchReport.KEY_DONE, "phases: recording until STOP");
    }

    void restoreToggles() {
        if (restored) return;
        restored = true;
        OptimizationToggles.dsTelemetry = savedDsTelemetry;
        OptimizationToggles.dsTransmissionIntervalMs = savedDsIntervalMs;
    }

    @Override
    protected void onEnd() {
        if (mode.equals("phases")) endPhase();
        else report.put("dsFramesOpened", dsFrames);
        restoreToggles();
        report.save();
    }
}
