package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.CloseFlowerPaths;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The real EnhancedOpMode pipeline on a bare Control Hub: {@link BenchRobot} (Drivetrain module, Pedro Mecanum on
 * empty motor ports, software localizer), driven like BioBuzz Tele, through phases that each change one telemetry
 * or dashboard setting. Loop period is timed between consecutive onLoopStart() calls, so it includes the SDK's
 * between-loop work.
 */
@TeleOp(name = "Bench: Framework Loop", group = "Test")
public class FrameworkLoopBench extends EnhancedOpMode {

    private interface Setting {
        void apply(FrameworkLoopBench bench);
    }

    private static final class Phase {
        final String name;
        final String what;
        final Setting setting;

        Phase(String name, String what, Setting setting) {
            this.name = name;
            this.what = what;
            this.setting = setting;
        }
    }

    private BenchRobot bench;
    private final ToggleGuard toggles = new ToggleGuard();
    private BenchReport report;
    private final List<Phase> phases = new ArrayList<>();
    private int phaseIndex = -1;
    private double phaseSeconds;

    private long phaseStartNs;
    private long lastLoopStartNs;
    private boolean lastLoopWasDs;
    private long phaseLoops;
    private int dsFrames;
    private long loopCounter;
    private Samples periods;
    private Samples dsPeriods;
    private Samples otherPeriods;
    private BenchIO.Gc gcStart;
    private BenchIO.Cpu cpuStart;
    private ProfilerDiff profilerStart;
    private boolean following;

    @Override
    protected Robot createRobot() {
        bench = new BenchRobot(this);
        return bench;
    }

    @Override
    protected void initialize() {
        report = new BenchReport("Framework Loop", "framework_loop");
        phaseSeconds = BenchIO.param(report.params, "phaseSeconds", 8);

        Map<String, Object> info = new LinkedHashMap<>();
        info.put("dsMsTransmissionInterval", robot.telemetry.getMsTransmissionInterval());
        info.put("slothboardIntervalAtInit", FtcDashboard.getInstance().getTelemetryTransmissionInterval());
        info.put("togglesAtInit", ToggleGuard.current());
        info.put("phaseSeconds", phaseSeconds);
        info.put("followerDriveCap", BenchRobot.DRIVE_CAP);
        info.put("driverStationConnected", NetworkConnectionHandler.getInstance().isPeerConnected());
        report.put("info", info);

        phases.add(new Phase("integrated", "the settings the code ships with", b -> {}));
        phases.add(new Phase("dsEveryLoop", "OptimizationToggles.dsTransmissionIntervalMs = 0: every loop is a DS frame",
                b -> OptimizationToggles.dsTransmissionIntervalMs = 0));
        phases.add(new Phase("dashboardInterval0", "OptimizationToggles.dashboardTransmissionIntervalMs = 0",
                b -> OptimizationToggles.dashboardTransmissionIntervalMs = 0));
        phases.add(new Phase("dashboardInterval100", "OptimizationToggles.dashboardTransmissionIntervalMs = 100",
                b -> OptimizationToggles.dashboardTransmissionIntervalMs = 100));
        phases.add(new Phase("dashboardTelemetryOff", "telemetryToggles.dashboardTelemetry = false (no Bench keys reach the dashboard)",
                b -> Robot.telemetryToggles.dashboardTelemetry = false));
        phases.add(new Phase("dsTelemetryOff", "telemetryToggles.dsTelemetry = false",
                b -> Robot.telemetryToggles.dsTelemetry = false));
        phases.add(new Phase("loopProfileOff", "loopProfile and profilerEnabled false (no section breakdown)",
                b -> {
                    Robot.telemetryToggles.loopProfile = false;
                    OptimizationToggles.profilerEnabled = false;
                }));
        phases.add(new Phase("drivetrainEncoderTelemetry", "Drivetrain.encoderTelemetry = true",
                b -> Drivetrain.encoderTelemetry = true));
        phases.add(new Phase("drivetrainCurrentTelemetry", "Drivetrain.motorCurrentTelemetry = true (4 motor current commands per telemetry frame)",
                b -> Drivetrain.motorCurrentTelemetry = true));
        phases.add(new Phase("hubCurrentTelemetry", "telemetryToggles.current = true (a hub current command per loop)",
                b -> Robot.telemetryToggles.current = true));
        phases.add(new Phase("followingPath", "Pedro follows CloseFlowerPaths.toCloseFlower (pose never moves, so it never finishes); Drivetrain writes off",
                FrameworkLoopBench::startFollowing));
    }

    private void startFollowing() {
        CloseFlowerPaths paths = new CloseFlowerPaths();
        robot.follower.setPose(paths.start);
        bench.drivetrain.setWriteEnabled(false);
        robot.follower.follow(paths.toCloseFlower());
        following = true;
    }

    @Override
    protected void onStart() {
        beginPhase(0);
    }

    @Override
    protected void onLoopStart() {
        if (phaseIndex < 0 || report.isDone()) return;
        long now = System.nanoTime();
        if (lastLoopStartNs != 0) {
            double ms = (now - lastLoopStartNs) / 1e6;
            periods.add(ms);
            (lastLoopWasDs ? dsPeriods : otherPeriods).add(ms);
        }
        lastLoopStartNs = now;
        lastLoopWasDs = robot.telemetry.isDSFrame();
        if (lastLoopWasDs) dsFrames++;
        phaseLoops++;
    }

    @Override
    protected void gameLoop() {
        loopCounter++;
        if (!following) bench.drivetrain.setMecanumTargets(0.2, 0.1, 0.05, false);
        if (phaseIndex < 0 || report.isDone()) return;
        double elapsed = (System.nanoTime() - phaseStartNs) / 1e9;
        report.status("%s %.0f/%.0fs", phases.get(phaseIndex).name, elapsed, phaseSeconds);
        if (elapsed < phaseSeconds) return;
        endPhase();
        if (phaseIndex + 1 < phases.size()) beginPhase(phaseIndex + 1);
        else report.done(phases.size() + " phases of " + phaseSeconds + " s");
    }

    private void beginPhase(int index) {
        phaseIndex = index;
        Phase p = phases.get(index);
        p.setting.apply(this);
        report.phase(p.name);
        periods = new Samples();
        dsPeriods = new Samples();
        otherPeriods = new Samples();
        phaseLoops = 0;
        dsFrames = 0;
        lastLoopStartNs = 0;
        gcStart = BenchIO.Gc.now();
        cpuStart = BenchIO.Cpu.now();
        profilerStart = new ProfilerDiff(getProfiler());
        phaseStartNs = System.nanoTime();
    }

    private void endPhase() {
        Phase p = phases.get(phaseIndex);
        double seconds = (System.nanoTime() - phaseStartNs) / 1e9;
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("what", p.what);
        r.put("settings", ToggleGuard.current());
        r.put("dsMsTransmissionInterval", robot.telemetry.getMsTransmissionInterval());
        r.put("loops", phaseLoops);
        r.put("loopsPerSecond", phaseLoops / seconds);
        r.put("loopPeriodMs", periods.summary());
        r.put("loopPeriodHistogram", periods.histogram(0.5, 80));
        r.put("dsFrames", dsFrames);
        r.put("dsFramesPerSecond", dsFrames / seconds);
        r.put("dsFrameLoopPeriodMs", dsPeriods.summary());
        r.put("otherLoopPeriodMs", otherPeriods.summary());
        r.put("sections", profilerStart.since(getProfiler()));
        r.put("gc", BenchIO.Gc.now().since(gcStart, phaseLoops));
        r.put("cpu", BenchIO.Cpu.now().since(cpuStart));
        r.put("slothboardInterval", FtcDashboard.getInstance().getTelemetryTransmissionInterval());
        report.endPhase(p.name, r);

        toggles.restore();
        if (following) {
            robot.follower.stop();
            bench.drivetrain.setWriteEnabled(true);
            following = false;
        }
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData("Bench loop", loopCounter);
    }
}
