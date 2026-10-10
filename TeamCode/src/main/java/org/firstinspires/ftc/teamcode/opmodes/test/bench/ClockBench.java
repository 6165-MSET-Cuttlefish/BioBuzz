package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Bench: Clock", group = "Test")
public class ClockBench extends EnhancedOpMode {
    private static final double JUMP_MS = 1000;
    private static final double AFTER_MS = 1000;
    private static final double GIVE_UP_MS = 5000;

    private BenchReport report;
    private double timeoutMs;
    private Command timeout;
    private long initNs;
    private long startNs;
    private long timeoutEndedNs;
    private double firstOffsetMs = Double.NaN;
    private double lastOffsetMs = Double.NaN;
    private double biggestStepMs;
    private final List<double[]> jumps = new ArrayList<>();
    private long loops;
    private boolean finished;

    @Override
    protected Robot createRobot() {
        return new BenchRobot(this);
    }

    @Override
    protected void initialize() {
        report = new BenchReport("Clock", "clock");
        timeoutMs = BenchIO.param(report.params, "timeoutMs", 15000);
        timeout = Groups.sequential(PathCommands.timeout(Commands.waitUntil(() -> false), timeoutMs),
                Commands.instant(() -> timeoutEndedNs = System.nanoTime()));
        initNs = System.nanoTime();
        report.put("timeoutMs", timeoutMs);
        report.put("wallMsAtInit", System.currentTimeMillis());
        report.put("jumps", jumps);
        report.phase("init");
    }

    @Override
    protected void onLoopStart() {
        double offset = System.currentTimeMillis() - System.nanoTime() / 1e6;
        if (Double.isNaN(firstOffsetMs)) firstOffsetMs = offset;
        if (!Double.isNaN(lastOffsetMs)) {
            double step = offset - lastOffsetMs;
            if (Math.abs(step) > Math.abs(biggestStepMs)) biggestStepMs = step;
            if (Math.abs(step) >= JUMP_MS) {
                double at = (System.nanoTime() - initNs) / 1e6;
                jumps.add(new double[] {at, step});
                BenchIO.log("clock: the wall clock moved %.0f ms against System.nanoTime() %.0f ms after INIT", step, at);
            }
        }
        lastOffsetMs = offset;
    }

    @Override
    protected void onStart() {
        startNs = System.nanoTime();
        Scheduler.schedule(timeout);
        report.phase("timeout");
    }

    @Override
    protected void gameLoop() {
        loops++;
        if (finished) return;
        double elapsedMs = (System.nanoTime() - startNs) / 1e6;
        report.status("%.1f s", elapsedMs / 1000);
        boolean ended = timeoutEndedNs != 0;
        if ((ended && elapsedMs >= (timeoutEndedNs - startNs) / 1e6 + AFTER_MS) || elapsedMs >= timeoutMs + GIVE_UP_MS) {
            finish();
        }
    }

    private void finish() {
        finished = true;
        Double endedAfterMs = timeoutEndedNs == 0 ? null : (timeoutEndedNs - startNs) / 1e6;
        report.put("timeoutEndedAfterMs", endedAfterMs);
        report.put("wallMsAtEnd", System.currentTimeMillis());
        report.put("wallMinusNanoTimeChangeMs", lastOffsetMs - firstOffsetMs);
        report.put("biggestLoopStepMs", biggestStepMs);
        report.put("loops", loops);
        report.done(endedAfterMs == null ? "PathCommands.timeout never ended"
                : String.format("PathCommands.timeout(%.0f ms) ended %.0f ms after START", timeoutMs, endedAfterMs));
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData("Bench loop", loops);
    }
}
