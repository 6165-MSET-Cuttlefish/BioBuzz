package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.CloseFlowerPaths;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Drive Handover", group = "Test")
public class DriveHandoverBench extends EnhancedOpMode {
    static final double STICK = 0.3;
    private static final double CASE_S = 3;
    private static final double TAKE_S = 0.5;
    private static final double RETURN_S = 1.5;
    private static final double MATCH = 0.01;
    private static final int LOGGED_SAMPLES = 12;
    private static final String[] CASES = {"C", "S", "E", "D"};
    private static final String[] WHAT = {
            "stick 0; from 0.5 s Pedro follows with Drivetrain writes off; at 1.5 s gameLoop cancels the path, turns "
                    + "writes on and holds the stick",
            "stick held; at 0.5 s writes off and sequential(timeout(follow, 1000 ms), stop, writes on)",
            "stick held; a collection-style command (writes off and follow at start; stop, zero targets, "
                    + "Drivetrain.stop() and writes on at end) cancelled from gameLoop at 1.5 s before the stick is set",
            "control: the same command ending itself at 1.5 s inside the scheduler, after gameLoop set the stick"};

    private BenchRobot bench;
    private BenchReport report;
    private DcMotorEx fl;
    private Path path;
    private int caseIndex = -1;
    private long caseStartNs;
    private boolean taken;
    private boolean returned;
    private Command command;
    private double stick;
    private boolean sawDisabled;
    private int sampleIndex;
    private int reenableIndex;
    private int fromSecond;
    private int mismatched;
    private double[] firstMismatch;
    private List<double[]> logged;

    @Override
    protected Robot createRobot() {
        bench = new BenchRobot(this);
        return bench;
    }

    @Override
    protected void initialize() {
        report = new BenchReport("Drive Handover", "drive_handover");
        fl = BenchIO.require(hardwareMap, DcMotorEx.class, "fl", BenchIO.RUNNER_HINT);
        CloseFlowerPaths paths = new CloseFlowerPaths();
        robot.follower.setPose(paths.start);
        path = paths.toCloseFlower();
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("stick", STICK);
        info.put("followerDriveCap", BenchRobot.DRIVE_CAP);
        info.put("caseSeconds", CASE_S);
        info.put("sampleColumns", new String[] {"loopsAfterReenable", "flPower", "commanded", "followerIdle"});
        report.put("info", info);
    }

    @Override
    protected void onStart() {
        beginCase(0);
    }

    @Override
    protected void onLoopStart() {
        if (caseIndex >= 0 && !report.isDone()) sample();
    }

    @Override
    protected void gameLoop() {
        if (caseIndex < 0 || report.isDone()) return;
        String c = CASES[caseIndex];
        double t = caseSeconds();
        if (!taken && t >= TAKE_S) {
            taken = true;
            take(c);
        }
        if (taken && !returned && t >= RETURN_S && (c.equals("C") || c.equals("E"))) {
            returned = true;
            Scheduler.cancel(command);
            if (c.equals("C")) bench.drivetrain.setWriteEnabled(true);
            stick = STICK;
        }
        bench.drivetrain.setMecanumTargets(stick, 0, 0, false);
        if (t < CASE_S) return;
        endCase();
        if (caseIndex + 1 < CASES.length) beginCase(caseIndex + 1);
        else finish();
    }

    private double caseSeconds() {
        return (System.nanoTime() - caseStartNs) / 1e9;
    }

    private void take(String c) {
        if (c.equals("C")) {
            bench.drivetrain.setWriteEnabled(false);
            command = PathCommands.follow(robot.follower, path);
        } else if (c.equals("S")) {
            bench.drivetrain.setWriteEnabled(false);
            command = Groups.sequential(
                    PathCommands.timeout(PathCommands.follow(robot.follower, path), (RETURN_S - TAKE_S) * 1000),
                    PathCommands.stop(robot.follower),
                    Commands.instant(() -> bench.drivetrain.setWriteEnabled(true)));
        } else {
            command = collection(c.equals("D"));
        }
        Scheduler.schedule(command);
    }

    private Command collection(boolean endsItself) {
        return Command.build()
                .setStart(() -> {
                    bench.drivetrain.setWriteEnabled(false);
                    robot.follower.follow(path);
                })
                .setDone(() -> endsItself && caseSeconds() >= RETURN_S)
                .setEnd(end -> {
                    robot.follower.stop();
                    bench.drivetrain.setTargets(0, 0, 0, 0);
                    bench.drivetrain.stop();
                    bench.drivetrain.setWriteEnabled(true);
                })
                .requiring(robot.follower, bench.drivetrain);
    }

    private void sample() {
        boolean enabled = bench.drivetrain.isWriteEnabled();
        double sdk = fl.getPower();
        double commanded = bench.drivetrain.getMotorPowers()[0];
        if (!enabled) sawDisabled = true;
        else if (sawDisabled && reenableIndex < 0) reenableIndex = sampleIndex;
        if (reenableIndex >= 0) {
            int after = sampleIndex - reenableIndex;
            double[] row = {after, sdk, commanded, robot.follower.idle() ? 1 : 0};
            if (logged.size() < LOGGED_SAMPLES) logged.add(row);
            if (after >= 2) {
                fromSecond++;
                if (Math.abs(sdk - commanded) >= MATCH) {
                    mismatched++;
                    if (firstMismatch == null) firstMismatch = row;
                }
            }
        }
        sampleIndex++;
    }

    private void beginCase(int index) {
        if (command != null) Scheduler.cancel(command);
        command = null;
        robot.follower.stop();
        robot.follower.drivetrain.stop();
        bench.drivetrain.stop();
        bench.drivetrain.setWriteEnabled(true);
        caseIndex = index;
        stick = CASES[index].equals("C") ? 0 : STICK;
        taken = false;
        returned = false;
        sawDisabled = false;
        sampleIndex = 0;
        reenableIndex = -1;
        fromSecond = 0;
        mismatched = 0;
        firstMismatch = null;
        logged = new ArrayList<>();
        caseStartNs = System.nanoTime();
        report.phase(CASES[index]);
    }

    private void endCase() {
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("what", WHAT[caseIndex]);
        r.put("samples", sampleIndex);
        r.put("reenableSample", reenableIndex < 0 ? null : reenableIndex);
        r.put("samplesFromSecondAfterReenable", fromSecond);
        r.put("mismatchedFromSecondAfterReenable", mismatched);
        r.put("firstMismatch", firstMismatch);
        r.put("afterReenable", logged);
        report.endPhase(CASES[caseIndex], r);
    }

    private void finish() {
        stick = 0;
        bench.drivetrain.setTargets(0, 0, 0, 0);
        bench.drivetrain.stop();
        report.done(CASES.length + " hand-back cases of " + CASE_S + " s");
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
    }
}
