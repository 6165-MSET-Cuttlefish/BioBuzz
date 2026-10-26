package org.firstinspires.ftc.teamcode.pedro.ivy;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import android.os.Debug;
import android.os.SystemClock;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicBoolean;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.PathHeadings;
import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.architecture.command.StateCommands;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.modules.BettaShooter;
import org.firstinspires.ftc.teamcode.biobuzz.modules.BettaTurret;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.BallCollectionRobot;

@Autonomous(name = "Ivy Path Test", group = "Test")
public class IvyPathTest extends EnhancedOpMode {

    private static final double ROUTINE_TIMEOUT_MS = 10 * 60 * 1000;
    private static final double PAUSE_MS = 2000;
    private static final File LOOP_LOG = new File(AppUtil.FIRST_FOLDER, "looptimes/IvyPathTest.csv");

    private final Pose start = new Pose(48, 48, 0);
    private final Pose control1 = new Pose(72, 48);
    private final Pose control2 = new Pose(96, 72);
    private final Pose end = new Pose(96, 96);


    private BettaTurret turret;
    private BettaShooter shooter;
    private Command routine;
    private double[][] splineDrawing;
    private double[] halfwayMark;
    private String leg = "waiting for START";
    private double halfwayFiredAt = Double.NaN;
    private Writer loopLog;
    private final StringBuilder row = new StringBuilder(512);
    private String phase = "init";
    private long phaseStartNs;
    private long lastLoopStartNs;
    private long runLoops;
    private double runSumMs;
    private double runMaxMs;
    private long gcCountAtStart;
    private long gcTimeMsAtStart;



    @Override
    protected Robot createRobot() {
        turret = new BettaTurret(hardwareMap);
        shooter = new BettaShooter(hardwareMap);

        return new BallCollectionRobot(this);
    }

    @Override
    protected void initialize() {
        robot.follower.setPose(start);
        openLoopLog();

        Path forward = curve(start, control1, control2, end).tangent();
        Path back = PathHeadings.backwardTangent(curve(end, control2, control1, start));
        splineDrawing = FieldVisualization.pathPolyline(forward);
        Vector2D middle = forward.getSegments().get(0).curve.get(0.5);
        halfwayMark = FieldVisualization.toField(middle.x(), middle.y());

        Command aimSideways = StateCommands.set(BettaTurret.RightState.SIDEWAYS);
        Command aimForward = StateCommands.set(BettaTurret.RightState.FORWARD);
        Command startShooter = StateCommands.set(BettaShooter.ShooterState.ON);
        Command stopShooter = StateCommands.set(BettaShooter.ShooterState.OFF);
        Command hoodUp = StateCommands.set(BettaShooter.HoodState.UP);
        Command hoodDown = StateCommands.set(BettaShooter.HoodState.DOWN);
        Command unblock = StateCommands.set(BettaShooter.HardstopState.UNBLOCKED);
        Command block = StateCommands.set(BettaShooter.HardstopState.BLOCKED);

        routine = sequential(PathCommands.timeout(Groups.loop(sequential(
                instant(() -> leg = "forward"),
                parallel(PathCommands.follow(robot.follower, forward), sequential(halfway(), recordHalfway(), aimSideways, startShooter), hoodUp, unblock),
                instant(() -> leg = "pause"),
                PathCommands.waitMs(PAUSE_MS),
                instant(() -> leg = "back"),
                parallel(PathCommands.follow(robot.follower, back), sequential(halfway(), recordHalfway(), aimForward, stopShooter), hoodDown, block),
                instant(() -> leg = "pause"),
                PathCommands.waitMs(PAUSE_MS)
        )), ROUTINE_TIMEOUT_MS),
                instant(() -> leg = "timed out"));
    } 

    // completion() reads 1 until Pedro updates on the new path, so wait to see it under half first.
    private Command halfway() {
        AtomicBoolean armed = new AtomicBoolean();
        return Command.build()
                .setStart(() -> armed.set(false))
                .setExecute(() -> {
                    if (robot.follower.completion() < 0.5) armed.set(true);
                })
                .setDone(() -> armed.get() && robot.follower.completion() >= 0.5);
    }

    private Command recordHalfway() {
        return instant(() -> halfwayFiredAt = robot.follower.completion());
    }

    @Override
    protected void onStart() {
        logLine("# start elapsedRealtimeMs=" + SystemClock.elapsedRealtime());
        phase = "run";
        phaseStartNs = System.nanoTime();
        lastLoopStartNs = 0;
        gcCountAtStart = gcStat("art.gc.gc-count");
        gcTimeMsAtStart = gcStat("art.gc.gc-time");
        Scheduler.schedule(routine);
    }

    // Each row's period ends at this loop's start, so its sections are the previous loop's, all finished by now.
    @Override
    protected void onLoopStart() {
        long now = System.nanoTime();
        if (lastLoopStartNs != 0) {
            double periodMs = (now - lastLoopStartNs) / 1e6;
            if (phase.equals("run")) {
                runLoops++;
                runSumMs += periodMs;
                runMaxMs = Math.max(runMaxMs, periodMs);
            }
            row.setLength(0);
            row.append(phase).append(',')
                    .append((lastLoopStartNs - phaseStartNs) / 1000).append(',')
                    .append(Math.round(periodMs * 1000)).append(',')
                    .append(leg).append(',')
                    .append(Math.round(robot.follower.completion() * 1000) / 1000.0).append(',')
                    .append(robot.follower.isBusy()).append(',');
            getProfiler().appendLastMicros(row);
            logLine(row);
        }
        lastLoopStartNs = now;
    }

    @Override
    protected void onEnd() {
        double avg = runLoops == 0 ? 0 : runSumMs / runLoops;
        logLine("# summary " + getProfiler().report(runLoops, avg, runMaxMs)
                + " gcCount=" + (gcStat("art.gc.gc-count") - gcCountAtStart)
                + " gcTimeMs=" + (gcStat("art.gc.gc-time") - gcTimeMsAtStart));
        try {
            loopLog.close();
        } catch (IOException e) {
            throw new IllegalStateException("can't close " + LOOP_LOG, e);
        }
    }

    private void openLoopLog() {
        File dir = LOOP_LOG.getParentFile();
        if (!dir.isDirectory() && !dir.mkdirs()) throw new IllegalStateException("can't create " + dir);
        try {
            loopLog = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(LOOP_LOG, true), StandardCharsets.UTF_8), 1 << 16);
        } catch (IOException e) {
            throw new IllegalStateException("can't open " + LOOP_LOG, e);
        }
        phaseStartNs = System.nanoTime();
        logLine("# run wallClockMs=" + System.currentTimeMillis() + " elapsedRealtimeMs=" + SystemClock.elapsedRealtime());
        logLine("phase,tUs,periodUs,leg,completion,busy,sectionsUs");
    }

    private void logLine(CharSequence line) {
        try {
            loopLog.append(line).append('\n');
        } catch (IOException e) {
            throw new IllegalStateException("can't write " + LOOP_LOG, e);
        }
    }

    private static long gcStat(String key) {
        String v = Debug.getRuntimeStat(key);
        return v == null ? -1 : Long.parseLong(v.trim());
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Running", Scheduler.isRunning(routine));
        telemetry.addData("Leg", leg);
        telemetry.addData("Path Completion", "%.2f", robot.follower.completion());
        telemetry.addData("Halfway Fired At Completion", "%.2f", halfwayFiredAt);
        BettaTurret.RightState aim = turret.getState(BettaTurret.RightState.class);
        telemetry.addData("Turret Right Position", aim == BettaTurret.RightState.IDLE
                ? "unpowered" : String.format("%.2f (%s)", aim.getValue(), aim));
        telemetry.addData("Shooter Power", "%.2f", shooter.getState(BettaShooter.ShooterState.class).getValue());
        telemetry.addData("Hood", shooter.getState(BettaShooter.HoodState.class));
        telemetry.addData("Hardstop", shooter.getState(BettaShooter.HardstopState.class));
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        overlay.setStroke("#2962FF");
        overlay.strokePolyline(splineDrawing[0], splineDrawing[1]);
        overlay.setStroke("#FFD600");
        overlay.strokeCircle(halfwayMark[0], halfwayMark[1], 2);
        // Null until the follower first computes a path: the loop START schedules the first leg draws before that.
        Pose target = robot.follower.closestPose();
        if (robot.follower.isBusy() && target != null) {
            double[] p = FieldVisualization.toField(target.x(), target.y());
            overlay.setFill("#2962FF");
            overlay.fillCircle(p[0], p[1], 1.5);
        }
    }
}
