package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.CloseFlowerPaths;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Close Flower Linear Auto's loop (follower.update, Scheduler.execute, its SDK telemetry lines) with the
 * software-localizer follower, idle and then following CloseFlowerPaths.toCloseFlower: the no-framework baseline for Bench: Framework Loop.
 */
@TeleOp(name = "Bench: Raw Loop", group = "Test")
public class RawLoopBench extends LinearOpMode {

    @Override
    public void runOpMode() {
        BenchReport report = new BenchReport("Raw Loop", "raw_loop");
        double phaseSeconds = BenchIO.param(report.params, "phaseSeconds", 8);
        Scheduler.reset();
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
        Follower follower = BenchRobot.softwareFollower(hardwareMap);
        CloseFlowerPaths paths = new CloseFlowerPaths();
        follower.setPose(paths.start);
        follower.update();
        report.put("dsMsTransmissionInterval", telemetry.getMsTransmissionInterval());
        report.put("followerDriveCap", BenchRobot.DRIVE_CAP);

        while (opModeInInit()) {
            report.sendIfDue();
            sleep(20);
        }

        try {
            runPhase(report, follower, "idle", phaseSeconds);
            follower.follow(paths.toCloseFlower());
            runPhase(report, follower, "following", phaseSeconds);
            follower.stop();
            follower.drivetrain.stop();
            report.done("idle and following, " + phaseSeconds + " s each");
            while (opModeIsActive()) {
                report.sendIfDue();
                sleep(20);
            }
        } finally {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private void runPhase(BenchReport report, Follower follower, String name, double seconds) {
        report.phase(name);
        Samples periods = new Samples();
        BenchIO.Gc gc = BenchIO.Gc.now();
        BenchIO.Cpu cpu = BenchIO.Cpu.now();
        long start = System.nanoTime();
        long last = 0;
        long loops = 0;
        while (opModeIsActive() && (System.nanoTime() - start) / 1e9 < seconds) {
            long now = System.nanoTime();
            if (last != 0) periods.addNanos(now - last);
            last = now;
            loops++;

            follower.update();
            Scheduler.execute();
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            telemetry.update();
            report.status("%s %.0f/%.0fs", name, (now - start) / 1e9, seconds);
            report.sendIfDue();
        }
        if (!opModeIsActive()) throw new IllegalStateException("stopped during phase " + name + " before it finished");
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("loops", loops);
        r.put("loopPeriodMs", periods.summary());
        r.put("loopPeriodHistogram", periods.histogram(0.5, 80));
        r.put("gc", BenchIO.Gc.now().since(gc, loops));
        r.put("cpu", BenchIO.Cpu.now().since(cpu));
        report.endPhase(name, r);
    }
}
