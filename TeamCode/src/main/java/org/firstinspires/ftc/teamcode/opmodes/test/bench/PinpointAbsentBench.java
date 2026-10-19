package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Pinpoint Absent", group = "Test")
public class PinpointAbsentBench extends LinearOpMode {

    @Override
    public void runOpMode() {
        BenchReport report = new BenchReport("Pinpoint Absent", "pinpoint_absent");
        if (!report.holdUntilStart(this)) return;

        report.phase("create");
        Map<String, Object> create = new LinkedHashMap<>();
        report.endPhase("create", create);
        long t0 = System.nanoTime();
        Follower follower;
        try {
            follower = BettaConstants.create(hardwareMap);
            create.put("threw", null);
        } catch (RuntimeException e) {
            create.put("threw", e.toString());
            throw e;
        } finally {
            create.put("createMs", (System.nanoTime() - t0) / 1e6);
            report.endPhase("create", create);
        }

        report.phase("update");
        Map<String, Object> update = new LinkedHashMap<>();
        Samples ms = new Samples();
        List<String> poses = new ArrayList<>();
        try {
            follower.setPose(new Pose(56, 8, Math.PI / 2));
            for (int i = 0; i < 200 && opModeIsActive(); i++) {
                long u0 = System.nanoTime();
                follower.update();
                ms.addNanos(System.nanoTime() - u0);
                if (i % 50 == 0) poses.add(follower.pose().toString());
                report.sendIfDue();
            }
            update.put("threw", null);
        } catch (RuntimeException e) {
            update.put("threw", e.toString());
            throw e;
        } finally {
            update.put("updateMs", ms.summary());
            update.put("posesEvery50", poses);
            report.endPhase("update", update);
            follower.stop();
            follower.drivetrain.stop();
        }
        report.done("create and 200 updates without a Pinpoint");
        report.holdUntilStop(this);
    }
}
