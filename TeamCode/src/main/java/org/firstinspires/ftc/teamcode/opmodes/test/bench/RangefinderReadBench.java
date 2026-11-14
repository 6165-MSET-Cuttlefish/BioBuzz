package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vendor.brushland.LaserRangefinder;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Reads a Brushland laser rangefinder's settings over I2C and never writes one. In INIT it keeps reading until the
 * sensor answers, so a sensor stored in analog mode that is plugged in meanwhile sees I2C traffic in its first second
 * after power-up and switches back to I2C. START does nothing; STOP ends it.
 */
@TeleOp(name = "Bench: Rangefinder Read", group = "Test")
public class RangefinderReadBench extends LinearOpMode {
    private static final String HINT = "the runner's bench_rf_i2c config names it Laser on Control Hub I2C port 3";
    // The vendored driver's private register numbers (vendor/brushland/LaserRangefinder.java).
    private static final int R_PIN0_MODE = 0x28;
    private static final int R_PIN1_MODE = 0x2D;
    private static final int R_DISTMODE = 0x36;
    private static final long POLL_MS = 50;
    private static final int ANSWERS_NEEDED = 3;
    private static final int MIN_BUDGET_MS = 5;

    @Override
    public void runOpMode() {
        Map<String, Object> params = BenchIO.params();
        String label = BenchIO.param(params, "label", "read");
        double waitSeconds = BenchIO.param(params, "waitSeconds", 60);
        BenchReport report = new BenchReport("Rangefinder Read", "rangefinder_read_" + label);
        LaserRangefinder lrf = new LaserRangefinder(BenchIO.require(hardwareMap, RevColorSensorV3.class, "Laser", HINT));
        report.phase("reading until it answers");
        long t0 = System.nanoTime();
        int polls = 0;
        int silent = 0;
        int streak = 0;
        double firstAnswerMs = Double.NaN;
        while (opModeInInit() && !report.isDone()) {
            int[] timing = lrf.getTiming();
            polls++;
            double ms = (System.nanoTime() - t0) / 1e6;
            if (timing[0] >= MIN_BUDGET_MS) {
                if (streak == 0) firstAnswerMs = ms;
                streak++;
            } else {
                streak = 0;
                silent++;
            }
            if (streak >= ANSWERS_NEEDED) {
                Map<String, Object> settings = settings(lrf);
                report.put("answering", true);
                report.put("firstAnswerMs", firstAnswerMs);
                report.put("settings", settings);
                report.put("polls", polls);
                report.put("silentPolls", silent);
                report.done("answering: timing " + settings.get("timing") + ", distance mode "
                        + settings.get("distanceModeRaw"));
            } else if (ms > waitSeconds * 1000) {
                report.put("answering", false);
                report.put("polls", polls);
                report.put("silentPolls", silent);
                report.done("not answering on I2C after " + Math.round(ms / 1000) + " s");
            } else {
                report.status("%d reads, %d without an answer", polls, silent);
                report.sendIfDue();
                sleep(POLL_MS);
            }
        }
        report.holdUntilStart(this);
        report.holdUntilStop(this);
    }

    private static Map<String, Object> settings(LaserRangefinder lrf) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("pin0", pin(lrf.i2c.read(R_PIN0_MODE, 5)));
        m.put("pin1", pin(lrf.i2c.read(R_PIN1_MODE, 5)));
        m.put("distanceModeRaw", lrf.i2c.read8(R_DISTMODE) & 0xFF);
        m.put("timing", lrf.getTiming());
        m.put("roi", lrf.getROI());
        m.put("opticalCenter", lrf.getOpticalCenter());
        m.put("distanceMm", lrf.getDistance(DistanceUnit.MM));
        m.put("distanceStatus", lrf.getStatus());
        return m;
    }

    /** A pin's mode byte, then its two little-endian bounds (thresholds in digital mode). */
    private static int[] pin(byte[] b) {
        return new int[] {b[0] & 0xFF, (b[1] & 0xFF) | (b[2] & 0xFF) << 8, (b[3] & 0xFF) | (b[4] & 0xFF) << 8};
    }
}
