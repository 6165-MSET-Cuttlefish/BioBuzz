package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

@TeleOp(name = "Bench: Overlay NaN", group = "Test")
public class OverlayNanBench extends EnhancedOpMode {
    private static final double NAN_START_S = 2;
    private static final double NAN_END_S = 3;
    private static final double AFTER_S = 5;

    private BenchReport report;
    private long startNs;
    private long loops;
    private long nanStartLoop = -1;
    private long nanEndLoop = -1;
    private String stage = "before";

    @Override
    protected Robot createRobot() {
        return new BenchRobot(this);
    }

    @Override
    protected void initialize() {
        report = new BenchReport("Overlay NaN", "overlay_nan");
    }

    @Override
    protected void onStart() {
        startNs = System.nanoTime();
        report.phase("before");
    }

    @Override
    protected void gameLoop() {
        loops++;
        if (report.isDone()) return;
        double t = (System.nanoTime() - startNs) / 1e9;
        if (nanStartLoop < 0 && t >= NAN_START_S) {
            nanStartLoop = loops;
            stage = "drawing NaN";
            report.phase(stage);
        }
        if (nanStartLoop >= 0 && nanEndLoop < 0 && t >= NAN_END_S) {
            nanEndLoop = loops;
            stage = "after";
            report.phase(stage);
        }
        if (nanEndLoop >= 0 && t >= NAN_END_S + AFTER_S) {
            report.put("nanStartLoop", nanStartLoop);
            report.put("nanEndLoop", nanEndLoop);
            report.put("lastLoop", loops);
            report.put("afterSeconds", AFTER_S);
            report.done("a NaN circle in every overlay from loop " + nanStartLoop + " to " + nanEndLoop + ", then "
                    + AFTER_S + " s to loop " + loops);
        }
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (nanStartLoop >= 0 && nanEndLoop < 0) overlay.strokeCircle(Double.NaN, 0, 1);
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData("Bench loop", loops);
        telemetry.addData("Bench NaN", stage);
    }
}
