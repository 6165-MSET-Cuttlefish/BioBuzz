package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.modules.CellTipCamera;

/**
 * Bench check for the webcam cell-tip verdict and {@code RobotActions.checkTip}. Run it on
 * {@code res/xml/cuttle_decode.xml}: the framework needs the drivetrain and Pinpoint even here.
 */
@TeleOp(name = "Cell Tip Test", group = "Test")
public class CellTipTest extends BioBuzzOpMode {

    private final ElapsedTime sinceStart = new ElapsedTime();

    private Command watch;
    private boolean started;
    private double tippedAtSeconds = -1;
    private String outcome;

    @Override
    protected void initialize() {
        watch = robot.actions.checkTip();
    }

    @Override
    protected void onStart() {
        sinceStart.reset();
        started = true;
        Scheduler.schedule(watch);
    }

    @Override
    protected void gameLoop() {
        // Judge the outcome before recording the tip: the command's done() saw last loop's tip state.
        if (outcome == null && !Scheduler.isRunning(watch)) {
            outcome = tippedAtSeconds < 0 ? "gave up (timeout)" : "finished on tip";
        }
        if (tippedAtSeconds < 0 && robot.cellTip.isTipped()) tippedAtSeconds = sinceStart.seconds();
    }

    @Override
    protected void telemetry() {
        CellTipCamera cellTip = robot.cellTip;
        if (Context.allianceColor != cellTip.getAlliance()) {
            telemetry.addData("Note", "%s selected — re-init to apply", Context.allianceColor);
        }
        telemetry.addData("Command", !started ? "not started" : (outcome == null ? "watching" : outcome));
        telemetry.addData("First tip at", tippedAtSeconds < 0 ? "—" : String.format("%.2fs", tippedAtSeconds));
    }
}
