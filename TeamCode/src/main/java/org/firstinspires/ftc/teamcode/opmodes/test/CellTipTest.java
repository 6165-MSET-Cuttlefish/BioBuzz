package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

/**
 * Bench check for the Limelight pipelines and {@code RobotActions.checkTip}. Run it on
 * {@code res/xml/cuttledecode.xml}: the framework needs the drivetrain and Pinpoint even here.
 */
@TeleOp(name = "Cell Tip Test", group = "Test")
public class CellTipTest extends EnhancedOpMode {

    private final ElapsedTime sinceStart = new ElapsedTime();

    private BioBuzzRobot bot;
    private Command watch;
    private boolean started;
    private double tippedAtSeconds = -1;
    private String outcome;

    @Override
    protected Robot createRobot() throws InterruptedException {
        bot = new BioBuzzRobot(this);
        return bot;
    }

    @Override
    protected void initialize() {
        watch = bot.actions.checkTip();
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
        if (tippedAtSeconds < 0 && bot.limelight.isTipped()) tippedAtSeconds = sinceStart.seconds();
    }

    @Override
    protected void telemetry() {
        LimelightCamera limelight = bot.limelight;
        telemetry.addData("Alliance", limelight.getAlliance());
        if (Context.allianceColor != limelight.getAlliance()) {
            telemetry.addData("Note", "%s selected — re-init to apply", Context.allianceColor);
        }
        telemetry.addData("Limelight", limelight.isConnected() ? "connected" : "not responding");
        telemetry.addData("Verdict", !limelight.hasVerdict()
                ? (limelight.isWrongPipeline()
                        ? "none — incorrect pipeline! please switch manually"
                        : (limelight.isConnected()
                                ? "none — no result from the Limelight yet"
                                : "none — the Limelight is not responding"))
                : (limelight.isTipped() ? "TIPPED" : "SCORABLE"));
        telemetry.addData("Command", !started ? "not started" : (outcome == null ? "watching" : outcome));
        telemetry.addData("First tip at", tippedAtSeconds < 0 ? "—" : String.format("%.2fs", tippedAtSeconds));
    }
}
