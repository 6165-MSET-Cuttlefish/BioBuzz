package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

/**
 * Bench check for the Limelight HIVE-cell pipelines: shows the selected alliance pipeline's live
 * verdict and runs {@code RobotActions.checkTip} against it, so the command path and its timeout are
 * exercised too. Run it on {@code res/xml/biobuzz.xml} — going through the framework means the
 * drivetrain and Pinpoint must be in the config even though nothing here drives.
 *
 * <p>{@code Cell Tip Test → alliance} writes {@link Context#allianceColor} at {@code createRobot()},
 * and only takes effect on the next init, because that is when the pipeline is selected.
 *
 * <p>Point the camera at a cell of your alliance: right-side up should read SCORABLE, and showing
 * the cluster upside-down — or taking it out of frame — should read TIPPED a quarter second later.
 * A permanent "NO VERDICT" with the Limelight connected means the wrong script is on that pipeline
 * index.
 */
@TeleOp(name = "Cell Tip Test", group = "test")
public class CellTipTest extends EnhancedOpMode {

    @Config("Cell Tip Test")
    public static class Tuning {
        public static AllianceColor alliance = AllianceColor.RED;
    }

    private final ElapsedTime sinceStart = new ElapsedTime();

    private BioBuzzRobot bot;
    private Command watch;
    private double tippedAtSeconds = -1;

    @Override
    protected Robot createRobot() throws InterruptedException {
        Context.allianceColor = Tuning.alliance;
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
        Scheduler.schedule(watch);
    }

    @Override
    protected void gameLoop() {
        if (tippedAtSeconds < 0 && bot.limelight.isTipped()) tippedAtSeconds = sinceStart.seconds();
    }

    @Override
    protected void telemetry() {
        LimelightCamera limelight = bot.limelight;
        telemetry.addData("Alliance", limelight.getAlliance());
        if (Tuning.alliance != limelight.getAlliance()) {
            telemetry.addData("Note", "%s selected — re-init to apply", Tuning.alliance);
        }
        telemetry.addData("Limelight", limelight.isPresent() ? "connected" : "NOT CONFIGURED");
        telemetry.addData("Verdict", !limelight.hasVerdict()
                ? "none — is the right SnapScript on this alliance's pipeline?"
                : (limelight.isTipped() ? "TIPPED" : (limelight.isScorable() ? "SCORABLE" : "not scorable, within hold")));
        telemetry.addData("Cluster", "%s  %d/4 visible  (other %d/4)",
                limelight.getCluster(), limelight.getVisibleCount(), limelight.getOtherVisibleCount());
        telemetry.addData("Roll", Double.isNaN(limelight.getRollDeg())
                ? "—" : String.format("%.1fdeg", limelight.getRollDeg()));
        telemetry.addData("Command", Scheduler.isRunning(watch)
                ? "watching"
                : (tippedAtSeconds < 0 ? "gave up (timeout)" : "finished on tip"));
        telemetry.addData("First tip at", tippedAtSeconds < 0 ? "—" : String.format("%.2fs", tippedAtSeconds));
    }
}
