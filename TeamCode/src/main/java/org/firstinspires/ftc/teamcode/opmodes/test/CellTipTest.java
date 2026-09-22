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

import java.util.Arrays;

/**
 * Bench check for the Limelight cell-tip pipelines: shows the selected pipeline's live verdict and
 * runs {@code RobotActions.checkTip} against it, so the command path and its timeout are exercised
 * too. Run it on {@code res/xml/biobuzz.xml} — going through the framework means the drivetrain and
 * Pinpoint must be in the config even though nothing here drives.
 *
 * <p>{@code Cell Tip Test → cell} picks which cell to watch, and is applied to {@link Context} at
 * {@code createRobot()}. It only takes effect on the next init, because that is when the Limelight
 * pipeline is selected — change it on FtcDashboard, then re-init.
 *
 * <p>Cover the cell's tags and the verdict should flip a quarter second later; uncover any one of
 * them and it should clear immediately. A permanent "NO VERDICT" with the Limelight connected means
 * the pipeline at that index isn't the script for this cell.
 */
@TeleOp(name = "Cell Tip Test", group = "test")
public class CellTipTest extends EnhancedOpMode {

    @Config("Cell Tip Test")
    public static class Tuning {
        public static Context.Cell cell = Context.Cell.RED_1;
    }

    private final ElapsedTime sinceStart = new ElapsedTime();

    private BioBuzzRobot bot;
    private Command watch;
    private double tippedAtSeconds = -1;

    @Override
    protected Robot createRobot() throws InterruptedException {
        Context.allianceColor = AllianceColor.RED;
        Context.cell = Tuning.cell;
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
        Context.Cell cell = limelight.getCell();
        telemetry.addData("Watching", "%s pipeline %d %s", cell, cell.pipeline, Arrays.toString(cell.tagIds));
        if (Tuning.cell != cell) telemetry.addData("Note", "%s selected — re-init to apply", Tuning.cell);
        telemetry.addData("Limelight", limelight.isPresent() ? "connected" : "NOT CONFIGURED");
        telemetry.addData("Verdict", !limelight.hasVerdict()
                ? "none — is the right SnapScript on pipeline " + cell.pipeline + "?"
                : (limelight.isTipped() ? "TIPPED" : "upright"));
        telemetry.addData("Tags visible", limelight.getVisibleCount() + "/4");
        telemetry.addData("Hidden for", "%.2fs", limelight.getHiddenSeconds());
        telemetry.addData("Tags in frame", limelight.getDetectedTagCount());
        telemetry.addData("Seen since start", limelight.hasSeenCell());
        telemetry.addData("Command", Scheduler.isRunning(watch)
                ? "watching"
                : (tippedAtSeconds < 0 ? "gave up (timeout)" : "finished on tip"));
        telemetry.addData("First tip at", tippedAtSeconds < 0 ? "—" : String.format("%.2fs", tippedAtSeconds));
    }
}
