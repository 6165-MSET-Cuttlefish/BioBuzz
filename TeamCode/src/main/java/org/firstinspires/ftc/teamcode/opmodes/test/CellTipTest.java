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
 * Bench check for the Limelight cell-tip SnapScript: shows the script's live verdict for one cell
 * and runs {@code RobotActions.checkTip} against it, so the command path and its timeout are
 * exercised too. Run it on {@code res/xml/biobuzz.xml} — going through the framework means the
 * drivetrain and Pinpoint must be in the config even though nothing here drives.
 *
 * <p>Cover the cell's tags and the verdict should flip {@link LimelightCamera#hiddenHoldSeconds}
 * later; uncover any one of them and it should clear immediately. {@code Cell Tip Test → cell}
 * retargets the live verdict without a restart, but the command is built at init and keeps watching
 * whichever cell was selected then.
 */
@TeleOp(name = "Cell Tip Test", group = "test")
public class CellTipTest extends EnhancedOpMode {

    @Config("Cell Tip Test")
    public static class Tuning {
        public static LimelightCamera.Cell cell = LimelightCamera.Cell.RED_1;
    }

    private final ElapsedTime sinceStart = new ElapsedTime();

    private BioBuzzRobot bot;
    private Command watch;
    private LimelightCamera.Cell watchedByCommand;
    private double tippedAtSeconds = -1;

    @Override
    protected Robot createRobot() throws InterruptedException {
        Context.allianceColor = AllianceColor.RED;
        bot = new BioBuzzRobot(this);
        return bot;
    }

    @Override
    protected void initialize() {
        watchedByCommand = Tuning.cell;
        watch = bot.actions.checkTip(watchedByCommand);
    }

    @Override
    protected void initializeLoop() {
        bot.limelight.watch(Tuning.cell);
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
        telemetry.addData("Watching", watchedByCommand + " " + Arrays.toString(watchedByCommand.tagIds));
        telemetry.addData("Limelight", limelight.isPresent() ? "connected" : "NOT CONFIGURED");
        telemetry.addData("Verdict", !limelight.hasVerdict()
                ? "none — is the SnapScript pipeline selected?"
                : (limelight.isTipped() ? "TIPPED" : "upright"));
        telemetry.addData("Tags visible", limelight.getVisibleCount() + "/4");
        telemetry.addData("Hidden for", "%.2fs", limelight.getHiddenSeconds());
        telemetry.addData("Tags in frame", limelight.getDetectedTagCount());
        telemetry.addData("Seen since arming", limelight.hasSeenCell());
        telemetry.addData("Command", Scheduler.isRunning(watch)
                ? "watching"
                : (tippedAtSeconds < 0 ? "gave up (timeout)" : "finished on tip"));
        telemetry.addData("First tip at", tippedAtSeconds < 0 ? "—" : String.format("%.2fs", tippedAtSeconds));
    }
}
