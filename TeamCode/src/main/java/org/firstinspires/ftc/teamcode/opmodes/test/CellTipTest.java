package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

/**
 * Bench check for the Limelight cell-tip SnapScript: watches one cell's tags and shows the script's
 * live verdict, plus how long a {@code checkTip} command took to finish. Needs {@code limelight} in
 * the hub config, with the SnapScript loaded at {@code LimelightCamera.snapScriptPipeline}.
 *
 * <p>Cover the cell's tags with a hand and the verdict should flip a quarter second later; uncover
 * any one of them and it should clear immediately.
 */
@TeleOp(name = "Cell Tip Test", group = "test")
public class CellTipTest extends BioBuzzOpMode {

    @Config("Cell Tip Test")
    public static class Tuning {
        public static LimelightCamera.Cell cell = LimelightCamera.Cell.RED_1;
    }

    private final ElapsedTime sinceStart = new ElapsedTime();
    private Command watch;
    private double tippedAtSeconds = -1;

    @Override
    protected void initialize() {
        watch = robot.actions.checkTip(Tuning.cell);
    }

    @Override
    protected void initializeLoop() {
        robot.limelight.checkTip(Tuning.cell);
    }

    @Override
    protected void onStart() {
        sinceStart.reset();
        Scheduler.schedule(watch);
    }

    @Override
    protected void gameLoop() {
        if (tippedAtSeconds < 0 && robot.limelight.isTipped()) {
            tippedAtSeconds = sinceStart.seconds();
        }
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Watching", Tuning.cell + " " + java.util.Arrays.toString(Tuning.cell.tagIds));
        telemetry.addData("Limelight", robot.limelight.isPresent() ? "connected" : "NOT CONFIGURED");
        telemetry.addData("Verdict", !robot.limelight.hasVerdict()
                ? "none — is the SnapScript pipeline selected?"
                : (robot.limelight.isTipped() ? "TIPPED" : "upright"));
        telemetry.addData("Tags visible", robot.limelight.getVisibleCount() + "/4");
        telemetry.addData("Hidden for", "%.2fs", robot.limelight.getHiddenSeconds());
        telemetry.addData("Tags in frame", robot.limelight.getDetectedTagCount());
        telemetry.addData("Command", Scheduler.isRunning(watch)
                ? "watching"
                : (tippedAtSeconds < 0 ? "gave up (timeout)" : "finished on tip"));
        telemetry.addData("First tip at", tippedAtSeconds < 0 ? "—" : String.format("%.2fs", tippedAtSeconds));
    }
}
