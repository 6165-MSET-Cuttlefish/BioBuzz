package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.LimelightCamera;

import java.util.Arrays;

/**
 * Bench check for the Limelight cell-tip SnapScript. Drives {@link LimelightCamera} directly as a
 * bare {@code LinearOpMode}, so it needs only {@code limelight} in the hub config — not the
 * drivetrain and Pinpoint a {@code BioBuzzOpMode} pulls in by building the follower. Run it on
 * {@code res/xml/camera.xml}.
 *
 * <p>Cover the cell's tags and the verdict should flip {@link LimelightCamera#hiddenHoldSeconds}
 * later; uncover any one of them and it should clear immediately. Switch
 * {@code Cell Tip Test → cell} on FtcDashboard to retarget without restarting.
 *
 * <p>What this does <em>not</em> exercise is {@code RobotActions.checkTip}: Ivy's scheduler is
 * framework-owned, so the command path needs an OpMode on the full robot config.
 */
@TeleOp(name = "Cell Tip Test", group = "test")
public class CellTipTest extends LinearOpMode {

    @Config("Cell Tip Test")
    public static class Tuning {
        public static LimelightCamera.Cell cell = LimelightCamera.Cell.RED_1;
    }

    private final ElapsedTime sinceStart = new ElapsedTime();

    private LimelightCamera limelight;
    private double tippedAtSeconds = -1;

    @Override
    public void runOpMode() {
        limelight = new LimelightCamera(hardwareMap);
        limelight.init();

        while (opModeInInit()) pump();
        sinceStart.reset();
        while (opModeIsActive()) pump();

        limelight.stop();
    }

    private void pump() {
        limelight.update();
        boolean tipped = limelight.checkTip(Tuning.cell);
        if (opModeIsActive() && tipped && tippedAtSeconds < 0) tippedAtSeconds = sinceStart.seconds();

        telemetry.addData("Watching", Tuning.cell + " " + Arrays.toString(Tuning.cell.tagIds));
        telemetry.addData("Limelight", limelight.isPresent() ? "connected" : "NOT CONFIGURED");
        telemetry.addData("Verdict", !limelight.hasVerdict()
                ? "none — is the SnapScript pipeline selected?"
                : (tipped ? "TIPPED" : "upright"));
        telemetry.addData("Tags visible", limelight.getVisibleCount() + "/4");
        telemetry.addData("Hidden for", "%.2fs", limelight.getHiddenSeconds());
        telemetry.addData("Tags in frame", limelight.getDetectedTagCount());
        telemetry.addData("Seen since arming", limelight.hasSeenCell());
        telemetry.addData("First tip at", tippedAtSeconds < 0 ? "—" : String.format("%.2fs", tippedAtSeconds));
        telemetry.update();
    }
}
