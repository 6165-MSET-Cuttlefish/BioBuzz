package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;
import org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter;
import org.firstinspires.ftc.teamcode.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.modules.vision.BallFieldTransform;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBallTracker;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateHistory;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.util.Collections;
import java.util.List;

/**
 * Deliberately framework-free; "field" positions are relative to the robot's pose at init. Motors
 * and Pinpoint come from {@link BettaConstants} at INIT, the same robot frame the follower uses. A Limelight
 * fault shows as EnhancedOpMode shows one (big red DS line, dashboard row and overlay text) and driving goes on.
 */
@TeleOp(name = "Ball Field Drive", group = "Test")
public class BallFieldDriveTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private BettaBareDrive drive;

    private LimelightBallSource source;
    private final RobotStateHistory robotHistory = new RobotStateHistory();
    private final FieldBallTracker fieldBallTracker = new FieldBallTracker();
    private List<FieldBall> fieldBalls = Collections.emptyList();
    private DualTelemetry out;

    @Override
    public void runOpMode() {

        out = new DualTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new BettaBareDrive(hardwareMap, false);
        pinpoint = drive.pinpoint;

        source = new LimelightBallSource(hardwareMap);

        try {
            while (opModeInInit()) pump(false);
            while (opModeIsActive()) pump(true);
        } finally {
            drive.stop();
            source.stop();
        }
    }

    private void pump(boolean driving) {
        pinpoint.update();
        recordRobotState();
        boolean newFrame = source.update();
        if (driving) drive.drive(gamepad1);

        LimelightBallSource.Frame frame = source.latest();
        double now = System.nanoTime() * 1e-9;
        if (newFrame) {
            RobotStateHistory.Sample captureState = robotHistory.sampleAt(frame.timestampSeconds);
            fieldBalls = fieldBallTracker.update(BallFieldTransform.toField(frame.balls, captureState), now);
        } else if (frame.stale) {
            fieldBalls = fieldBallTracker.update(Collections.<FieldBall>emptyList(), now);
        }

        out.beginLoop();
        TelemetryPacket packet = new TelemetryPacket(false);
        packet.setDisplayFormat(TelemetryPacket.DisplayFormat.HTML);
        packet.setCaptionValueSeparator(": ");
        out.setPacket(packet);
        String problem = source.problem();
        if (problem != null) {
            out.addFault(LimelightBallSource.FAULT_KEY, problem);
            Canvas overlay = packet.fieldOverlay();
            overlay.setAlpha(1);
            overlay.setFill(HtmlFormatter.COLOR_FAULT);
            overlay.fillText("FAULT " + LimelightBallSource.FAULT_KEY + ": " + problem, 2, 7,
                    "bold 5px sans-serif", 0, true);
        }
        out.addData("Pinpoint status", pinpoint.getDeviceStatus());
        RobotStateHistory.Sample robot = robotHistory.newest();
        out.addData("Robot (in, deg)", "(%.1f, %.1f) @ %.0f",
                robot.x, robot.y, Math.toDegrees(robot.heading));
        out.addData("Limelight FPS", "%.1f%s", frame.fps, frame.stale ? "  STALE" : "");
        out.addData("Latency (ms)", "%.0f", frame.latencyMs);
        out.addData("Balls", fieldBalls.size());
        int shown = 0;
        for (FieldBall ball : fieldBalls) {
            if (shown++ >= 5) break;
            out.addData("Ball " + ball.id, ball + (ball.visible() ? "" : " [last seen]"));
        }
        out.update();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        sleep(20);
    }

    private void recordRobotState() {
        double now = System.nanoTime() * 1e-9;
        robotHistory.record(new RobotStateHistory.Sample(
                pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS),
                pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS),
                now));
    }
}
