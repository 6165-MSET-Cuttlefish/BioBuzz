package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    private DcMotorEx fl, bl, fr, br;

    private LimelightBallSource source;
    private final RobotStateHistory robotHistory = new RobotStateHistory();
    private final FieldBallTracker fieldBallTracker = new FieldBallTracker();
    private List<FieldBall> fieldBalls = Collections.emptyList();
    private DualTelemetry out;

    @Override
    public void runOpMode() {

        out = new DualTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BettaConstants.MecanumSettings m = BettaConstants.mecanum;
        fl = hardwareMap.get(DcMotorEx.class, m.frontLeftName);
        bl = hardwareMap.get(DcMotorEx.class, m.backLeftName);
        fr = hardwareMap.get(DcMotorEx.class, m.frontRightName);
        br = hardwareMap.get(DcMotorEx.class, m.backRightName);
        fl.setDirection(m.frontLeftDirection);
        bl.setDirection(m.backLeftDirection);
        fr.setDirection(m.frontRightDirection);
        br.setDirection(m.backRightDirection);

        // The same calls, in the same order, as Pedro's PinpointLocalizer.
        BettaConstants.PinpointSettings p = BettaConstants.pinpoint;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, p.name);
        pinpoint.setOffsets(p.xPodOffset, p.yPodOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(p.podType);
        pinpoint.setEncoderDirections(p.xPodDirection, p.yPodDirection);
        pinpoint.resetPosAndIMU(); // robot must be stationary (IMU recalibration)

        source = new LimelightBallSource(hardwareMap);

        try {
            while (opModeInInit()) pump(false);
            while (opModeIsActive()) pump(true);
        } finally {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
            source.stop();
        }
    }

    private void pump(boolean drive) {
        pinpoint.update();
        recordRobotState();
        boolean newFrame = source.update();
        if (drive) driveFromGamepad();

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

    private void driveFromGamepad() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Mirrors Drivetrain.setMecanumTargets (robot-centric) so this drives like BioBuzz Tele.
        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
                Math.max(Math.abs(frontRight), Math.abs(backRight)))));

        fl.setPower(frontLeft / max);
        bl.setPower(backLeft / max);
        fr.setPower(frontRight / max);
        br.setPower(backRight / max);
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
