package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamSession;
import org.firstinspires.ftc.teamcode.modules.vision.BallDetectionPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.BallFieldTransform;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateHistory;

import java.util.List;

/**
 * Drives on raw gamepad mecanum and reports ball field positions, using a {@link
 * GoBildaPinpointDriver} read directly for robot pose — no {@code Follower}, {@code Robot}, or
 * {@code EnhancedOpMode}. Deliberately independent of the rest of the framework so it only exercises
 * the vision subsystems ({@link BallDetectionPipeline}, {@link BallFieldTransform}, {@link
 * RobotStateHistory}) plus the bare hardware needed to move and localize: {@code fl}/{@code
 * bl}/{@code fr}/{@code br} motors, {@code pinpoint}, and {@code nerdDetector}.
 *
 * <p>Position is relative to wherever the robot was at init — {@link
 * GoBildaPinpointDriver#resetPosAndIMU()} zeroes it there, not to a field-alliance start pose the
 * way an auto's {@code FieldPose.forAlliance(...)} would, so "field position" here means "position
 * relative to that origin," not true field coordinates. Keep the robot stationary through init
 * (IMU recalibration); the Pinpoint offsets/directions below must match {@code
 * pedro/Constants.localizerConfig} — the Y (strafe) pod direction there is an unverified guess per
 * CLAUDE.md, so trust heading before trusting strafe.
 */
@TeleOp(name = "Ball Field Drive", group = "test")
public class BallFieldDriveTest extends LinearOpMode {

    // Must match res/xml/camera.xml and pedro/Constants.localizerConfig's pod offsets/directions.
    private static final String WEBCAM_NAME = "nerdDetector";
    private static final String PINPOINT_NAME = "pinpoint";

    private static final double X_POD_OFFSET_IN = 5.827277476393332;
    private static final double Y_POD_OFFSET_IN = -0.7426906946137196;
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx fl, bl, fr, br;

    private WebcamSession session;
    private BallDetectionPipeline pipeline;
    private final RobotStateHistory robotHistory = new RobotStateHistory();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        pinpoint.setOffsets(X_POD_OFFSET_IN, Y_POD_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU(); // robot must be stationary; also zeroes pose to (0, 0, 0)

        pipeline = new BallDetectionPipeline();
        session = new WebcamSession(hardwareMap, telemetry, WEBCAM_NAME, pipeline);

        while (opModeInInit()) pump(false);
        while (opModeIsActive()) pump(true);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        session.close();
    }

    private void pump(boolean drive) {
        pinpoint.update();
        recordRobotState();
        session.update();
        if (drive) driveFromGamepad();

        BallDetectionPipeline.Frame frame = pipeline.latest();
        RobotStateHistory.Sample captureState = robotHistory.sampleAt(frame.timestampSeconds);
        List<FieldBall> fieldBalls = BallFieldTransform.toField(frame.balls, captureState);

        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        RobotStateHistory.Sample now = robotHistory.newest();
        if (now != null) {
            telemetry.addData("Robot (in, deg)", "(%.1f, %.1f) @ %.0f",
                    now.x, now.y, Math.toDegrees(now.heading));
        }
        telemetry.addData("FPS", "%.1f", frame.fps);
        telemetry.addData("Balls", fieldBalls.size());
        int shown = 0;
        for (FieldBall ball : fieldBalls) {
            if (shown++ >= 5) break;
            telemetry.addData("Ball " + ball.id, ball.toString());
        }
        telemetry.update();

        sleep(20);
    }

    private void driveFromGamepad() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Matches Drivetrain.setMecanumTargets's robot-centric mixing and normalization exactly, so
        // this drives the same way BioBuzz Tele does.
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
