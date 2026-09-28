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
import org.firstinspires.ftc.teamcode.modules.vision.LimelightBallSource;
import org.firstinspires.ftc.teamcode.modules.vision.BallFieldTransform;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBall;
import org.firstinspires.ftc.teamcode.modules.vision.FieldBallTracker;
import org.firstinspires.ftc.teamcode.modules.vision.RobotStateHistory;

import java.util.Collections;
import java.util.List;

/** Deliberately framework-free; "field" positions are relative to the robot's pose at init. */
@TeleOp(name = "Ball Field Drive", group = "Test")
public class BallFieldDriveTest extends LinearOpMode {

    private static final String PINPOINT_NAME = "pinpoint";

    // The Cuttle bot's Pinpoint values; must match pedro/CuttleConstants.localizerConfig.
    private static final double X_POD_OFFSET_IN = 5.827277476393332;
    private static final double Y_POD_OFFSET_IN = -0.7426906946137196;
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx fl, bl, fr, br;

    private LimelightBallSource source;
    private final RobotStateHistory robotHistory = new RobotStateHistory();
    private final FieldBallTracker fieldBallTracker = new FieldBallTracker();
    // The loop outruns the Limelight; only transform + track on a new frame.
    private double lastFieldBallFrameTimestamp = -1;
    private List<FieldBall> fieldBalls = Collections.emptyList();

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
        pinpoint.resetPosAndIMU(); // robot must be stationary (IMU recalibration)

        source = new LimelightBallSource(hardwareMap);

        while (opModeInInit()) pump(false);
        while (opModeIsActive()) pump(true);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        source.stop();
    }

    private void pump(boolean drive) {
        pinpoint.update();
        recordRobotState();
        source.update();
        if (drive) driveFromGamepad();

        LimelightBallSource.Frame frame = source.latest();
        if (frame.timestampSeconds != lastFieldBallFrameTimestamp) {
            lastFieldBallFrameTimestamp = frame.timestampSeconds;
            RobotStateHistory.Sample captureState = robotHistory.sampleAt(frame.timestampSeconds);
            List<FieldBall> fresh = BallFieldTransform.toField(frame.balls, captureState);
            fieldBalls = fieldBallTracker.update(fresh, System.nanoTime() * 1e-9);
        }

        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        RobotStateHistory.Sample now = robotHistory.newest();
        telemetry.addData("Robot (in, deg)", "(%.1f, %.1f) @ %.0f",
                now.x, now.y, Math.toDegrees(now.heading));
        telemetry.addData("Limelight FPS", "%.1f", frame.fps);
        telemetry.addData("Latency (ms)", "%.0f", frame.latencyMs);
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
