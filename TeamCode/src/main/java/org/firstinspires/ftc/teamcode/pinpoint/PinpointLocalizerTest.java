package org.firstinspires.ftc.teamcode.pinpoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config("Pinpoint Localizer Test")
@TeleOp(name = "Pinpoint Localizer Test", group = "Test")
public class PinpointLocalizerTest extends LinearOpMode {
    public static String pinpointName = "pinpoint";
    // OctoQuad's TCP offset points from the pods' TCP to the centre, so these are (-tcpOffsetMmY, -tcpOffsetMmX).
    public static double xPodOffsetMm = -34.13125;
    public static double yPodOffsetMm = 157.1625;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static boolean useCustomResolution = false;
    public static double customTicksPerMm = 19.89436789;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    // Off keeps the factory-calibrated scalar; on overrides it.
    public static boolean useCustomYawScalar = false;
    public static double yawScalar = 1.0;

    public static double startXIn = 0;
    public static double startYIn = 0;
    public static double startHeadingDeg = 0;

    public static String frontLeftName = "fl";
    public static String backLeftName = "bl";
    public static String frontRightName = "fr";
    public static String backRightName = "br";
    public static DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.FORWARD;
    public static double maxPower = 1.0;
    public static boolean fieldCentric = false;

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx fl, bl, fr, br;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        fl = motor(frontLeftName, frontLeftDirection);
        bl = motor(backLeftName, backLeftDirection);
        fr = motor(frontRightName, frontRightDirection);
        br = motor(backRightName, backRightDirection);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);

        applyParameters();
        while (opModeInInit()) {
            if (gamepad1.aWasPressed()) applyParameters();
            if (gamepad1.bWasPressed()) setStartPose();
            pinpoint.update();
            show();
        }

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                setDrivePower(0, 0, 0, 0);
                applyParameters();
                continue;
            }
            if (gamepad1.bWasPressed()) setStartPose();
            pinpoint.update();
            drive();
            show();
        }

        setDrivePower(0, 0, 0, 0);
    }

    private DcMotorEx motor(String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void applyParameters() {
        pinpoint.setOffsets(xPodOffsetMm, yPodOffsetMm, DistanceUnit.MM);
        if (useCustomResolution) {
            pinpoint.setEncoderResolution(customTicksPerMm, DistanceUnit.MM);
        } else {
            pinpoint.setEncoderResolution(podType);
        }
        pinpoint.setEncoderDirections(xPodDirection, yPodDirection);
        if (useCustomYawScalar) pinpoint.setYawScalar(yawScalar);
        pinpoint.resetPosAndIMU();
        waitForCalibration();
        setStartPose();
    }

    private void waitForCalibration() {
        sleep(50);
        GoBildaPinpointDriver.DeviceStatus status;
        do {
            pinpoint.update();
            status = pinpoint.getDeviceStatus();
            telemetry.addLine("Calibrating IMU: keep the robot still");
            telemetry.addData("Status", status);
            telemetry.update();
            sleep(20);
        } while (!isStopRequested()
                && (status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                || status == GoBildaPinpointDriver.DeviceStatus.NOT_READY));
    }

    private void setStartPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startXIn, startYIn, AngleUnit.DEGREES, startHeadingDeg));
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (fieldCentric) {
            // Pinpoint's +y is left while the stick's +x is right.
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            double forward = y * Math.cos(heading) - x * Math.sin(heading);
            double right = y * Math.sin(heading) + x * Math.cos(heading);
            y = forward;
            x = right;
        }

        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
                Math.max(Math.abs(frontRight), Math.abs(backRight)))));
        double scale = maxPower / max;

        setDrivePower(frontLeft * scale, backLeft * scale, frontRight * scale, backRight * scale);
    }

    private void setDrivePower(double frontLeft, double backLeft, double frontRight, double backRight) {
        fl.setPower(frontLeft);
        bl.setPower(backLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
    }

    private void show() {
        telemetry.addLine("Sticks: drive  A: apply parameters and recalibrate (keep still)  B: reset to start pose");
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("x (in)", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y (in)", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("heading unwrapped (deg)", pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("vx (in/s)", pinpoint.getVelX(DistanceUnit.INCH));
        telemetry.addData("vy (in/s)", pinpoint.getVelY(DistanceUnit.INCH));
        telemetry.addData("heading vel (deg/s)", pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Raw X pod ticks (push forward: rises)", pinpoint.getEncoderX());
        telemetry.addData("Raw Y pod ticks (push left: rises)", pinpoint.getEncoderY());
        telemetry.addData("x offset on device (mm)", pinpoint.getXOffset(DistanceUnit.MM));
        telemetry.addData("y offset on device (mm)", pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.addData("Yaw scalar on device", pinpoint.getYawScalar());
        telemetry.addData("Pinpoint loop (us)", pinpoint.getLoopTime());
        telemetry.addData("Pinpoint frequency (Hz)", pinpoint.getFrequency());
        telemetry.addData("Device version", pinpoint.getDeviceVersion());
        telemetry.update();
    }
}
