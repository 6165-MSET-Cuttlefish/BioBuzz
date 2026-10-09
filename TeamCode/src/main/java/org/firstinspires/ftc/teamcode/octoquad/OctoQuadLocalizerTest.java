package org.firstinspires.ftc.teamcode.octoquad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

@Config("OctoQuad Localizer Test")
@TeleOp(name = "OctoQuad Localizer Test", group = "Test")
public class OctoQuadLocalizerTest extends LinearOpMode {
    private static final double MM_PER_INCH = 25.4;

    public static String name = "octoquad";
    public static int portX = 3;
    public static int portY = 4;
    public static OctoQuad.EncoderDirection directionX = OctoQuad.EncoderDirection.REVERSE;
    public static OctoQuad.EncoderDirection directionY = OctoQuad.EncoderDirection.REVERSE;
    public static double ticksPerMmX = 19.89;
    public static double ticksPerMmY = 19.89;
    public static double tcpOffsetMmX = -34.13125f;
    public static double tcpOffsetMmY = 157.1625f;
    public static double imuHeadingScalar = 1.0085;
    public static int velocityIntervalMs = 25;

    private OctoQuad.LocalizerDataBlock pose = new OctoQuad.LocalizerDataBlock();
    private OctoQuad.LocalizerDataBlock poseRead = new OctoQuad.LocalizerDataBlock();
    private OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
    private OctoQuad.EncoderDataBlock encodersRead = new OctoQuad.EncoderDataBlock();
    private OctoQuad octoquad;
    private DcMotorEx fl, bl, fr, br;
    private String firmware;
    private boolean lastPacketOk;
    private int badPackets;
    private int totalPackets;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        octoquad = hardwareMap.get(OctoQuad.class, name);
        firmware = octoquad.getFirmwareVersionString();
        octoquad.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        BettaConstants.MecanumSettings m = BettaConstants.mecanum;
        fl = motor(m.frontLeftName, m.frontLeftDirection);
        bl = motor(m.backLeftName, m.backLeftDirection);
        fr = motor(m.frontRightName, m.frontRightDirection);
        br = motor(m.backRightName, m.backRightDirection);

        applyParameters();
        while (opModeInInit()) {
            if (gamepad1.aWasPressed()) applyParameters();
            if (gamepad1.bWasPressed()) octoquad.setLocalizerPose(0, 0, 0f);
            read();
            show();
        }

        waitForCalibration();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                setDrivePower(0, 0, 0, 0);
                applyParameters();
                waitForCalibration();
                continue;
            }
            if (gamepad1.bWasPressed()) octoquad.setLocalizerPose(0, 0, 0f);
            drive();
            read();
            show();
        }

        setDrivePower(0, 0, 0, 0);
    }

    private DcMotorEx motor(String name, DcMotorEx.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void applyParameters() {
        octoquad.setSingleEncoderDirection(portX, directionX);
        octoquad.setSingleEncoderDirection(portY, directionY);
        octoquad.setAllLocalizerParameters(portX, portY,
                (float) ticksPerMmX, (float) ticksPerMmY,
                (float) tcpOffsetMmX, (float) tcpOffsetMmY,
                (float) imuHeadingScalar, velocityIntervalMs);
        // Parameters take effect only on a localizer reset.
        octoquad.resetLocalizerAndCalibrateIMU();
    }

    private void waitForCalibration() {
        OctoQuad.LocalizerStatus status;
        while (!isStopRequested() && (status = octoquad.getLocalizerStatus()) != OctoQuad.LocalizerStatus.RUNNING) {
            if (status == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
                throw new IllegalStateException("OctoQuad has no IMU (MK1?): the localizer needs an MK2");
            }
            telemetry.addLine("Waiting for IMU calibration: keep the robot still");
            telemetry.addData("Status", status);
            telemetry.update();
            sleep(20);
        }
    }

    private void read() {
        octoquad.readLocalizerDataAndAllEncoderData(poseRead, encodersRead);
        totalPackets++;
        lastPacketOk = poseRead.crcOk && encodersRead.crcOk;
        if (!lastPacketOk) {
            badPackets++;
            return;
        }
        OctoQuad.LocalizerDataBlock p = pose;
        pose = poseRead;
        poseRead = p;
        OctoQuad.EncoderDataBlock e = encoders;
        encoders = encodersRead;
        encodersRead = e;
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
                Math.max(Math.abs(frontRight), Math.abs(backRight)))));

        setDrivePower(frontLeft / max, backLeft / max, frontRight / max, backRight / max);
    }

    private void setDrivePower(double frontLeft, double backLeft, double frontRight, double backRight) {
        fl.setPower(frontLeft);
        bl.setPower(backLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
    }

    private void show() {
        telemetry.addLine("Sticks: drive  A: apply parameters and recalibrate (keep still)  B: zero pose");
        telemetry.addData("Status", pose.localizerStatus);
        telemetry.addData("Last packet CRC ok", lastPacketOk);
        telemetry.addData("x (in)", pose.posX_mm / MM_PER_INCH);
        telemetry.addData("y (in)", pose.posY_mm / MM_PER_INCH);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading_rad));
        telemetry.addData("x (mm)", pose.posX_mm);
        telemetry.addData("y (mm)", pose.posY_mm);
        telemetry.addData("heading (rad)", pose.heading_rad);
        telemetry.addData("vx (in/s)", pose.velX_mmS / MM_PER_INCH);
        telemetry.addData("vy (in/s)", pose.velY_mmS / MM_PER_INCH);
        telemetry.addData("heading vel (deg/s)", Math.toDegrees(pose.velHeading_radS));
        telemetry.addData("Raw X pod counts (push forward: rises)", encoders.positions[portX]);
        telemetry.addData("Raw Y pod counts (push left: rises)", encoders.positions[portY]);
        telemetry.addData("Bad packets", badPackets + "/" + totalPackets);
        telemetry.addData("Firmware", firmware);
        telemetry.update();
    }
}
