package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BioBuzzConstants;

@TeleOp(name = "OctoQuad Localizer Test", group = "Test")
public class OctoQuadLocalizerTest extends LinearOpMode {
    private final OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
    private GuardedOctoQuadLocalizer odometry;
    private OctoQuad octoquad;
    private DcMotorEx fl, bl, fr, br;
    private String firmware;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        BioBuzzConstants.MecanumSettings m = BioBuzzConstants.mecanum;
        fl = motor(Drivetrain.FRONT_LEFT, m.frontLeftDirection);
        bl = motor(Drivetrain.BACK_LEFT, m.backLeftDirection);
        fr = motor(Drivetrain.FRONT_RIGHT, m.frontRightDirection);
        br = motor(Drivetrain.BACK_RIGHT, m.backRightDirection);

        applySettings();
        while (opModeInInit()) {
            if (gamepad1.aWasPressed()) applySettings();
            if (gamepad1.bWasPressed()) odometry.setPose(Pose.zero());
            read();
            show();
        }
        if (isStopRequested()) return;

        odometry.awaitRunning(this::isStopRequested);
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                setDrivePower(0, 0, 0, 0);
                applySettings();
                odometry.awaitRunning(this::isStopRequested);
                continue;
            }
            if (gamepad1.bWasPressed()) odometry.setPose(Pose.zero());
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

    // A new localizer writes BioBuzzConstants.octoQuad to the board and recalibrates its IMU, exactly as every INIT does.
    private void applySettings() {
        odometry = BioBuzzConstants.localizer(hardwareMap);
        octoquad = hardwareMap.get(OctoQuad.class, BioBuzzConstants.octoQuad.name);
        firmware = octoquad.getFirmwareVersionString();
    }

    private void read() {
        odometry.update();
        octoquad.readAllEncoderData(encoders);
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
        BioBuzzConstants.OctoQuadSettings s = BioBuzzConstants.octoQuad;
        Pose pose = odometry.state().pose();
        Velocity velocity = odometry.state().velocity();
        telemetry.addLine("Sticks: drive  A: apply BioBuzz Constants and recalibrate (keep still)  B: zero pose");
        telemetry.addData("Status", odometry.status());
        telemetry.addData("Firmware", firmware);
        telemetry.addData("x (in)", "%.2f", pose.x());
        telemetry.addData("y (in)", "%.2f", pose.y());
        telemetry.addData("heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("vx (in/s)", "%.1f", velocity.vx);
        telemetry.addData("vy (in/s)", "%.1f", velocity.vy);
        telemetry.addData("heading vel (deg/s)", "%.1f", Math.toDegrees(velocity.omega));
        telemetry.addData("Forward pod, channel " + s.forwardPodChannel + " (push forward: rises)",
                encoders.positions[s.forwardPodChannel]);
        telemetry.addData("Strafe pod, channel " + s.strafePodChannel + " (push left: rises)",
                encoders.positions[s.strafePodChannel]);
        telemetry.addData("Encoder read CRC ok", encoders.crcOk);
        telemetry.update();
    }
}
