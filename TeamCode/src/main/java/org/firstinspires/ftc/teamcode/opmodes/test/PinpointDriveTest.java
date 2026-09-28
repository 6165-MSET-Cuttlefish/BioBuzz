package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

@TeleOp(name = "Pinpoint Drive Test", group = "Test")
public class PinpointDriveTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx fl, bl, fr, br;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BettaConstants.MecanumSettings m = BettaConstants.mecanum;
        fl = motor(m.frontLeftName, m.frontLeftDirection);
        bl = motor(m.backLeftName, m.backLeftDirection);
        fr = motor(m.frontRightName, m.frontRightDirection);
        br = motor(m.backRightName, m.backRightDirection);

        BettaConstants.PinpointSettings p = BettaConstants.pinpoint;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, p.name);
        pinpoint.setOffsets(p.xPodOffset, p.yPodOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(p.podType);
        pinpoint.setEncoderDirections(p.xPodDirection, p.yPodDirection);
        pinpoint.resetPosAndIMU(); // robot must be stationary (IMU recalibration)

        while (opModeInInit()) update();
        while (opModeIsActive()) {
            drive();
            update();
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    private DcMotorEx motor(String name, DcMotorEx.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void update() {
        pinpoint.update();
        telemetry.addData("x (in)", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y (in)", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.update();
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

        fl.setPower(frontLeft / max);
        bl.setPower(backLeft / max);
        fr.setPower(frontRight / max);
        br.setPower(backRight / max);
    }
}
