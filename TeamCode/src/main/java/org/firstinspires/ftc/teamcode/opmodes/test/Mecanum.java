package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

@TeleOp(name = "Mecanum", group = "Test")
public class Mecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Drivetrain.FRONT_LEFT);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Drivetrain.BACK_LEFT);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Drivetrain.FRONT_RIGHT);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Drivetrain.BACK_RIGHT);

        BettaConstants.MecanumSettings m = BettaConstants.mecanum;
        frontLeftMotor.setDirection(m.frontLeftDirection);
        backLeftMotor.setDirection(m.backLeftDirection);
        frontRightMotor.setDirection(m.frontRightDirection);
        backRightMotor.setDirection(m.backRightDirection);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
