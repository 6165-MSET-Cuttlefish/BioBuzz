package org.firstinspires.ftc.teamcode.opmodes.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Butterfly Test", group="Test")
@Config("Butterfly Test")
public class ButterflyTest extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Servo leftShifter, rightShifter;

    public static double leftUpPos = 0.85;
    public static double leftDownPos = 0.1;
    public static double rightUpPos  = 0.25;
    public static double rightDownPos = 1;
    public static double powerScale = 0.5;

    private boolean tractionMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft   = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight  = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft    = hardwareMap.get(DcMotorEx.class, "bl");
        backRight   = hardwareMap.get(DcMotorEx.class, "br");

        leftShifter  = hardwareMap.get(Servo.class, "left_shifter");
        rightShifter = hardwareMap.get(Servo.class, "right_shifter");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                tractionMode = !tractionMode;

                if (tractionMode) {
                    leftShifter.setPosition(leftDownPos);
                    rightShifter.setPosition(rightDownPos);
                } else {
                    leftShifter.setPosition(leftUpPos);
                    rightShifter.setPosition(rightUpPos);
                }

            }

            double x  = -gamepad1.left_stick_y;
            double y =  gamepad1.left_stick_x;
            double rx   =  gamepad1.right_stick_x;

            double flPower, frPower, blPower, brPower;

            if (tractionMode) {
                flPower = x + rx;
                frPower = x - rx;
                blPower = x + rx;
                brPower = x - rx;
            } else {
                flPower = x + y + rx;
                frPower = x - y - rx;
                blPower = x - y + rx;
                brPower = x + y - rx;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            if (denominator > 1.0) {
                flPower /= denominator;
                frPower /= denominator;
                blPower /= denominator;
                brPower /= denominator;
            }

            frontLeft.setPower(flPower * powerScale);
            frontRight.setPower(frPower * powerScale);
            backLeft.setPower(blPower * powerScale);
            backRight.setPower(brPower * powerScale);

            telemetry.addData("Mode", tractionMode ? "Traction" : "Mecanum");
            telemetry.update();
        }
    }
}
