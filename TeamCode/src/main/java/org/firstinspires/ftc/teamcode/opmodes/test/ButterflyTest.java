package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="ButterflyDrive", group="Test")
@Config
public class ButterflyTest extends OpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Servo leftShifter, rightShifter;

    public static double LeftUpPos = 0.52;
    public static double LeftDownPos = 0.45;
    public static double RightUpPos  = 0.5;
    public static double RightDownPos = 0.56;
    public static double speedLimit = 2;

    private boolean tractionMode = false;
    private boolean lastState = false;

    @Override

    public void init() {
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        boolean currentState = gamepad1.aWasPressed();
        if (currentState && !lastState) {
            tractionMode = !tractionMode;
        }
        lastState = currentState;

        if (tractionMode) {
            leftShifter.setPosition(LeftDownPos);
            rightShifter.setPosition(RightDownPos);
        } else {
            leftShifter.setPosition(LeftUpPos);
            rightShifter.setPosition(RightUpPos);
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

        frontLeft.setPower(flPower/speedLimit);
        frontRight.setPower(frPower/speedLimit);
        backLeft.setPower(blPower/speedLimit);
        backRight.setPower(brPower/speedLimit);

        telemetry.addData("Mode", tractionMode ? "Traction" : "Omni");
    }
}
