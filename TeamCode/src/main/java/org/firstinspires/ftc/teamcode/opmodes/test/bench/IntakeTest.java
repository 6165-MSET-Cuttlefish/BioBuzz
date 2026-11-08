package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Intake Test", group = "Test")
@Config("Intake Test")
public class IntakeTest extends OpMode {
    public static double intakePower = 1.0;
    public static double outtakePower = 1.0;
    public static double driveScale = 1.0;

    private DcMotorEx fl, bl, fr, br;
    private DcMotorEx intake;

    @Override
    public void init() {
        fl = motor("fl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        bl = motor("bl", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        fr = motor("fr", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        br = motor("br", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        intake = motor("intake", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private DcMotorEx motor(String name, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPower) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPower);
        return motor;
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y * driveScale;
        double x = gamepad1.left_stick_x * driveScale;
        double rx = gamepad1.right_stick_x * driveScale;

        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)),
                Math.max(Math.abs(frontRight), Math.abs(backRight))));

        fl.setPower(frontLeft / max);
        bl.setPower(backLeft / max);
        fr.setPower(frontRight / max);
        br.setPower(backRight / max);

        double power;
        String mode;
        if (gamepad1.right_trigger > 0.1 || gamepad1.right_bumper) {
            power = intakePower;
            mode = "intake";
        } else if (gamepad1.left_trigger > 0.1 || gamepad1.left_bumper) {
            power = -outtakePower;
            mode = "outtake";
        } else {
            power = 0;
            mode = "off";
        }
        intake.setPower(power);

        telemetry.addLine("Left stick drive, right stick turn; RT/RB intake, LT/LB outtake");
        telemetry.addData("Intake", "%s (%.2f)", mode, power);
        telemetry.addData("Intake current (A)", intake.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void stop() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        intake.setPower(0);
    }
}
