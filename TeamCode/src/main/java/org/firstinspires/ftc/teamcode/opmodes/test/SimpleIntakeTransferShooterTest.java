package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Simple Intake Transfer Shooter Test", group = "Test")
@Config("Simple Intake Transfer Shooter Test")
public class SimpleIntakeTransferShooterTest extends OpMode {
    public static double flywheelPower = 0;
    public static double hoodPosition = 0.5;
    public static double intakePower = 0;
    public static double transferPower = 0;
    public static double blockerOpenPosition = 0.6;
    public static double blockerClosedPosition = 0.53;
    public static boolean blockerOpen = true;

    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private Servo hood;
    private Servo blocker;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        flywheel.setPower(flywheelPower);
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
        hood.setPosition(hoodPosition);
        if (gamepad1.aWasPressed()) blockerOpen = !blockerOpen;
        double blockerPosition = blockerOpen ? blockerOpenPosition : blockerClosedPosition;
        blocker.setPosition(blockerPosition);

        telemetry.addData("Flywheel power", flywheelPower);
        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Transfer power", transferPower);
        telemetry.addData("Hood position", hoodPosition);
        telemetry.addData("Blocker", "%s (%.3f), gamepad1 A toggles", blockerOpen ? "open" : "closed", blockerPosition);
        telemetry.addData("Flywheel current (A)", flywheel.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake current (A)", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Transfer current (A)", transfer.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void stop() {
        flywheel.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
