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

    private DcMotorEx pollenFlywheel;
    private DcMotorEx nectarFlywheel;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private Servo pollenHood;
    private Servo nectarHood;
    private Servo blocker;

    @Override
    public void init() {
        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenFlywheel");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarFlywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        pollenHood = hardwareMap.get(Servo.class, "pollenHood");
        nectarHood = hardwareMap.get(Servo.class, "nectarHood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        pollenFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pollenFlywheel.setPower(flywheelPower);
        nectarFlywheel.setPower(flywheelPower);
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
        pollenHood.setPosition(hoodPosition);
        nectarHood.setPosition(hoodPosition);
        if (gamepad1.aWasPressed()) blockerOpen = !blockerOpen;
        double blockerPosition = blockerOpen ? blockerOpenPosition : blockerClosedPosition;
        blocker.setPosition(blockerPosition);

        telemetry.addData("Flywheel power", flywheelPower);
        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Transfer power", transferPower);
        telemetry.addData("Hood position", hoodPosition);
        telemetry.addData("Blocker", "%s (%.3f), gamepad1 A toggles", blockerOpen ? "open" : "closed", blockerPosition);
        telemetry.addData("Pollen flywheel current (A)", pollenFlywheel.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Nectar flywheel current (A)", nectarFlywheel.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake current (A)", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Transfer current (A)", transfer.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void stop() {
        pollenFlywheel.setPower(0);
        nectarFlywheel.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
