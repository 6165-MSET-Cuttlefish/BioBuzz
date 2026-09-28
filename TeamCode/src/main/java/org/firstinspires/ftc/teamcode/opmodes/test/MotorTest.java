package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Motor Test", group = "Test")
@Config
public class MotorTest extends OpMode {
    public static String name = "motor";
    public static double power = 0;
    DcMotorEx motor;
    String lastName = name;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, name);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        motor.setPower(power);

        if (!lastName.equals(name)) {
            motor = hardwareMap.get(DcMotorEx.class, name);
            lastName = name;
        }

        telemetry.addData("Current (A)", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Power",  motor.getPower());
    }
}
