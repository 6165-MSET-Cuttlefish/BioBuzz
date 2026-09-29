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
    String lastName;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, name);
        lastName = name;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Zero the old motor before rebinding, or it keeps running at the last power.
        if (!lastName.equals(name)) {
            motor.setPower(0);
            motor = hardwareMap.get(DcMotorEx.class, name);
            lastName = name;
        }

        motor.setPower(power);

        telemetry.addData("Current (A)", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Power",  motor.getPower());
    }
}
