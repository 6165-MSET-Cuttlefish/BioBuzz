package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Two Motor Test", group = "Test")
@Config
public class TwoMotorTest extends OpMode {
    public static String name1 = "motor1";
    public static String name2 = "motor2";
    public static double power = 0;
    public static boolean reverse2 = false;
    DcMotorEx motor1, motor2;
    String lastName1 = name1, lastName2 = name2;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, name1);
        motor2 = hardwareMap.get(DcMotorEx.class, name2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Zero the old motor before rebinding, or it keeps running at the last power.
        if (!lastName1.equals(name1)) {
            motor1.setPower(0);
            motor1 = hardwareMap.get(DcMotorEx.class, name1);
            lastName1 = name1;
        }
        if (!lastName2.equals(name2)) {
            motor2.setPower(0);
            motor2 = hardwareMap.get(DcMotorEx.class, name2);
            lastName2 = name2;
        }

        motor1.setPower(power);
        motor2.setPower(reverse2 ? -power : power);

        telemetry.addData("Motor 1 Current (A)", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 1 Power", motor1.getPower());
        telemetry.addData("Motor 2 Current (A)", motor2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 2 Power", motor2.getPower());
    }
}
