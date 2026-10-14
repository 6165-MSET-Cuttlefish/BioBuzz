package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Laser Rangefinder Test", group = "Test")
@Config
public class LaserRangefinderTest extends LinearOpMode {
    public static String pin0Name = "analog0";
    public static String pin1Name = "analog1";
    // Must match the analog bounds the sensor was configured with over I2C.
    public static double maxRangeMm = 1300;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AnalogInput pin0 = hardwareMap.analogInput.get(pin0Name);
        AnalogInput pin1 = hardwareMap.analogInput.get(pin1Name);

        waitForStart();
        while (opModeIsActive()) {
            double voltage0 = pin0.getVoltage();
            double voltage1 = pin1.getVoltage();
            telemetry.addData("voltage 0", voltage0);
            telemetry.addData("voltage 1", voltage1);
            telemetry.addData("distance 0 (mm)", voltage0 / 3.3 * maxRangeMm);
            telemetry.addData("distance 1 (mm)", voltage1 / 3.3 * maxRangeMm);
            telemetry.update();
        }
    }
}
