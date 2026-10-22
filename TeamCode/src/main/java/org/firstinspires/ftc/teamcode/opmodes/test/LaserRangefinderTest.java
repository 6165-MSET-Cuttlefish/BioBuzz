package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Laser Rangefinder Test", group = "Test")
@Config("Laser Rangefinder Test")
public class LaserRangefinderTest extends LinearOpMode {
    public static String sensorAName = "analog0";
    public static String sensorBName = "analog1";
    public static double maxRangeMm = 1300;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AnalogInput sensorA = hardwareMap.analogInput.get(sensorAName);
        AnalogInput sensorB = hardwareMap.analogInput.get(sensorBName);

        waitForStart();
        while (opModeIsActive()) {
            double voltageA = sensorA.getVoltage();
            double voltageB = sensorB.getVoltage();
            telemetry.addData("Sensor A Voltage (V)", voltageA);
            telemetry.addData("Sensor B Voltage (V)", voltageB);
            telemetry.addData("Sensor A Distance (mm)", voltageA / 3.3 * maxRangeMm);
            telemetry.addData("Sensor B Distance (mm)", voltageB / 3.3 * maxRangeMm);
            telemetry.update();
        }
    }
}
