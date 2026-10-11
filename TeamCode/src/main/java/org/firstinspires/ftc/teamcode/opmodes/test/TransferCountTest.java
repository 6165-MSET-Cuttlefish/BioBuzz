package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Transfer Count Test", group = "Test")
@Config
public class TransferCountTest extends OpMode {
    public static String sensorName1 = "analog0";
    public static String sensorName2 = "analog1";
    public static String motor1Name = "transfer1";
    public static String motor2Name = "transfer2";
    // Must match the analog bounds written by Configure Laser Rangefinder.
    public static double maxRangeMm = 4000;
    public static double ballBelowMm = 60;
    public static double emptyAboveMm = 80;
    public static int stopAtCount = 2;
    public static double motor1Power = 1;
    public static double motor2Power = 1;
    public static boolean resetCount = false;
    public static boolean run = false;
    public static boolean reset = false;
    public int sensingSensor = -1;

    AnalogInput sensor1;
    AnalogInput sensor2;
    DcMotorEx motor1;

    int count;
    boolean ballInSensor;
    boolean wasRunning;

    @Override
    public void init() {
        sensor1 = hardwareMap.analogInput.get(sensorName1);
        sensor2 = hardwareMap.analogInput.get(sensorName2);
        motor1 = hardwareMap.get(DcMotorEx.class, motor1Name);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        double distanceMm1 = sensor1.getVoltage() / 3.3 * maxRangeMm;
        double distanceMm2 = sensor2.getVoltage() / 3.3 * maxRangeMm;


        // Between the two thresholds keeps the last state, so one ball can't count twice.
        if (!ballInSensor && (distanceMm1 < ballBelowMm || distanceMm2 < ballBelowMm)) {
            if (distanceMm1 < ballBelowMm) {
                sensingSensor = 1;
            }
            else {
                sensingSensor = 2;
            }
            ballInSensor = true;
            count++;
        } else if (ballInSensor && ((sensingSensor == 1 && distanceMm1 > emptyAboveMm) || (sensingSensor == 2 && distanceMm2 >emptyAboveMm))) {
            ballInSensor = false;
        }

        if (reset) {
            if (count >= stopAtCount) {
                motor1Power = 0;
            }
        }

        if (resetCount) {
            count = 0;
        }

        motor1.setPower(motor1Power);


        telemetry.addData("Count", count);
        telemetry.addData("Distance Sensor 1 (mm)", distanceMm1);
        telemetry.addData("Distance Sensor 2 (mm)", distanceMm2);
        telemetry.addData("Ball in sensor", ballInSensor);
        telemetry.addData("Running", run);
        telemetry.addData("Low Threshold", ballBelowMm);
        telemetry.addData("High Threshold", emptyAboveMm);
        telemetry.update();
    }
}
