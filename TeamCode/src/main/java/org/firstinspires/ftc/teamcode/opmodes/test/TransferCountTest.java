package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Transfer Count Test", group = "Test")
@Config
public class TransferCountTest extends OpMode {
    public static String sensorName1 = "analog0";
    public static String sensorName2 = "analog1";
    public static String motor1Name = "transfer1";
    public static String motor2Name = "transfer2";
    // Must match the analog bounds written by Configure Laser Rangefinder.
    public static double maxRangeMm = 1300;
    public static double ballBelowMmCenter = 30;
    public static double emptyAboveMmCenter = 80;
    public static double idealtime = 40;
    public static double ballBelowMmLow = 50;
    public static double emptyAboveMmLow = 80;
    public static int stopAtCount = 2;
    public static double motor1Power = 0;
    public static double motor2Power = 1;
    public static double threshold = -40;
    public static double highestmm1;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stawp = new ElapsedTime();
    public static boolean resetCount = false;
    public static boolean run = false;
    public static boolean reset = false;
    public static boolean check = false;
    public int sensingSensor = -1;

    AnalogInput sensor1;
    AnalogInput sensor2;
    DcMotorEx motor1;

    int count;
    boolean ballInSensor;
    double pastmm1;
    double pastmm2;
    boolean wasRunning;

    @Override
    public void init() {
        sensor1 = hardwareMap.analogInput.get(sensorName1);
        sensor2 = hardwareMap.analogInput.get(sensorName2);
        motor1 = hardwareMap.get(DcMotorEx.class, motor1Name);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void start() {
        runtime.reset();
        pastmm1 = sensor1.getVoltage() / 3.3 * maxRangeMm;
        pastmm2 = sensor2.getVoltage() / 3.3 * maxRangeMm;
        highestmm1 = sensor1.getVoltage() / 3.3 * maxRangeMm;


    }

    @Override
    public void loop() {
        runtime.reset();
        double distanceMm1 = sensor1.getVoltage() / 3.3 * maxRangeMm;
        double distanceMm2 = sensor2.getVoltage() / 3.3 * maxRangeMm;

        if (!ballInSensor && (distanceMm1 < ballBelowMmLow || distanceMm2 < ballBelowMmLow)) {
            stawp.reset();
            ballInSensor = true;
            count++;
            if (distanceMm1 < ballBelowMmLow) {
                sensingSensor = 1;
            }
            else {
                sensingSensor = 2;
            }
        }
        if (ballInSensor && stawp.milliseconds() > idealtime && ((sensingSensor == 1 && distanceMm1 > emptyAboveMmLow) || (sensingSensor == 2 && distanceMm2 > emptyAboveMmLow))) {
            ballInSensor = false;
        }



        /*
        if (check && distanceMm1 - highestmm1 < threshold) {
            check = false;
            highestmm1 = distanceMm1;
            count++;
        }
        else if (distanceMm1 - pastmm1 > 0) {
            check = true;
            if (distanceMm1 > highestmm1) {
                highestmm1 = distanceMm1;
            }

        }
         */

        pastmm1 = distanceMm1;
        pastmm2 = distanceMm2;

        // Between the two thresholds keeps the last state, so one ball can't count twice.
      /*  if (!ballInSensor && (distanceMm1 < ballBelowMmCenter || distanceMm2 < ballBelowMmLow)) {
            if (distanceMm1 < ballBelowMmCenter) {
                sensingSensor = 1;
            }
            else {
                sensingSensor = 2;
            }
            ballInSensor = true;
            count++;
        } else if (ballInSensor && ((sensingSensor == 1 && distanceMm1 > emptyAboveMmCenter) || (sensingSensor == 2 && distanceMm2 > emptyAboveMmLow))) {
            ballInSensor = false;
        }
       */

        if (reset) {
            if (count >= stopAtCount) {
                motor1Power = 0;
            }
        }

        if (resetCount) {
            count = 0;
        }

        motor1.setPower(motor1Power);

        while (runtime.milliseconds() < 20) {

        }

        telemetry.addData("Count", count);
        telemetry.addData("Distance Sensor 1 (mm)", distanceMm1);
        telemetry.addData("Distance Sens    or 2 (mm)", distanceMm2);
        telemetry.addData("Ball in sensor", ballInSensor);
        telemetry.addData("motor current", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("loop time (ms)",  runtime.milliseconds());
        telemetry.addData("Running", run);
        telemetry.addData("wait time for next ball", stawp.milliseconds());
        telemetry.update();
    }
}
