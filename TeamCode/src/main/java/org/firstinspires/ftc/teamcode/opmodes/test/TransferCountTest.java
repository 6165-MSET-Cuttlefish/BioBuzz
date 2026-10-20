package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Transfer Count Test", group = "Test")
@Config
public class TransferCountTest extends OpMode {
    public static String upstreamSensorName = "analog0";
    public static String downstreamSensorName = "analog1";
    public static String motor1Name = "transfer1";
    public static double maxRangeMm = 1300;
    public static double diffThreshold = 25;
    public static int stopAtCount = 2;
    public static double motor1Power = 0;
    public static boolean resetCount = false;

    private final ElapsedTime runtime = new ElapsedTime();

    AnalogInput upstream;
    AnalogInput downstream;
    DcMotorEx motor1;

    int count;
    double pastDiff;
    boolean lastResetCount;

    @Override
    public void init() {
        upstream = hardwareMap.analogInput.get(upstreamSensorName);
        downstream = hardwareMap.analogInput.get(downstreamSensorName);
        motor1 = hardwareMap.get(DcMotorEx.class, motor1Name);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastResetCount = resetCount;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        pastDiff = (upstream.getVoltage() - downstream.getVoltage()) / 3.3 * maxRangeMm;
    }

    @Override
    public void loop() {
        runtime.reset();
        double mmA = upstream.getVoltage() / 3.3 * maxRangeMm;
        double mmB = downstream.getVoltage() / 3.3 * maxRangeMm;
        double diff = mmA - mmB;

        // Each ball's trailing edge (or the gap behind it) passing A pushes A − B up through the threshold once.
        if (pastDiff <= diffThreshold && diff > diffThreshold) {
            count++;
        }
        pastDiff = diff;

        if (resetCount != lastResetCount) count = 0;
        lastResetCount = resetCount;
        boolean stopped = stopAtCount > 0 && count >= stopAtCount;

        motor1.setPower(stopped ? 0 : motor1Power);

        while (runtime.milliseconds() < 20) {

        }

        telemetry.addData("Motor", stopped ? "STOPPED at count " + stopAtCount + ": flip resetCount to run again" : "running");
        telemetry.addData("Count", count);
        telemetry.addData("Upstream (mm)", mmA);
        telemetry.addData("Downstream (mm)", mmB);
        telemetry.addData("A - B (mm)", diff);
        telemetry.addData("Diff threshold (mm)", diffThreshold);
        telemetry.addData("motor current", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("loop time (ms)", runtime.milliseconds());
    }
}
