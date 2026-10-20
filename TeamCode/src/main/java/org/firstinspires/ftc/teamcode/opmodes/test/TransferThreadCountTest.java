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
import org.firstinspires.ftc.teamcode.modules.TransferCounter;

@TeleOp(name = "Transfer Thread Count Test", group = "Test")
@Config
public class TransferThreadCountTest extends OpMode {
    public static String upstreamSensorName = "analog0";
    public static String downstreamSensorName = "analog1";
    public static String motor1Name = "transfer1";
    public static double maxRangeMm = 1300;
    public static double diffThreshold = 25;
    public static double hysteresisMm = 10;
    public static boolean runThread = true;
    public static double pollMs = 10;
    public static double loopMs = 20;
    public static int stopAtCount = 2;
    public static double motor1Power = 0;
    public static boolean resetCount = false;
    public static boolean crashInLoop = false;
    public static boolean crashInStop = false;

    private final ElapsedTime runtime = new ElapsedTime();

    AnalogInput upstream;
    AnalogInput downstream;
    DcMotorEx motor1;
    TransferCounter counter;
    final TransferCounter.Edge loopEdge = new TransferCounter.Edge();

    int loopCount;
    int lastPosition = 0;
    boolean lastResetCount;

    @Override
    public void init() {
        upstream = hardwareMap.analogInput.get(upstreamSensorName);
        downstream = hardwareMap.analogInput.get(downstreamSensorName);
        motor1 = hardwareMap.get(DcMotorEx.class, motor1Name);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (runThread) {
            counter = new TransferCounter(hardwareMap, upstreamSensorName, downstreamSensorName, maxRangeMm, pollMs);
            counter.setThreshold(diffThreshold, hysteresisMm);
        }
        lastResetCount = resetCount;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        if (counter != null) counter.start();
    }

    @Override
    public void loop() {
        if (crashInLoop) throw new IllegalStateException("crashInLoop is on: deliberate crash in loop() to test the TransferCounter thread");
        runtime.reset();
        lastPosition = motor1.getCurrentPosition();
        if (counter != null) counter.setThreshold(diffThreshold, hysteresisMm);

        double mmA = upstream.getVoltage() / 3.3 * maxRangeMm;
        double mmB = downstream.getVoltage() / 3.3 * maxRangeMm;
        double diff = mmA - mmB;
        if (loopEdge.update(diff, diffThreshold, hysteresisMm)) loopCount++;

        if (resetCount != lastResetCount) {
            loopCount = 0;
            if (counter != null) counter.resetCount();
        }
        lastResetCount = resetCount;
        int count = counter != null ? counter.count() : loopCount;
        boolean stopped = stopAtCount > 0 && count >= stopAtCount;

        motor1.setPower(stopped ? 0 : motor1Power);
        double workMs = runtime.milliseconds();

        while (runtime.milliseconds() < loopMs) {

        }

        telemetry.addData("Motor", stopped ? "STOPPED at count " + stopAtCount + ": flip resetCount to run again" : "running");
        telemetry.addData("Count (thread)", counter != null ? count : "off");
        telemetry.addData("Count (loop)", loopCount);
        telemetry.addData("Upstream (mm)", mmA);
        telemetry.addData("Downstream (mm)", mmB);
        telemetry.addData("A - B (mm)", diff);
        telemetry.addData("Diff threshold (mm)", diffThreshold);
        if (counter != null) {
            TransferCounter.Stats s = counter.stats();
            if (s != null) {
                telemetry.addData("A - B thread (mm)", s.mmA - s.mmB);
                telemetry.addData("poll Hz", s.polls / s.seconds);
                telemetry.addData("new A values/s", s.newValues / s.seconds);
                telemetry.addData(String.format("late polls (>%.1f ms)", counter.latePollMs()), s.latePolls);
                telemetry.addData("max poll gap (ms)", s.maxGapMs);
                telemetry.addData("bulk read mean (ms)", s.meanCommandMs);
            }
        }
        telemetry.addData("motor current", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("loop work (ms)", workMs);
        telemetry.addData("loop time (ms)", runtime.milliseconds());
        telemetry.addData("motor RPM", 60000 * (motor1.getCurrentPosition()-lastPosition) / runtime.milliseconds() / 145.1);
    }

    @Override
    public void stop() {
        if (crashInStop) throw new IllegalStateException("crashInStop is on: deliberate crash in stop() before stopping the TransferCounter thread");
        if (counter != null) counter.stop();
    }
}
