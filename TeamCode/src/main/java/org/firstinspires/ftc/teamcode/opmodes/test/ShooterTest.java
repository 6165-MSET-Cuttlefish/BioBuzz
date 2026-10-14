package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayDeque;
import java.util.List;

@TeleOp(name = "Shooter Test", group = "Test")
@Config
public class ShooterTest extends OpMode {
    public static String flywheelName = "flywheel";
    public static String encoderName = "flywheel";
    public static String hoodName = "hood";
    public static boolean reverseFlywheel = false;
    public static boolean reverseEncoder = false;
    public static double ticksPerRev = 8192;
    public static double rpmWindowMs = 50;
    public static double hoodPosition = 0.5;
    public static double power = 0;
    public static double targetRpm = 0;
    public static double kV = 0;
    public static double kP = 0;

    private final ArrayDeque<double[]> history = new ArrayDeque<>();
    private List<LynxModule> hubs;
    private DcMotorEx flywheel;
    private DcMotorEx encoder;
    private Servo hood;
    private VoltageSensor battery;
    private long lastLoopNs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        flywheel = hardwareMap.get(DcMotorEx.class, flywheelName);
        encoder = hardwareMap.get(DcMotorEx.class, encoderName);
        hood = hardwareMap.get(Servo.class, hoodName);
        battery = hardwareMap.voltageSensor.iterator().next();
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        lastLoopNs = System.nanoTime();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        long t0 = System.nanoTime();
        int ticks = encoder.getCurrentPosition();
        long t1 = System.nanoTime();
        double rpm = rpm((t0 + (t1 - t0) / 2) / 1e9, reverseEncoder ? -ticks : ticks);

        boolean closedLoop = targetRpm > 0;
        double out = closedLoop ? Range.clip(kV * targetRpm + kP * (targetRpm - rpm), 0, 1) : power;
        flywheel.setDirection(reverseFlywheel ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel.setPower(out);
        hood.setPosition(hoodPosition);

        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("RPM", rpm);
        telemetry.addData("Error (RPM)", closedLoop ? targetRpm - rpm : 0);
        telemetry.addData("Power", out);
        if (!closedLoop && rpm > 0) telemetry.addData("kV estimate (power / RPM)", "%.7f", out / rpm);
        if (rpm < -50 && out > 0.1) telemetry.addLine("RPM reads negative while driven forward: flip reverseEncoder");
        telemetry.addData("Hood position", hoodPosition);
        telemetry.addData("Flywheel current (A)", flywheel.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Battery (V)", battery.getVoltage());
        telemetry.addData("Encoder ticks", ticks);
        telemetry.addData("Loop (ms)", (t0 - lastLoopNs) / 1e6);
        lastLoopNs = t0;
    }

    @Override
    public void stop() {
        flywheel.setPower(0);
    }

    private double rpm(double seconds, int ticks) {
        history.addLast(new double[] {seconds, ticks});
        while (history.size() > 2 && seconds - history.peekFirst()[0] > rpmWindowMs / 1000) history.removeFirst();
        if (history.size() < 2) return 0;
        double[] oldest = history.peekFirst();
        return (ticks - oldest[1]) / (seconds - oldest[0]) / ticksPerRev * 60;
    }
}
