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

@TeleOp(name = "Intake Transfer Shooter Test", group = "Test")
@Config("Intake Transfer Shooter Test")
public class IntakeTransferShooterTest extends OpMode {
    public static class FlywheelConfig {
        public String name = "flywheel";
        public String encoderName = "flywheel";
        public boolean reverse = false;
        public boolean reverseEncoder = false;
        public double ticksPerRev = 8192;
        public double rpmWindowMs = 50;
        public double power = 0;
        public double targetRpm = 0;
        public double kV = 0;
        public double kP = 0;
    }

    public static class HoodConfig {
        public String name = "hood";
        public double position = 0.5;
    }

    public static class MotorConfig {
        public String name;
        public boolean reverse = false;
        public double power = 0;

        MotorConfig(String name) {
            this.name = name;
        }
    }

    public static class BlockerConfig {
        public String name = "blocker";
        public double openPosition = 0.6;
        public double closedPosition = 0.53;
        public boolean open = true;
    }

    public static FlywheelConfig flywheel = new FlywheelConfig();
    public static HoodConfig hood = new HoodConfig();
    public static MotorConfig intake = new MotorConfig("intake");
    public static MotorConfig transfer = new MotorConfig("transfer");
    public static BlockerConfig blocker = new BlockerConfig();

    private final ArrayDeque<double[]> history = new ArrayDeque<>();
    private List<LynxModule> hubs;
    private DcMotorEx flywheelMotor;
    private DcMotorEx encoder;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private Servo hoodServo;
    private Servo blockerServo;
    private VoltageSensor battery;
    private long lastLoopNs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        flywheelMotor = hardwareMap.get(DcMotorEx.class, flywheel.name);
        encoder = hardwareMap.get(DcMotorEx.class, flywheel.encoderName);
        hoodServo = hardwareMap.get(Servo.class, hood.name);
        intakeMotor = hardwareMap.get(DcMotorEx.class, intake.name);
        transferMotor = hardwareMap.get(DcMotorEx.class, transfer.name);
        blockerServo = hardwareMap.get(Servo.class, blocker.name);
        battery = hardwareMap.voltageSensor.iterator().next();
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        double rpm = rpm((t0 + (t1 - t0) / 2) / 1e9, flywheel.reverseEncoder ? -ticks : ticks);

        boolean closedLoop = flywheel.targetRpm > 0;
        double out = closedLoop
                ? Range.clip(flywheel.kV * flywheel.targetRpm + flywheel.kP * (flywheel.targetRpm - rpm), 0, 1)
                : flywheel.power;
        flywheelMotor.setDirection(direction(flywheel.reverse));
        flywheelMotor.setPower(out);
        hoodServo.setPosition(hood.position);
        intakeMotor.setDirection(direction(intake.reverse));
        transferMotor.setDirection(direction(transfer.reverse));
        intakeMotor.setPower(intake.power);
        transferMotor.setPower(transfer.power);
        if (gamepad1.aWasPressed()) blocker.open = !blocker.open;
        double blockerPosition = blocker.open ? blocker.openPosition : blocker.closedPosition;
        blockerServo.setPosition(blockerPosition);

        telemetry.addData("Target RPM", flywheel.targetRpm);
        telemetry.addData("RPM", rpm);
        telemetry.addData("Error (RPM)", closedLoop ? flywheel.targetRpm - rpm : 0);
        telemetry.addData("Power", out);
        if (!closedLoop && rpm > 0) telemetry.addData("kV estimate (power / RPM)", "%.7f", out / rpm);
        if (rpm < -50 && out > 0.1) telemetry.addLine("RPM reads negative while driven forward: flip reverseEncoder");
        telemetry.addData("Hood position", hood.position);
        telemetry.addData("Intake power", intake.power);
        telemetry.addData("Transfer power", transfer.power);
        telemetry.addData("Blocker", "%s (%.3f), gamepad1 A toggles", blocker.open ? "open" : "closed", blockerPosition);
        telemetry.addData("Flywheel current (A)", flywheelMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake current (A)", intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Transfer current (A)", transferMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Battery (V)", battery.getVoltage());
        telemetry.addData("Encoder ticks", ticks);
        telemetry.addData("Loop (ms)", (t0 - lastLoopNs) / 1e6);
        lastLoopNs = t0;
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        flywheelMotor.setPower(0);
    }

    private static DcMotorSimple.Direction direction(boolean reverse) {
        return reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private double rpm(double seconds, int ticks) {
        history.addLast(new double[] {seconds, ticks});
        while (history.size() > 2 && seconds - history.peekFirst()[0] > flywheel.rpmWindowMs / 1000) history.removeFirst();
        if (history.size() < 2) return 0;
        double[] oldest = history.peekFirst();
        return (ticks - oldest[1]) / (seconds - oldest[0]) / flywheel.ticksPerRev * 60;
    }
}
