package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayDeque;
import java.util.List;
import java.util.concurrent.locks.LockSupport;

@TeleOp(name = "Intake Transfer Shooter Test", group = "Test")
@Config("Intake Transfer Shooter Test")
public class IntakeTransferShooterTest extends OpMode {
    public static class NectarShooter {
        public double targetRpm = 0;
        public double power = 0;
        public double kV = 0;
        public double kP = 0;
        public double hoodPosition = 0.5;
        public boolean reverseFlywheel = false;
        public boolean reverseEncoder = false;
    }

    public static class PollenShooter {
        public double targetRpm = 0;
        public double power = 0;
        public double kV = 0;
        public double kP = 0;
        public double hoodPosition = 0.5;
        public boolean reverseFlywheel = false;
        public boolean reverseEncoder = false;
    }

    public static class Intake {
        public double power = 0;
        public boolean reverse = false;
    }

    public static class Transfer {
        public double power = 0;
        public boolean reverse = false;
        public double melonboticsPower = 0;
        public boolean reverseMelonbotics = false;
    }

    public static class Blocker {
        public double openPosition = 0.6;
        public double closedPosition = 0.53;
        public boolean open = true;
    }

    public static NectarShooter nectar = new NectarShooter();
    public static PollenShooter pollen = new PollenShooter();
    public static Intake intake = new Intake();
    public static Transfer transfer = new Transfer();
    public static Blocker blocker = new Blocker();

    public static double ticksPerRev = 8192;
    public static double rpmWindowMs = 50;
    public static double loopMs = 15;

    private static final long LATE_TOLERANCE_NS = 1_000_000;

    private final ArrayDeque<double[]> nectarHistory = new ArrayDeque<>();
    private final ArrayDeque<double[]> pollenHistory = new ArrayDeque<>();
    private List<LynxModule> hubs;
    private DcMotorEx nectarFlywheel;
    private DcMotorEx pollenFlywheel;
    private ServoImplEx nectarHood;
    private ServoImplEx pollenHood;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private CRServoImplEx melonbotics;
    private ServoImplEx blockerServo;
    private VoltageSensor battery;
    private long lastLoopNs;
    private long nextLoopNs;
    private int lateLoops;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarFlywheel");
        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenFlywheel");
        nectarHood = hardwareMap.get(ServoImplEx.class, "nectarHood");
        pollenHood = hardwareMap.get(ServoImplEx.class, "pollenHood");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        melonbotics = hardwareMap.get(CRServoImplEx.class, "melonbotics");
        blockerServo = hardwareMap.get(ServoImplEx.class, "blocker");
        battery = hardwareMap.voltageSensor.iterator().next();
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        lastLoopNs = System.nanoTime();
        nextLoopNs = lastLoopNs + loopPeriodNs();
    }

    @Override
    public void loop() {
        long waitNs;
        while ((waitNs = nextLoopNs - System.nanoTime()) > 0) LockSupport.parkNanos(waitNs);
        long loopStartNs = System.nanoTime();
        if (loopStartNs - nextLoopNs > LATE_TOLERANCE_NS) {
            lateLoops++;
            nextLoopNs = loopStartNs;
        }
        nextLoopNs += loopPeriodNs();
        for (LynxModule hub : hubs) hub.clearBulkCache();

        double nectarRpm = rpm(nectarHistory, nectarFlywheel, nectar.reverseEncoder);
        double pollenRpm = rpm(pollenHistory, pollenFlywheel, pollen.reverseEncoder);
        double nectarOut = power(nectar.targetRpm, nectar.power, nectar.kV, nectar.kP, nectarRpm);
        double pollenOut = power(pollen.targetRpm, pollen.power, pollen.kV, pollen.kP, pollenRpm);

        nectarFlywheel.setDirection(direction(nectar.reverseFlywheel));
        pollenFlywheel.setDirection(direction(pollen.reverseFlywheel));
        nectarFlywheel.setPower(nectarOut);
        pollenFlywheel.setPower(pollenOut);
        nectarHood.setPosition(nectar.hoodPosition);
        pollenHood.setPosition(pollen.hoodPosition);

        intakeMotor.setDirection(direction(intake.reverse));
        transferMotor.setDirection(direction(transfer.reverse));
        intakeMotor.setPower(intake.power);
        transferMotor.setPower(transfer.power);
        melonbotics.setDirection(direction(transfer.reverseMelonbotics));
        melonbotics.setPower(transfer.melonboticsPower);
        double blockerPosition = blocker.open ? blocker.openPosition : blocker.closedPosition;
        blockerServo.setPosition(blockerPosition);

        telemetry.addData("Nectar target RPM", nectar.targetRpm);
        telemetry.addData("Nectar RPM", nectarRpm);
        telemetry.addData("Nectar power", nectarOut);
        if (nectar.targetRpm <= 0 && nectarRpm > 0) telemetry.addData("Nectar kV estimate (power / RPM)", "%.7f", nectarOut / nectarRpm);
        telemetry.addData("Nectar current (A)", nectarFlywheel.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Pollen target RPM", pollen.targetRpm);
        telemetry.addData("Pollen RPM", pollenRpm);
        telemetry.addData("Pollen power", pollenOut);
        if (pollen.targetRpm <= 0 && pollenRpm > 0) telemetry.addData("Pollen kV estimate (power / RPM)", "%.7f", pollenOut / pollenRpm);
        telemetry.addData("Pollen current (A)", pollenFlywheel.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Intake power", intake.power);
        telemetry.addData("Intake current (A)", intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Transfer power", transfer.power);
        telemetry.addData("Transfer current (A)", transferMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Melonbotics power", transfer.melonboticsPower);
        telemetry.addData("Blocker", "%s (%.3f)", blocker.open ? "open" : "closed", blockerPosition);

        telemetry.addData("Battery (V)", battery.getVoltage());
        telemetry.addData("Loop (ms)", (loopStartNs - lastLoopNs) / 1e6);
        telemetry.addData("Late loops (over loopMs + 1 ms)", lateLoops);
        lastLoopNs = loopStartNs;
    }

    @Override
    public void stop() {
        nectarFlywheel.setPower(0);
        pollenFlywheel.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        melonbotics.setPower(0);
    }

    private static double power(double targetRpm, double openLoopPower, double kV, double kP, double rpm) {
        if (targetRpm <= 0) return openLoopPower;
        return Range.clip(kV * targetRpm + kP * (targetRpm - rpm), 0, 1);
    }

    private static long loopPeriodNs() {
        return (long) (loopMs * 1e6);
    }

    private static DcMotorSimple.Direction direction(boolean reverse) {
        return reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static double rpm(ArrayDeque<double[]> history, DcMotorEx motor, boolean reverse) {
        long before = System.nanoTime();
        int ticks = motor.getCurrentPosition();
        double seconds = (before + (System.nanoTime() - before) / 2) / 1e9;
        if (reverse) ticks = -ticks;

        history.addLast(new double[] {seconds, ticks});
        while (history.size() > 2 && seconds - history.peekFirst()[0] > rpmWindowMs / 1000) history.removeFirst();
        if (history.size() < 2) return 0;
        double[] oldest = history.peekFirst();
        return (ticks - oldest[1]) / (seconds - oldest[0]) / ticksPerRev * 60;
    }
}
