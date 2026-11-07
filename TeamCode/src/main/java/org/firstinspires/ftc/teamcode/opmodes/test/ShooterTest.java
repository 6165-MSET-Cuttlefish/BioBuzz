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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayDeque;
import java.util.List;

@TeleOp(name = "Shooter Test", group = "Test")
@Config("Shooter Test")
public class ShooterTest extends OpMode {
    public static class ShooterSettings {
        public double targetRpm = 0;
        public double power = 0;
        public double kV = 0;
        public double kP = 0;
        public double hoodPosition = 0.5;
        public boolean reverseFlywheel = false;
        public boolean reverseEncoder = false;
    }

    public static ShooterSettings nectar = new ShooterSettings();
    public static ShooterSettings pollen = new ShooterSettings();

    public static double ticksPerRev = 8192;
    public static double rpmWindowMs = 50;

    private final ArrayDeque<double[]> nectarHistory = new ArrayDeque<>();
    private final ArrayDeque<double[]> pollenHistory = new ArrayDeque<>();
    private List<LynxModule> hubs;
    private DcMotorEx nectarFlywheel;
    private DcMotorEx pollenFlywheel;
    private ServoImplEx nectarHood;
    private ServoImplEx pollenHood;
    private VoltageSensor battery;
    private long lastLoopNs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarFlywheel");
        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenFlywheel");
        nectarHood = hardwareMap.get(ServoImplEx.class, "nectarHood");
        pollenHood = hardwareMap.get(ServoImplEx.class, "pollenHood");
        battery = hardwareMap.voltageSensor.iterator().next();
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        lastLoopNs = System.nanoTime();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        long loopStartNs = System.nanoTime();

        double nectarRpm = rpm(nectarHistory, nectarFlywheel, nectar.reverseEncoder);
        double pollenRpm = rpm(pollenHistory, pollenFlywheel, pollen.reverseEncoder);
        double nectarOut = power(nectar, nectarRpm);
        double pollenOut = power(pollen, pollenRpm);

        nectarFlywheel.setDirection(direction(nectar.reverseFlywheel));
        pollenFlywheel.setDirection(direction(pollen.reverseFlywheel));
        nectarFlywheel.setPower(nectarOut);
        pollenFlywheel.setPower(pollenOut);
        nectarHood.setPosition(nectar.hoodPosition);
        pollenHood.setPosition(pollen.hoodPosition);

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

        telemetry.addData("Battery (V)", battery.getVoltage());
        telemetry.addData("Loop (ms)", (loopStartNs - lastLoopNs) / 1e6);
        lastLoopNs = loopStartNs;
    }

    @Override
    public void stop() {
        nectarFlywheel.setPower(0);
        pollenFlywheel.setPower(0);
    }

    private static double power(ShooterSettings shooter, double rpm) {
        if (shooter.targetRpm <= 0) return shooter.power;
        return Range.clip(shooter.kV * shooter.targetRpm + shooter.kP * (shooter.targetRpm - rpm), 0, 1);
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
