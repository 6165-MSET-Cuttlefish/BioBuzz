package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.control.PidController;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.BatteryVoltage;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

import java.util.ArrayDeque;

@Config("BioBuzz Shooter")
public class Shooter extends Module {
    private static final double REFERENCE_VOLTS = 13.0;

    public static class FlywheelTuning {
        public boolean reverseMotor = false;
        public boolean reverseEncoder = false;
        public double ticksPerRev = 8192;
        public double rpmWindowMs = 50;
        public double targetRPM = 0;
        public double openLoopPower = 0;
        public double kP = 0;
        public double kI = 0;
        public double kD = 0;
        public double hoodPosition = 0.5;
        public FeedforwardPoint[] feedforward = emptyTable(8);
    }

    public static class FeedforwardPoint {
        public double rpm = 0;
        public double power = 0;
    }

    public static FlywheelTuning pollenTuning = new FlywheelTuning();
    public static FlywheelTuning nectarTuning = new FlywheelTuning();

    private enum Drive { OFF, RPM, POWER }

    public enum PollenState implements State {
        OFF(Drive.OFF),
        ON(Drive.RPM),
        OPEN_LOOP(Drive.POWER);

        private final Drive drive;

        PollenState(Drive drive) {
            this.drive = drive;
            setValue(0);
        }
    }

    public enum NectarState implements State {
        OFF(Drive.OFF),
        ON(Drive.RPM),
        OPEN_LOOP(Drive.POWER);

        private final Drive drive;

        NectarState(Drive drive) {
            this.drive = drive;
            setValue(0);
        }
    }

    public enum PollenHoodState implements State {
        POSITION(0.5);

        PollenHoodState(double value) {
            setValue(value);
        }
    }

    public enum NectarHoodState implements State {
        POSITION(0.5);

        NectarHoodState(double value) {
            setValue(value);
        }
    }

    public enum ShooterHardstopState implements State {
        BLOCKED(0.6),
        UNBLOCKED(0.3);

        ShooterHardstopState(double value) {
            setValue(value);
        }
    }

    private final Flywheel pollen;
    private final Flywheel nectar;
    private final EnhancedServo pollenHood;
    private final EnhancedServo nectarHood;
    private final EnhancedServo hardstop;

    public Shooter(HardwareMap hardwareMap) {
        pollen = new Flywheel(hardwareMap, "PollenShooter", "Pollen", pollenTuning);
        nectar = new Flywheel(hardwareMap, "NectarShooter", "Nectar", nectarTuning);
        pollenHood = new EnhancedServo(hardwareMap, "PollenHood").withCachingTolerance(0.001);
        nectarHood = new EnhancedServo(hardwareMap, "NectarHood").withCachingTolerance(0.001);
        hardstop = new EnhancedServo(hardwareMap, "hardstop").withCachingTolerance(0.001);
    }

    @Override
    protected void initStates() {
        setStates(PollenState.OFF, NectarState.OFF, PollenHoodState.POSITION, NectarHoodState.POSITION, ShooterHardstopState.BLOCKED);
        bindTunable(PollenState.ON, () -> pollenTuning.targetRPM);
        bindTunable(PollenState.OPEN_LOOP, () -> pollenTuning.openLoopPower);
        bindTunable(NectarState.ON, () -> nectarTuning.targetRPM);
        bindTunable(NectarState.OPEN_LOOP, () -> nectarTuning.openLoopPower);
        bindTunable(PollenHoodState.POSITION, () -> pollenTuning.hoodPosition);
        bindTunable(NectarHoodState.POSITION, () -> nectarTuning.hoodPosition);
    }

    @Override
    protected void read() {
        PollenState pollenState = getState(PollenState.class);
        NectarState nectarState = getState(NectarState.class);
        pollen.update(pollenState.drive, pollenState.getValue());
        nectar.update(nectarState.drive, nectarState.getValue());
    }

    @Override
    protected void write() {
        pollen.write();
        nectar.write();
        pollenHood.setPosition(getState(PollenHoodState.class).getValue());
        nectarHood.setPosition(getState(NectarHoodState.class).getValue());
        hardstop.setPosition(getState(ShooterHardstopState.class).getValue());
    }

    @Override
    public void stop() {
        pollen.stop();
        nectar.stop();
    }

    @Override
    protected void onTelemetry() {
        logDashboard("Pollen State", getState(PollenState.class));
        pollen.telemetry();
        log("Pollen Hood Position", "%.3f", getState(PollenHoodState.class).getValue());

        logDashboard("Nectar State", getState(NectarState.class));
        nectar.telemetry();
        log("Nectar Hood Position", "%.3f", getState(NectarHoodState.class).getValue());
    }

    public double getPollenRPM() {
        return pollen.rpm;
    }

    public double getNectarRPM() {
        return nectar.rpm;
    }

    public static double feedforwardPower(String name, FeedforwardPoint[] table, double rpm) {
        for (FeedforwardPoint p : table) {
            for (FeedforwardPoint q : table) {
                if (p.rpm > 0 && q != p && q.rpm == p.rpm) {
                    throw new IllegalArgumentException(String.format("%s feedforward has two points at %.0f RPM", name, p.rpm));
                }
            }
        }
        if (rpm <= 0) return 0;
        FeedforwardPoint below = null;
        FeedforwardPoint above = null;
        for (FeedforwardPoint p : table) {
            if (p.rpm <= 0) continue;
            if (p.rpm <= rpm && (below == null || p.rpm > below.rpm)) below = p;
            if (p.rpm >= rpm && (above == null || p.rpm < above.rpm)) above = p;
        }
        if (below == null) return above == null ? 0 : above.power;
        if (above == null || above == below) return below.power;
        return below.power + (rpm - below.rpm) / (above.rpm - below.rpm) * (above.power - below.power);
    }

    private static FeedforwardPoint[] emptyTable(int size) {
        FeedforwardPoint[] table = new FeedforwardPoint[size];
        for (int i = 0; i < size; i++) table[i] = new FeedforwardPoint();
        return table;
    }

    private static DcMotorSimple.Direction direction(boolean reverse) {
        return reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static double clip(double power) {
        return Math.max(0, Math.min(1, power));
    }

    private final class Flywheel {
        private final EnhancedMotor motor;
        private final String name;
        private final FlywheelTuning tuning;
        private final PidController pid = new PidController();
        private final ArrayDeque<double[]> history = new ArrayDeque<>();
        private boolean motorReversed;
        private Drive drive = Drive.OFF;
        private double target;
        private double rpm;
        private double feedforward;
        private double headroom = 1;
        private double power;

        Flywheel(HardwareMap hardwareMap, String deviceName, String name, FlywheelTuning tuning) {
            motor = new EnhancedMotor(hardwareMap, deviceName).withCachingTolerance(0.005).withVoltageCompensation(REFERENCE_VOLTS);
            this.name = name;
            this.tuning = tuning;
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorReversed = tuning.reverseMotor;
            motor.setDirection(direction(motorReversed));
            pid.resetIntegralOnTargetChange = false;
            pid.derivativeOnMeasurement = true;
        }

        void update(Drive drive, double value) {
            this.drive = drive;
            if (tuning.reverseMotor != motorReversed) {
                motorReversed = tuning.reverseMotor;
                motor.setDirection(direction(motorReversed));
                history.clear();
            }
            measure();

            target = drive == Drive.RPM ? value : 0;
            feedforward = feedforwardPower(name, tuning.feedforward, target);
            headroom = Math.min(1, BatteryVoltage.get() / REFERENCE_VOLTS);
            if (drive == Drive.RPM) {
                pid.setGains(tuning.kP, tuning.kI, tuning.kD, 0);
                pid.outputMin = -feedforward;
                pid.outputMax = headroom - feedforward;
                pid.update(target, rpm);
                power = clip(feedforward + pid.calculate());
            } else {
                // A spin-down or open-loop run must not leave integral behind for the next spin-up.
                pid.reset();
                power = drive == Drive.POWER ? clip(value) : 0;
            }
        }

        private void measure() {
            long before = System.nanoTime();
            int ticks = motor.getCurrentPosition();
            long after = System.nanoTime();
            double seconds = (before + (after - before) / 2) / 1e9;
            history.addLast(new double[] {seconds, ticks});
            while (history.size() > 2 && seconds - history.peekFirst()[0] > tuning.rpmWindowMs / 1000) history.removeFirst();
            double[] oldest = history.peekFirst();
            double sign = tuning.reverseEncoder ? -1 : 1;
            rpm = history.size() < 2 ? 0 : sign * (ticks - (int) oldest[1]) / (seconds - oldest[0]) / tuning.ticksPerRev * 60;
        }

        void write() {
            motor.setPower(power);
        }

        void stop() {
            motor.stop();
        }

        void telemetry() {
            log(name + " Target Velocity (RPM)", "%.1f", target);
            log(name + " Measured Velocity (RPM)", "%.1f", rpm);
            log(name + " Power", "%.3f (feedforward %.3f)", power, feedforward);
            if (drive == Drive.POWER) {
                log(name + " Feedforward Point", power > headroom ? "rpm %.0f, power %.3f (saturated)" : "rpm %.0f, power %.3f", rpm, Math.min(power, headroom));
            }
            if (rpm < -50 && power > 0.1) log(name + " Encoder", "RPM reads negative while driven forward: flip reverseEncoder");
        }
    }
}
