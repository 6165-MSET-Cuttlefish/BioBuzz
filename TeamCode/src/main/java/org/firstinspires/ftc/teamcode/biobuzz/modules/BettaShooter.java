package org.firstinspires.ftc.teamcode.biobuzz.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

public class BettaShooter extends Module {

    @Config("Betta Shooter")
    public static class Tuning {
        public static double OffPower = 0;
        public static double OnPower = 0.7;
        public static double hoodDownPosition = 0.3;
        public static double hoodUpPosition = 0.7;
        public static double hardstopBlockedPosition = 0.6;
        public static double hardstopUnblockedPosition = 0.3;
    }

    public enum ShooterState implements State {
        ON(0.7),
        OFF(0);

        ShooterState(double position) {
            setValue(position);
        }

        @Override
        public double getValue() {
            return State.super.getValue();
        }
    }

    public enum HoodState implements State {
        IDLE(-1),
        DOWN(0.3),
        UP(0.7);

        HoodState(double position) {
            setValue(position);
        }
    }

    public enum HardstopState implements State {
        IDLE(-1),
        BLOCKED(0.6),
        UNBLOCKED(0.3);

        HardstopState(double position) {
            setValue(position);
        }
    }

    private final EnhancedMotor left;
    private final EnhancedMotor right;
    private final EnhancedServo hood;
    private final EnhancedServo hardstop;

    public BettaShooter(HardwareMap hardwareMap) {
        left = new EnhancedMotor(hardwareMap, "leftShooter").withCachingTolerance(0.001);
        right = new EnhancedMotor(hardwareMap, "rightShooter").withCachingTolerance(0.001);
        hood = new EnhancedServo(hardwareMap, "hood").withCachingTolerance(0.001);
        hardstop = new EnhancedServo(hardwareMap, "hardstop").withCachingTolerance(0.001);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    protected void initStates() {
        setStates(ShooterState.OFF, HoodState.IDLE, HardstopState.IDLE);
        bindTunable(ShooterState.OFF, () -> Tuning.OffPower);
        bindTunable(ShooterState.ON, () -> Tuning.OnPower);
        bindTunable(HoodState.DOWN, () -> Tuning.hoodDownPosition);
        bindTunable(HoodState.UP, () -> Tuning.hoodUpPosition);
        bindTunable(HardstopState.BLOCKED, () -> Tuning.hardstopBlockedPosition);
        bindTunable(HardstopState.UNBLOCKED, () -> Tuning.hardstopUnblockedPosition);
    }

    @Override
    protected void read() {}

    // IDLE never drives a servo, so it stays unpowered until a position is asked for.
    @Override
    protected void write() {
        ShooterState both = getState(ShooterState.class);
        if (both == ShooterState.OFF) {
            left.setPower(both.getValue());
            right.setPower(both.getValue());
        }
        if (both == ShooterState.ON) {
            left.setPower(both.getValue());
            right.setPower(both.getValue());
        }
        HoodState hoodState = getState(HoodState.class);
        HardstopState hardstopState = getState(HardstopState.class);
        if (hoodState != HoodState.IDLE) hood.setPosition(hoodState.getValue());
        if (hardstopState != HardstopState.IDLE) hardstop.setPosition(hardstopState.getValue());
    }

    @Override
    public void stop() {
        hood.setPwmDisable();
        hardstop.setPwmDisable();
    }

    @Override
    protected void onTelemetry() {
        log("Shooter State", getState(ShooterState.class));
        log("Hood State", getState(HoodState.class));
        log("Hardstop State", getState(HardstopState.class));
    }
}
