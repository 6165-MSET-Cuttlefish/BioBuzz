package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

public class BettaShooter extends Module {

    @Config("Betta Shooter")
    public static class Tuning {
        public static double onPower = 0.5;
        public static double hoodPosition = 0.5;
        // Read at construction: change, then re-INIT. The right motor always runs opposite, as they share a shaft.
        public static boolean reverseLeft = true;
        public static boolean reverseHood = false;
    }

    public enum FlywheelState implements State {
        OFF(0),
        ON(0.5);

        FlywheelState(double power) {
            setValue(power);
        }
    }

    public enum HoodState implements State {
        IDLE(-1),
        POSITION(0.5);

        HoodState(double position) {
            setValue(position);
        }
    }

    private final EnhancedMotor left;
    private final EnhancedMotor right;
    private final EnhancedServo hood;

    public BettaShooter(HardwareMap hardwareMap) {
        left = new EnhancedMotor(hardwareMap, "leftShooter").withCachingTolerance(0.005).withVoltageCompensation(13.0);
        right = new EnhancedMotor(hardwareMap, "rightShooter").withCachingTolerance(0.005).withVoltageCompensation(13.0);
        hood = new EnhancedServo(hardwareMap, "hood").withCachingTolerance(0.001);

        left.setDirection(Tuning.reverseLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(Tuning.reverseLeft ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hood.setDirection(Tuning.reverseHood ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    @Override
    protected void initStates() {
        setStates(FlywheelState.OFF, HoodState.IDLE);
        bindTunable(FlywheelState.ON, () -> Tuning.onPower);
        bindTunable(HoodState.POSITION, () -> Tuning.hoodPosition);
    }

    @Override
    protected void read() {}

    @Override
    protected void write() {
        double power = getState(FlywheelState.class).getValue();
        left.setPower(power);
        right.setPower(power);

        // IDLE never drives the hood, so it stays unpowered until a position is asked for.
        HoodState hoodState = getState(HoodState.class);
        if (hoodState != HoodState.IDLE) hood.setPosition(hoodState.getValue());
    }

    @Override
    public void stop() {
        left.stop();
        right.stop();
    }

    @Override
    protected void onTelemetry() {
        log("Flywheel State", getState(FlywheelState.class));
        log("Flywheel Power", "%.2f", getState(FlywheelState.class).getValue());
        log("Hood State", getState(HoodState.class));
    }
}
