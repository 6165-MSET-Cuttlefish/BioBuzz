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
        public static double OffPower = 0;
        public static double OnPower = 0.7;
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

    private final EnhancedMotor left;
    private final EnhancedMotor right;

    public BettaShooter(HardwareMap hardwareMap) {
        left = new EnhancedMotor(hardwareMap, "leftShooter").withCachingTolerance(0.001);
        right = new EnhancedMotor(hardwareMap, "rightShooter").withCachingTolerance(0.001);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    protected void initStates() {
        setStates(ShooterState.OFF, ShooterState.OFF);
        bindTunable(ShooterState.OFF, () -> Tuning.OffPower);
        bindTunable(ShooterState.ON, () -> Tuning.OnPower);
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
    }

    @Override
    public void stop() {
    }

    @Override
    protected void onTelemetry() {
        log("Shooter State", getState(ShooterState.class));
    }
}
