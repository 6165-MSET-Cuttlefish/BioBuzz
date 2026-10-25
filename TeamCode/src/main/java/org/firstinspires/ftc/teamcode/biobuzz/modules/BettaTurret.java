package org.firstinspires.ftc.teamcode.biobuzz.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

public class BettaTurret extends Module {

    @Config("Betta Turret")
    public static class Tuning {
        public static double centerPosition = 0.5;
        public static double rightForwardPosition = 0.1;
        public static double rightSidewaysPosition = 0.3;
        // Read at construction: change, then re-INIT.
        public static boolean reverseLeft = false;
        public static boolean reverseRight = false;
    }

    public enum TurretState implements State {
        IDLE(-1),
        CENTER(0.5);

        TurretState(double position) {
            setValue(position);
        }
    }

    /** Drives TurretR alone, leaving TurretL to TurretState. */
    public enum RightState implements State {
        IDLE(-1),
        FORWARD(0.1),
        SIDEWAYS(0.3);

        RightState(double position) {
            setValue(position);
        }
    }

    private final EnhancedServo left;
    private final EnhancedServo right;

    public BettaTurret(HardwareMap hardwareMap) {
        left = new EnhancedServo(hardwareMap, "TurretL").withCachingTolerance(0.001);
        right = new EnhancedServo(hardwareMap, "TurretR").withCachingTolerance(0.001);

        left.setDirection(Tuning.reverseLeft ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        right.setDirection(Tuning.reverseRight ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    @Override
    protected void initStates() {
        setStates(TurretState.IDLE, RightState.IDLE);
        bindTunable(TurretState.CENTER, () -> Tuning.centerPosition);
        bindTunable(RightState.FORWARD, () -> Tuning.rightForwardPosition);
        bindTunable(RightState.SIDEWAYS, () -> Tuning.rightSidewaysPosition);
    }

    @Override
    protected void read() {}

    // IDLE never drives a servo, so it stays unpowered until a position is asked for.
    @Override
    protected void write() {
        TurretState both = getState(TurretState.class);
        RightState rightOnly = getState(RightState.class);
        if (both != TurretState.IDLE) left.setPosition(both.getValue());
        if (rightOnly != RightState.IDLE) right.setPosition(rightOnly.getValue());
        else if (both != TurretState.IDLE) right.setPosition(both.getValue());
    }

    @Override
    public void stop() {
        left.setPwmDisable();
        right.setPwmDisable();
    }

    @Override
    protected void onTelemetry() {
        log("Turret State", getState(TurretState.class));
        log("Turret Right State", getState(RightState.class));
    }
}
