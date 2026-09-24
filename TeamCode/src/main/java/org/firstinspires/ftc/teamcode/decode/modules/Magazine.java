package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.magazineTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

@Config("Decode Magazine")
public class Magazine extends Module {
    private final EnhancedServo horizontalFront;
    private final EnhancedServo horizontalBack;
    private final EnhancedMotor intake;
    private final EnhancedMotor vertical;

    public static boolean optimizeServoCachingTolerances = true;

    private double horizontalFrontPosition, horizontalBackPosition;
    private double intakePower, verticalPower;

    // Servo position units per millisecond.
    public static double horizontalFrontSpeed = 0.0008;
    public static double horizontalBackSpeed = 0.0005;
    private double currentHorizontalFrontPosition = Double.NaN;
    private double currentHorizontalBackPosition = Double.NaN;
    private long lastHorizontalFrontUpdateTime = 0;
    private long lastHorizontalBackUpdateTime = 0;

    public enum HorizontalFrontState implements State {
        OPEN(0.6),
        OPEN_SHOOT(OPEN.getValue() - 0.125);

        HorizontalFrontState(double value) {
            setValue(value);
        }
    }

    public enum HorizontalBackState implements State {
        OPEN(0.39),
        OPEN_SHOOT(OPEN.getValue() - 0.02);

        HorizontalBackState(double value) {
            setValue(value);
        }
    }

    public enum IntakeState implements State {
        FORWARD(1),
        SHOOTING(1),
        IDLE(0.3),
        REVERSE(-1),
        OFF(0);

        IntakeState(double value) {
            setValue(value);
        }
    }

    public enum VerticalState implements State {
        ON(1),
        HALF_DOWN(-0.5),
        OFF(0);

        VerticalState(double value) {
            setValue(value);
        }
    }

    public Magazine(HardwareMap hardwareMap) {
        setTelemetryEnabled(magazineTelemetry.TOGGLE);

        double servoTol = optimizeServoCachingTolerances ? 0.005 : 0.0;
        horizontalFront = new EnhancedServo(hardwareMap, "horizontalFront").withCachingTolerance(servoTol);
        horizontalBack = new EnhancedServo(hardwareMap, "horizontalBack").withCachingTolerance(servoTol);
        intake = new EnhancedMotor(hardwareMap, "intake");
        vertical = new EnhancedMotor(hardwareMap, "vertical");

        vertical.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    protected void initStates() {
        setStates(HorizontalFrontState.OPEN, HorizontalBackState.OPEN,
                IntakeState.OFF, VerticalState.OFF);
    }

    @Override
    protected void read() {
        double targetHorizontalFrontPos = getState(HorizontalFrontState.class).getValue();
        double targetHorizontalBackPos = getState(HorizontalBackState.class).getValue();

        if (Double.isNaN(currentHorizontalFrontPosition)) {
            currentHorizontalFrontPosition = targetHorizontalFrontPos;
        }
        if (Double.isNaN(currentHorizontalBackPosition)) {
            currentHorizontalBackPosition = targetHorizontalBackPos;
        }

        long now = System.currentTimeMillis();
        if (lastHorizontalFrontUpdateTime == 0) lastHorizontalFrontUpdateTime = now;
        currentHorizontalFrontPosition = slew(currentHorizontalFrontPosition, targetHorizontalFrontPos,
                horizontalFrontSpeed, now - lastHorizontalFrontUpdateTime);
        lastHorizontalFrontUpdateTime = now;

        if (lastHorizontalBackUpdateTime == 0) lastHorizontalBackUpdateTime = now;
        currentHorizontalBackPosition = slew(currentHorizontalBackPosition, targetHorizontalBackPos,
                horizontalBackSpeed, now - lastHorizontalBackUpdateTime);
        lastHorizontalBackUpdateTime = now;

        horizontalFrontPosition = currentHorizontalFrontPosition;
        horizontalBackPosition = currentHorizontalBackPosition;

        intakePower = getState(IntakeState.class).getValue();
        verticalPower = getState(VerticalState.class).getValue();
    }

    @Override
    protected void write() {
        horizontalFront.setPosition(horizontalFrontPosition);
        horizontalBack.setPosition(horizontalBackPosition);
        intake.setPower(intakePower);
        vertical.setPower(verticalPower);
    }

    @Override
    public void stop() {
        intake.setPower(0);
        vertical.setPower(0);
    }

    private static double slew(double current, double target, double unitsPerMs, long dtMs) {
        double maxStep = unitsPerMs * dtMs;
        if (current < target) return Math.min(current + maxStep, target);
        if (current > target) return Math.max(current - maxStep, target);
        return current;
    }

    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public double getVerticalCurrent() {
        return vertical.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    protected void onTelemetry() {
        if (magazineTelemetry.TOGGLE) {
            if (magazineTelemetry.intake) {
                logDashboard("Intake State", getState(IntakeState.class));
                logDashboard("Intake Power", "%.3f", intakePower);
            }
            if (magazineTelemetry.vertical) {
                logDashboard("Vertical State", getState(VerticalState.class));
                logDashboard("Vertical Power", "%.3f", verticalPower);
            }
            if (magazineTelemetry.servos) {
                logDashboard("Horizontal Front State", getState(HorizontalFrontState.class));
                logDashboard("Horizontal Front Position", "%.3f", horizontalFrontPosition);
                logDashboard("Horizontal Back State", getState(HorizontalBackState.class));
                logDashboard("Horizontal Back Position", "%.3f", horizontalBackPosition);
            }
            if (magazineTelemetry.current) {
                logDashboard("Intake Current (A)", "%.2f", intake.getCurrent(CurrentUnit.AMPS));
                logDashboard("Vertical Current (A)", "%.2f", vertical.getCurrent(CurrentUnit.AMPS));
            }
        }
    }
}
