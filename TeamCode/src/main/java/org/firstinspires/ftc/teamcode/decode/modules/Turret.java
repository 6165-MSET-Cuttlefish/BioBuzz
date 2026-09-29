package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetY;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldY;
import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.turretTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

@Config("Decode Turret")
public class Turret extends Module {

    public double turretManualOffset = 0;

    public static boolean isCloseTele = true;
    public static double teleTurretOffsetClose = 2;
    public static double teleTurretOffsetFar = -4;

    public static double turretX = -3.875;
    public static double turretY = -1.6;

    public static double TENSION_OFFSET = 0.0;

    public static double AHEAD_GAIN = 0;
    public static double MIN_DELTA_FOR_AHEAD = 0;

    public static double frontServoOffset = 0.008;
    public static double backServoOffset = 0.00;

    private final EnhancedServo turretServoFront;
    private final EnhancedServo turretServoBack;

    private Follower follower;

    private double targetAngle = 0;
    private double rawTargetAngle = 0;
    private double previousTargetAngle = 0.0;

    private double aheadTargetAngle = 0.0;

    private double targetServoPosition = 0.5;
    private double lastTargetServoPosition = 0.5;

    private double deltaRawTarget = 0;
    private double previousRawTargetAngle = 0.0;

    public double flightTime = 1;

    public enum TurretState implements State {
        CENTER(.5),
        RIGHT(0.995),
        LEFT(0.005),
        AUTOAIM(-1),
        HOLD(-1);

        TurretState(double value) {
            setValue(value);
        }
    }

    public Turret(HardwareMap hardwareMap) {
        turretServoFront = new EnhancedServo(hardwareMap, "turretFront").withCachingTolerance(0.001);
        turretServoBack = new EnhancedServo(hardwareMap, "turretBack").withCachingTolerance(0.001);

        turretServoFront.setPwmRange(new PwmControl.PwmRange(525, 2475));
        turretServoBack.setPwmRange(new PwmControl.PwmRange(525, 2475));

        turretServoFront.setDirection(Servo.Direction.FORWARD);
        turretServoBack.setDirection(Servo.Direction.FORWARD);
    }

    public Turret withFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    private Follower requireFollower() {
        if (follower == null) {
            throw new IllegalStateException("Turret needs a Follower; pass one with withFollower()");
        }
        return follower;
    }

    @Override
    protected void initStates() {
        setStates(TurretState.AUTOAIM);
    }

    @Override
    protected void read() {
        turretManualOffset = isCloseTele ? teleTurretOffsetClose : teleTurretOffsetFar;

        updateTargetPosition();
    }

    @Override
    protected void write() {
        double frontPos = Math.max(0.0, Math.min(1.0, targetServoPosition - TENSION_OFFSET + frontServoOffset));
        double backPos = Math.max(0.0, Math.min(1.0, targetServoPosition + TENSION_OFFSET + backServoOffset));

        turretServoFront.setPosition(frontPos);
        turretServoBack.setPosition(backPos);
    }

    @Override
    public void stop() {
        turretServoFront.setPwmDisable();
        turretServoBack.setPwmDisable();
    }

    @Override
    protected void onTelemetry() {
        if (turretTelemetry.position) {
            logDashboard("Turret State", getState(TurretState.class));
            logDashboard("Raw Target Angle (deg)", "%.1f", rawTargetAngle);
            logDashboard("Delta Raw Target (deg)", "%.1f", deltaRawTarget);
            log("Target Angle (deg)", "%.1f", targetAngle);
            logDashboard("Ahead Target Angle (deg)", "%.1f", aheadTargetAngle);
            log("Turret Offset (deg)", "%.1f", turretManualOffset);
        }
        logDashboard("Target Servo Position", "%.3f", targetServoPosition);
        if (turretTelemetry.servos) {
            logDashboard("Front Servo Position", "%.3f", turretServoFront.getPosition());
            logDashboard("Back Servo Position", "%.3f", turretServoBack.getPosition());
        }
    }

    private void updateTargetPosition() {
        if (getState(TurretState.class).equals(TurretState.AUTOAIM)) {
            double robotHeading = Math.toDegrees(requireFollower().pose().heading());

            double absoluteAngle =
                    Math.toDegrees(Math.atan2(targetY - turretFieldY, targetX - turretFieldX));

            rawTargetAngle = normalizeAngle(absoluteAngle - robotHeading);

            deltaRawTarget = calculateShortestError(rawTargetAngle, previousRawTargetAngle);
            previousRawTargetAngle = rawTargetAngle;

            targetAngle = rawTargetAngle + turretManualOffset;

            double angleDelta = calculateShortestError(targetAngle, previousTargetAngle);
            aheadTargetAngle = (Math.abs(angleDelta) > MIN_DELTA_FOR_AHEAD)
                    ? targetAngle + Math.signum(angleDelta) * AHEAD_GAIN
                    : targetAngle;

            previousTargetAngle = targetAngle;
            targetServoPosition = angleToServoPosition(aheadTargetAngle);

        } else if (getState(TurretState.class).equals(TurretState.HOLD)) {
            targetServoPosition = lastTargetServoPosition;
        } else {
            targetServoPosition = getState(TurretState.class).getValue();
        }

        double minServo = Math.min(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        double maxServo = Math.max(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        if (targetServoPosition > maxServo) {
            targetServoPosition = maxServo;
        } else if (targetServoPosition < minServo) {
            targetServoPosition = minServo;
        }

        lastTargetServoPosition = targetServoPosition;
    }

    private double angleToServoPosition(double angleDeg) {
        double angle = normalizeAngle(angleDeg);
        double scale = (TurretState.LEFT.getValue() - TurretState.RIGHT.getValue()) / 180.0;
        return (angle > 180)
                ? TurretState.CENTER.getValue() + scale * (angle - 360)
                : TurretState.CENTER.getValue() + scale * angle;
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double calculateShortestError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}
