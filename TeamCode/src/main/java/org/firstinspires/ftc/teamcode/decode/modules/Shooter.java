package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.decode.DecodeContext.distanceToGoal;
import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.shooterTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.control.PidController;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

@Config("Decode Shooter")
public class Shooter extends Module {
    private static final double TICKS_PER_REV = 8192.0;

    private final EnhancedMotor left;
    private final EnhancedMotor right;
    private final EnhancedServo hood;

    private double targetVelocityRPM = 50;
    private double hoodPosition;

    public double closeVelocityOffset = 0;
    public double closeHoodOffset = 0;
    public double farVelocityOffset = 0;
    public double farHoodOffset = 0;

    public static class ShooterPID {
        public double Kp = 0.0018;
        public double Ki = 0;
        public double Kd = 0.000195;
        public double FScale = 0.89;
        public double Kl = 0;
        public double LP1Rate = 0.7;
        public double hoodLP1Rate = 0.5;
        public double LP2Rate = 0.3;
    }

    public static boolean bangBangEnabled = true;
    public static boolean zeroPower = false;
    public static double bangBangTolerancePercent = 0.10;
    public static double reverseBangBangPercent = 0.10;
    public static double reverseBangBangPower = -0.01;
    public static boolean pureBangBang = false;
    public static ShooterPID shooterPid = new ShooterPID();

    private final PidController shooterPidController = new PidController();
    private final ElapsedTime shooterVelocityTimer = new ElapsedTime();

    private double shooterCurrentVelocityRPM = 0;
    private double shooterCurrentVelocityRPMRaw = 0;
    private double shooterCurrentVelocityRPMLP1, shooterCurrentVelocityRPMLP2 = 0;
    private double shooterCurrentVelocityRPMHoodLP1 = 0;
    private int shooterPreviousPosition = 0;
    private boolean shooterFirstLoop = true;
    private double shooterPidOutput = 0;

    public enum FlywheelState implements State {
        IDLE(1800),
        OFF(0),
        PID(0);

        FlywheelState(double value) {
            setValue(value);
        }
    }

    public enum HoodState implements State {
        RESET(0.585),
        BOTTOM(0),
        TOP(0.32),
        PID(0);

        HoodState(double value) {
            setValue(value);
        }
    }

    public Shooter(HardwareMap hardwareMap) {
        left = new EnhancedMotor(hardwareMap, "leftFlywheel").withCachingTolerance(0.005);
        right = new EnhancedMotor(hardwareMap, "rightFlywheel").withCachingTolerance(0.005);
        hood = new EnhancedServo(hardwareMap, "hood").withCachingTolerance(0.001);

        left.setVoltageCompensationEnabled(true);
        right.setVoltageCompensationEnabled(true);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterPidController.resetIntegralOnTargetChange = false;
        shooterPidController.derivativeOnMeasurement = true;
        shooterVelocityTimer.reset();
    }

    @Override
    protected void initStates() {
        setStates(FlywheelState.OFF, HoodState.BOTTOM);
    }

    @Override
    protected void read() {
        updateShooterVelocity();

        double velocityOffset;
        double hoodOffset;
        if (ShooterInterpolation.activeMode == ShooterInterpolation.Mode.CLOSE) {
            velocityOffset = closeVelocityOffset;
            hoodOffset = closeHoodOffset;
        } else {
            velocityOffset = farVelocityOffset;
            hoodOffset = farHoodOffset;
        }

        FlywheelState fw = getState(FlywheelState.class);
        HoodState hs = getState(HoodState.class);

        if (fw == FlywheelState.PID) {
            targetVelocityRPM = ShooterInterpolation.getTargetRPM(distanceToGoal);
        } else {
            targetVelocityRPM = fw.getValue();
        }

        if (hs == HoodState.PID) {
            hoodPosition = ShooterInterpolation.getHoodPosition(
                    distanceToGoal,
                    shooterCurrentVelocityRPMHoodLP1 - velocityOffset)
                    + HoodState.RESET.getValue();
        } else if (hs == HoodState.RESET) {
            hoodPosition = HoodState.RESET.getValue();
        } else {
            hoodPosition = hs.getValue() + HoodState.RESET.getValue();
        }

        if (fw != FlywheelState.IDLE && fw != FlywheelState.OFF) {
            targetVelocityRPM += velocityOffset;
        }
        hoodPosition += hoodOffset;

        calculateShooterPower();
    }

    @Override
    protected void write() {
        left.setPower(shooterPidOutput);
        right.setPower(shooterPidOutput);

        if (hoodPosition >= HoodState.RESET.getValue()
                && hoodPosition <= HoodState.TOP.getValue() + HoodState.RESET.getValue()) {
            hood.setPosition(hoodPosition);
        }
    }

    @Override
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    @Override
    protected void onTelemetry() {
        if (shooterTelemetry.flywheel) {
            logDashboard("Flywheel State", getState(FlywheelState.class));
            log("Target Velocity (RPM)", "%.1f", targetVelocityRPM);
            log("Measured Velocity (RPM)", "%.1f", shooterCurrentVelocityRPM);
            log("Close Vel Offset", "%.1f", closeVelocityOffset);
            log("Far Vel Offset", "%.1f", farVelocityOffset);
            logDashboard("Measured Velocity LP1 (RPM)", "%.1f", shooterCurrentVelocityRPMLP1);
            logDashboard("Measured Velocity Raw (RPM)", "%.1f", shooterCurrentVelocityRPMRaw);

            logDashboard("PID Error (RPM)", "%.1f", shooterPidController.getError());
            logDashboard("PID Output", "%.3f", shooterPidOutput);
            logDashboard("Motor power", "%.5f", left.getPower());

            double bbRPM = targetVelocityRPM * bangBangTolerancePercent;
            logDashboard("Bang-Bang Error (RPM)", "%.1f", targetVelocityRPM - shooterCurrentVelocityRPM);
            logDashboard("Bang-Bang Tolerance (RPM)", "%.1f", bbRPM);

            logDashboard("Distance to Goal", "%.2f", distanceToGoal);
        }

        if (shooterTelemetry.lut) {
            logDashboard("Interp Target Distance", "%.1f", ShooterInterpolation.lastTargetDistance);
            logDashboard("Interp Base RPM", "%.1f", ShooterInterpolation.lastBaseRPM);
            logDashboard("Interp Compensated RPM", "%.1f", ShooterInterpolation.lastCompensatedRPM);
            logDashboard("Interp Selected Hood", "%.3f", ShooterInterpolation.lastSelectedHood);
            logDashboard("Interp Distance Key", "%s", ShooterInterpolation.lastClosestDistanceKey);
            logDashboard("Interp Points Count", "%d", ShooterInterpolation.lastPointsCount);
        }

        if (shooterTelemetry.hood) {
            logDashboard("Hood State", getState(HoodState.class));
            log("Hood Position", "%.3f", hoodPosition);
            log("Close Hood Offset", "%.3f", closeHoodOffset);
            log("Far Hood Offset", "%.3f", farHoodOffset);
        }

        if (shooterTelemetry.current) {
            logDashboard("Left Flywheel Current (A)", "%.2f", left.getCurrent(CurrentUnit.AMPS));
            logDashboard("Right Flywheel Current (A)", "%.2f", right.getCurrent(CurrentUnit.AMPS));
        }
    }

    private void updateShooterVelocity() {
        int currentPosition = left.getCurrentPosition();

        if (shooterFirstLoop) {
            shooterCurrentVelocityRPM = 0;
            shooterFirstLoop = false;
        } else {
            double deltaTime = shooterVelocityTimer.seconds();
            if (deltaTime > 0) {
                double deltaPosition = currentPosition - shooterPreviousPosition;
                double rawRPM = (deltaPosition / deltaTime) * (60.0 / TICKS_PER_REV);

                shooterCurrentVelocityRPMRaw = rawRPM;

                shooterCurrentVelocityRPMLP1 +=
                        (rawRPM - shooterCurrentVelocityRPMLP1) * shooterPid.LP1Rate;
                shooterCurrentVelocityRPMHoodLP1 +=
                        (rawRPM - shooterCurrentVelocityRPMHoodLP1) * shooterPid.hoodLP1Rate;
                shooterCurrentVelocityRPMLP2 +=
                        (shooterCurrentVelocityRPMLP1 - shooterCurrentVelocityRPMLP2)
                                * shooterPid.LP2Rate;

                shooterCurrentVelocityRPM = shooterCurrentVelocityRPMLP2;
            }
        }

        shooterPreviousPosition = currentPosition;
        shooterVelocityTimer.reset();
    }

    private void calculateShooterPower() {
        double error = targetVelocityRPM - shooterCurrentVelocityRPM;

        if (pureBangBang) {
            shooterPidOutput = (error > 0) ? 1 : 0;
        } else if (!bangBangEnabled) {
            shooterPidOutput = calculateShooterPIDPower();
        } else {
            double pidPower = calculateShooterPIDPower();
            double thresholdRPM = targetVelocityRPM * bangBangTolerancePercent;
            double reverseBangBangThresholdRPM = targetVelocityRPM * reverseBangBangPercent;

            if (error > thresholdRPM) {
                shooterPidOutput = 1;
            } else if (error < -reverseBangBangThresholdRPM) {
                shooterPidOutput = reverseBangBangPower;
            } else if (error < (zeroPower ? 0 : -thresholdRPM)) {
                shooterPidOutput = 0;
            } else {
                shooterPidOutput = pidPower;
            }
        }

        if (getState(FlywheelState.class) == FlywheelState.OFF) {
            shooterPidOutput = 0;
        }
    }

    private double calculateShooterPIDPower() {
        // Fit from https://www.desmos.com/calculator/ykvsfthqvf
        double kf = targetVelocityRPM != 0
                ? shooterPid.FScale * (0.0253212 * Math.sqrt(targetVelocityRPM + 3626.49145) - 1.47831) / targetVelocityRPM
                : 0;
        shooterPidController.setGains(shooterPid.Kp, shooterPid.Ki, shooterPid.Kd, shooterPid.Kl);
        shooterPidController.kPosition = kf;
        shooterPidController.update(targetVelocityRPM, shooterCurrentVelocityRPM);
        return shooterPidController.calculate();
    }

    public double getLeftShooterCurrent() {
        return left.getCurrent(CurrentUnit.AMPS);
    }

    public double getRightShooterCurrent() {
        return right.getCurrent(CurrentUnit.AMPS);
    }

}
