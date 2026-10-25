package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.architecture.control.PidController;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

@Config("BioBuzz Shooter")
public class Shooter extends Module {
    private static final double TICKS_PER_REV = 8192.0;
    private final EnhancedMotor pollen;
    private final EnhancedMotor nectar;
    private final EnhancedServo pollenHood;
    private final EnhancedServo nectarHood;
    private final EnhancedServo hardstop;
    public static class PollenShooterTuning {
        public double P_Kp = 0;
        public double P_Ki = 0;
        public double P_Kd = 0;
        public double P_Kf = 0;
        public double targetRPM = 0;
        public double hoodPosition = 0.5;
    }
    public static class NectarShooterTuning {
        public double N_Kp = 0;
        public double N_Ki = 0;
        public double N_Kd = 0;
        public double N_Kf = 0;
        public double targetRPM = 0;
        public double hoodPosition = 0.5;
    }
    public static PollenShooterTuning pollenTuning = new PollenShooterTuning();
    public static NectarShooterTuning nectarTuning = new NectarShooterTuning();
    private final PidController pollenPidController = new PidController();
    private final PidController nectarPidController = new PidController();
    private final ElapsedTime pollenVelocityTimer = new ElapsedTime();
    private final ElapsedTime nectarVelocityTimer = new ElapsedTime();
    private double pollenTargetVelocityRPM = 0;
    private double pollenCurrentVelocityRPM = 0;
    private int pollenPreviousPosition = 0;
    private boolean pollenFirstLoop = true;
    private double pollenPidOutput = 0;
    private double nectarTargetVelocityRPM = 0;
    private double nectarCurrentVelocityRPM = 0;
    private int nectarPreviousPosition = 0;
    private boolean nectarFirstLoop = true;
    private double nectarPidOutput = 0;
    public enum PollenState implements State {
        OFF(0),
        ON(0);

        PollenState(double value) {
            setValue(value);
        }
    }
    public enum NectarState implements State {
        OFF(0),
        ON(0);

        NectarState(double value) {
            setValue(value);
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

    public enum ShooterHardstopState implements  State {
        BLOCKED(0.6),
        UNBLOCKED(0.3);

        ShooterHardstopState(double value) {
            setValue(value);
        }
    }
    public Shooter(HardwareMap hardwareMap) {
        pollen = new EnhancedMotor(hardwareMap, "PollenShooter").withCachingTolerance(0.005).withVoltageCompensation(13.0);
        nectar = new EnhancedMotor(hardwareMap, "NectarShooter").withCachingTolerance(0.005).withVoltageCompensation(13.0);
        pollenHood = new EnhancedServo(hardwareMap, "PollenHood").withCachingTolerance(0.001);
        nectarHood = new EnhancedServo(hardwareMap, "NectarHood").withCachingTolerance(0.001);
        hardstop = new EnhancedServo(hardwareMap, "hardstop").withCachingTolerance(0.001);

        pollen.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        nectar.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pollen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollen.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        nectar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pollenPidController.resetIntegralOnTargetChange = false;
        pollenPidController.derivativeOnMeasurement = true;
        nectarPidController.resetIntegralOnTargetChange = false;
        nectarPidController.derivativeOnMeasurement = true;
        pollenVelocityTimer.reset();
        nectarVelocityTimer.reset();
    }

    @Override
    protected void initStates() {
        setStates(PollenState.OFF, NectarState.OFF, PollenHoodState.POSITION, NectarHoodState.POSITION, ShooterHardstopState.BLOCKED);
        bindTunable(PollenState.ON, () -> pollenTuning.targetRPM);
        bindTunable(NectarState.ON, () -> nectarTuning.targetRPM);
        bindTunable(PollenHoodState.POSITION, () -> pollenTuning.hoodPosition);
        bindTunable(NectarHoodState.POSITION, () -> nectarTuning.hoodPosition);
    }

    @Override
    protected void read() {
        updatePollenVelocity();
        updateNectarVelocity();

        pollenTargetVelocityRPM = getState(PollenState.class).getValue();
        nectarTargetVelocityRPM = getState(NectarState.class).getValue();

        calculatePollenPower();
        calculateNectarPower();
    }

    @Override
    protected void write() {
        pollen.setPower(pollenPidOutput);
        nectar.setPower(nectarPidOutput);
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
        log("Pollen Target Velocity (RPM)", "%.1f", pollenTargetVelocityRPM);
        log("Pollen Measured Velocity (RPM)", "%.1f", pollenCurrentVelocityRPM);
        log("Pollen Hood Position", "%.3f", getState(PollenHoodState.class).getValue());


        logDashboard("Nectar State", getState(NectarState.class));
        log("Nectar Target Velocity (RPM)", "%.1f", nectarTargetVelocityRPM);
        log("Nectar Measured Velocity (RPM)", "%.1f", nectarCurrentVelocityRPM);
        log("Nectar Hood Position", "%.3f", getState(NectarHoodState.class).getValue());

    }

    public double getPollenRPM() {
        return pollenCurrentVelocityRPM;
    }

    public double getNectarRPM() {
        return nectarCurrentVelocityRPM;
    }


    private void updatePollenVelocity() {
        int currentPosition = pollen.getCurrentPosition();
        double deltaTime = pollenVelocityTimer.seconds();

        if (pollenFirstLoop) {
            pollenCurrentVelocityRPM = 0;
            pollenFirstLoop = false;
        } else if (deltaTime > 0) {
            double deltaPosition = currentPosition - pollenPreviousPosition;
            pollenCurrentVelocityRPM = (deltaPosition / deltaTime) * (60.0 / TICKS_PER_REV);
        }

        pollenPreviousPosition = currentPosition;
        pollenVelocityTimer.reset();
    }

    private void updateNectarVelocity() {
        int currentPosition = nectar.getCurrentPosition();
        double deltaTime = nectarVelocityTimer.seconds();

        if (nectarFirstLoop) {
            nectarCurrentVelocityRPM = 0;
            nectarFirstLoop = false;
        } else if (deltaTime > 0) {
            double deltaPosition = currentPosition - nectarPreviousPosition;
            nectarCurrentVelocityRPM = (deltaPosition / deltaTime) * (60.0 / TICKS_PER_REV);
        }

        nectarPreviousPosition = currentPosition;
        nectarVelocityTimer.reset();
    }

    private void calculatePollenPower() {
        pollenPidController.setGains(pollenTuning.P_Kp, pollenTuning.P_Ki, pollenTuning.P_Kd, 0);
        pollenPidController.kPosition = pollenTuning.P_Kf;
        pollenPidController.update(pollenTargetVelocityRPM, pollenCurrentVelocityRPM);
        pollenPidOutput = Math.max(0, Math.min(1, pollenPidController.calculate()));

        if (getState(PollenState.class) == PollenState.OFF) {
            pollenPidOutput = 0;
        }
    }

    private void calculateNectarPower() {
        nectarPidController.setGains(nectarTuning.N_Kp, nectarTuning.N_Ki, nectarTuning.N_Kd, 0);
        nectarPidController.kPosition = nectarTuning.N_Kf;
        nectarPidController.update(nectarTargetVelocityRPM, nectarCurrentVelocityRPM);
        nectarPidOutput = Math.max(0, Math.min(1, nectarPidController.calculate()));

        if (getState(NectarState.class) == NectarState.OFF) {
            nectarPidOutput = 0;
        }
    }
}
