package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.endgameTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.control.PidController;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedCRServo;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;

@Config("Decode Endgame")
public class Endgame extends Module {
    private final EnhancedServo leftPto;
    private final EnhancedServo rightPto;
    private final EnhancedCRServo leftInitial;
    private final EnhancedCRServo rightInitial;

    public AbsoluteAnalogEncoder leftInitialEncoder;
    public AbsoluteAnalogEncoder rightInitialEncoder;

    private final PidController leftPidfl = new PidController();
    private final PidController rightPidfl = new PidController();
    private final PidController leftInitialPidfl = new PidController();
    private final PidController rightInitialPidfl = new PidController();

    private Drivetrain drivetrain;

    private boolean firstRead = true;

    public static class FullLiftConfig {
        public double leftMultiplier = 1;
        public double rightMultiplier = 0.96;

        public boolean enableLinearDeceleration = true;
        public double decelerationDistanceTicks = 1000;
        public double minimumMultiplier = 0.3;
    }

    public static class InitialLiftConfig {
        public double leftMultiplier = 1;
        public double rightMultiplier = 1;

        public boolean enableLinearDeceleration = false;
        public double decelerationDistance = 10.0;
        public double minimumDecelerationPower = 0.1;
    }

    /** {@code l} is the static-friction kick (kS); {@code f} is the position feed-forward. */
    public static class Pidfl {
        public double p, i, d, f, l;

        Pidfl(double p, double d) {
            this.p = p;
            this.d = d;
        }
    }

    public static Pidfl leftPidflConfig = new Pidfl(0.01, 0.0002);
    public static Pidfl rightPidflConfig = new Pidfl(0.01, 0.0002);
    public static InitialLiftConfig initialLiftConfig = new InitialLiftConfig();
    public static Pidfl leftInitialPidflConfig = new Pidfl(0.005, 0.0002);
    public static Pidfl rightInitialPidflConfig = new Pidfl(0.005, 0.0002);
    public static Pidfl leftInitialHoldPidfl = new Pidfl(0.002, 0.0002);
    public static Pidfl rightInitialHoldPidfl = new Pidfl(0.002, 0.0002);
    public static FullLiftConfig fullLiftConfig = new FullLiftConfig();

    public enum FullLiftState implements State {
        OFF(0),
        FULL_LIFT(-14000);

        FullLiftState(double value) {
            setValue(value);
        }
    }

    public enum LeftPtoState implements State {
        UP(0.4),
        DOWN(1);

        LeftPtoState(double value) {
            setValue(value);
        }
    }

    public enum RightPtoState implements State {
        UP(0.5),
        DOWN(0.3);

        RightPtoState(double value) {
            setValue(value);
        }
    }

    public enum InitialState implements State {
        LIFT(700),
        DISABLED(LIFT.getValue()),
        HOLD_BELLYPAN(-150);

        InitialState(double value) {
            setValue(value);
        }
    }

    private double leftPtoPosition = 0.0;
    private double rightPtoPosition = 0.0;
    private double leftInitialPosition = 0.0;
    private double rightInitialPosition = 0.0;

    private double fullLiftTargetPosition = 0;
    private double fullLiftLeftPower = 0;
    private double fullLiftRightPower = 0;
    private double leftInitialPower = 0;
    private double rightInitialPower = 0;

    public Endgame(HardwareMap hardwareMap) {
        setTelemetryEnabled(endgameTelemetry.TOGGLE);

        leftPto = new EnhancedServo(hardwareMap, "leftPto");
        rightPto = new EnhancedServo(hardwareMap, "rightPto");
        leftInitial = new EnhancedCRServo(hardwareMap, "leftInitial").withCachingTolerance(0.01);
        rightInitial = new EnhancedCRServo(hardwareMap, "rightInitial").withCachingTolerance(0.01);
        leftInitialEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "leftInitialEncoder"), 0, 1.0, false);
        rightInitialEncoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "rightInitialEncoder"), 0, 1.0, false);

        rightInitial.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** The full lift drives through the drivetrain's motors via the PTOs; required before FULL_LIFT or telemetry. */
    public Endgame withDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        return this;
    }

    private Drivetrain requireDrivetrain() {
        if (drivetrain == null) {
            throw new IllegalStateException("Endgame needs the Drivetrain; pass it with withDrivetrain()");
        }
        return drivetrain;
    }

    @Override
    protected void initStates() {
        setStates(FullLiftState.OFF, LeftPtoState.UP, RightPtoState.UP, InitialState.HOLD_BELLYPAN);
    }

    @Override
    protected void read() {
        if (firstRead) {
            leftInitialEncoder.zero();
            rightInitialEncoder.zero();
            firstRead = false;
            return;
        }

        leftInitialPosition = leftInitialEncoder.getRelativePosition();
        rightInitialPosition = -rightInitialEncoder.getRelativePosition();

        leftPtoPosition = getState(LeftPtoState.class).getValue();
        rightPtoPosition = getState(RightPtoState.class).getValue();

        InitialState initialState = getState(InitialState.class);
        if (initialState == InitialState.LIFT) {
            applyGains(leftInitialPidfl, leftInitialPidflConfig);
            applyGains(rightInitialPidfl, rightInitialPidflConfig);
        } else {
            applyGains(leftInitialPidfl, leftInitialHoldPidfl);
            applyGains(rightInitialPidfl, rightInitialHoldPidfl);
        }

        leftInitialPidfl.setTarget(initialState.getValue());
        leftInitialPidfl.updatePosition(leftInitialPosition);
        rightInitialPidfl.setTarget(initialState.getValue());
        rightInitialPidfl.updatePosition(rightInitialPosition);

        double leftPower = leftInitialPidfl.calculate();
        double rightPower = rightInitialPidfl.calculate();

        if (initialLiftConfig.enableLinearDeceleration) {
            leftPower = applyLinearDeceleration(leftPower, leftInitialPidfl.getError(),
                    initialLiftConfig.decelerationDistance, initialLiftConfig.minimumDecelerationPower);
            rightPower = applyLinearDeceleration(rightPower, rightInitialPidfl.getError(),
                    initialLiftConfig.decelerationDistance, initialLiftConfig.minimumDecelerationPower);
        }

        leftInitialPower = leftPower * initialLiftConfig.leftMultiplier;
        rightInitialPower = -rightPower * initialLiftConfig.rightMultiplier;

        if (getState(FullLiftState.class) == FullLiftState.FULL_LIFT) {
            Drivetrain dt = requireDrivetrain();
            applyGains(leftPidfl, leftPidflConfig);
            applyGains(rightPidfl, rightPidflConfig);

            fullLiftTargetPosition = getState(FullLiftState.class).getValue();
            leftPidfl.setTarget(fullLiftTargetPosition);
            leftPidfl.updatePosition(-dt.getFl().getCurrentPosition());
            rightPidfl.setTarget(fullLiftTargetPosition);
            rightPidfl.updatePosition(-dt.getFr().getCurrentPosition());

            fullLiftLeftPower = -1 * Math.max(-1, Math.min(1, leftPidfl.calculate())) * fullLiftConfig.leftMultiplier;
            fullLiftRightPower = -1 * Math.max(-1, Math.min(1, rightPidfl.calculate())) * fullLiftConfig.rightMultiplier;

            if (fullLiftConfig.enableLinearDeceleration) {
                double leftDecelMultiplier = computeDecelMultiplier(leftPidfl.getError(),
                        fullLiftConfig.decelerationDistanceTicks, fullLiftConfig.minimumMultiplier);
                double rightDecelMultiplier = computeDecelMultiplier(rightPidfl.getError(),
                        fullLiftConfig.decelerationDistanceTicks, fullLiftConfig.minimumMultiplier);
                fullLiftLeftPower *= leftDecelMultiplier;
                fullLiftRightPower *= rightDecelMultiplier;
            }

            // Must happen in read() so the targets land before Drivetrain.write() runs.
            dt.setRawTargets(fullLiftLeftPower, fullLiftLeftPower,
                    fullLiftRightPower, fullLiftRightPower);
        }
    }

    private static void applyGains(PidController controller, Pidfl gains) {
        controller.setGains(gains.p, gains.i, gains.d, gains.l);
        controller.kPosition = gains.f;
    }

    private static double computeDecelMultiplier(double error, double decelerationDistance, double minimumMultiplier) {
        double absError = Math.abs(error);
        if (absError >= decelerationDistance) return 1.0;
        if (absError <= 0) return minimumMultiplier;
        return minimumMultiplier + (1.0 - minimumMultiplier) * (absError / decelerationDistance);
    }

    private static double applyLinearDeceleration(double power, double error, double decelerationDistance,
                                                  double minimumPower) {
        double absError = Math.abs(error);
        if (absError <= decelerationDistance && absError > 0) {
            double decelerationFactor = absError / decelerationDistance;
            double signedMinPower = Math.signum(power) * minimumPower;
            return signedMinPower + (power - signedMinPower) * decelerationFactor;
        }
        return power;
    }

    @Override
    protected void write() {
        if (servosDisabled()) {
            leftInitial.setPwmDisable();
            rightInitial.setPwmDisable();
        } else {
            if (!leftInitial.isPwmEnabled()) {
                leftInitial.setPwmEnable();
            }
            if (!rightInitial.isPwmEnabled()) {
                rightInitial.setPwmEnable();
            }
            leftInitial.setPower(leftInitialPower);
            rightInitial.setPower(rightInitialPower);
        }

        leftPto.setPosition(getState(LeftPtoState.class).getValue());
        rightPto.setPosition(getState(RightPtoState.class).getValue());
    }

    @Override
    public void stop() {
        leftInitial.setPower(0);
        rightInitial.setPower(0);
    }

    /** True while the endgame has the initial-lift servos PWM-disabled; turret and headlight servos follow suit. */
    public boolean servosDisabled() {
        return getState(InitialState.class) == InitialState.DISABLED;
    }

    public boolean initialLiftComplete() {
        return Math.abs(leftInitialPosition - getState(InitialState.class).getValue()) < 70 &&
                Math.abs(rightInitialPosition - getState(InitialState.class).getValue()) < 70;
    }

    @Override
    public int telemetryOrder() { return Integer.MAX_VALUE; }

    @Override
    protected void onTelemetry() {
        if (endgameTelemetry.TOGGLE) {
            Drivetrain dt = requireDrivetrain();

            logDashboard("Full Lift State", getState(FullLiftState.class));
            log("FL Position (ticks)", dt.getFl().getCurrentPosition());
            log("FR Position (ticks)", dt.getFr().getCurrentPosition());
            logDashboard("Full Lift Target", "%.1f", fullLiftTargetPosition);

            logDashboard("Left PID Power", "%.3f", leftPidfl.calculate());
            logDashboard("Right PID Power", "%.3f", rightPidfl.calculate());
            logDashboard("Left PID Error", "%.3f", leftPidfl.getError());
            logDashboard("Right PID Error", "%.3f", rightPidfl.getError());

            if (endgameTelemetry.current) {
                logDashboard("FL Current (A)", "%.2f", dt.getFl().getCurrent(CurrentUnit.AMPS));
                logDashboard("FR Current (A)", "%.2f", dt.getFr().getCurrent(CurrentUnit.AMPS));
                logDashboard("BR Current (A)", "%.2f", dt.getBr().getCurrent(CurrentUnit.AMPS));
                logDashboard("BL Current (A)", "%.2f", dt.getBl().getCurrent(CurrentUnit.AMPS));
            }

            if (endgameTelemetry.pto) {
                logDashboard("Left PTO State", getState(LeftPtoState.class));
                logDashboard("Right PTO State", getState(RightPtoState.class));
                logDashboard("Left PTO Position", "%.3f", leftPtoPosition);
                logDashboard("Right PTO Position", "%.3f", rightPtoPosition);
            }

            if (endgameTelemetry.initial) {
                logDashboard("Initial Lift State", getState(InitialState.class));
                logDashboard("Initial Target", "%.2f", getState(InitialState.class).getValue());
                log("Left Initial Encoder (deg)", "%.2f", leftInitialPosition);
                log("Right Initial Encoder (deg)", "%.2f", rightInitialPosition);
                logDashboard("Left Initial Voltage", "%.2f", leftInitialEncoder.getVoltage());
                logDashboard("Right Initial Voltage", "%.2f", rightInitialEncoder.getVoltage());
                logDashboard("Left Initial PID Error", "%.3f", leftInitialPidfl.getError());
                logDashboard("Right Initial PID Error", "%.3f", rightInitialPidfl.getError());
                logDashboard("Left Initial PID Power", "%.3f", leftInitialPidfl.calculate());
                logDashboard("Right Initial PID Power", "%.3f", rightInitialPidfl.calculate());
                logDashboard("Left Initial Applied Power", "%.3f", leftInitialPower);
                logDashboard("Right Initial Applied Power", "%.3f", rightInitialPower);
            }
        }
    }
}
