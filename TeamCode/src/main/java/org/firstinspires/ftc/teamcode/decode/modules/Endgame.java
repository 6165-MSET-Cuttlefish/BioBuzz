package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.endgameTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
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

    public boolean disableServosForEndgame = false;

    public double leftPidflOffset = 0;
    public double rightPidflOffset = 0;

    private boolean firstRead = true;

    public static class FullLiftConfig {
        public double leftMultiplier = 1;
        public double rightMultiplier = 0.96;
        public double leftPower;
        public double rightPower;

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

    public static class LeftPIDFLConfig {
        public double p = 0.01, i = 0.0, d = 0.0002, f = 0.0, l = 0.0;
    }

    public static class RightPIDFLConfig {
        public double p = 0.01, i = 0.0, d = 0.0002, f = 0.0, l = 0.0;
    }

    public static class LeftInitialPIDFLConfig {
        public double p = 0.005, i = 0.0, d = 0.0002, f = 0.0, l = 0.0;
    }

    public static class RightInitialPIDFLConfig {
        public double p = 0.005, i = 0.0, d = 0.0002, f = 0.0, l = 0.0;
    }

    public static class LeftInitialHoldPIDFL {
        public double p = 0.002, i = 0.0, d = 0.0002, f = 0.0, l = 0.0;
    }

    public static class RightInitialHoldPIDFL {
        public double p = 0.002, i = 0.0, d = 0.0002, f = 0.0, l = 0.0;
    }

    public static LeftPIDFLConfig leftPidflConfig = new LeftPIDFLConfig();
    public static RightPIDFLConfig rightPidflConfig = new RightPIDFLConfig();
    public static InitialLiftConfig initialLiftConfig = new InitialLiftConfig();
    public static LeftInitialPIDFLConfig leftInitialPidflConfig = new LeftInitialPIDFLConfig();
    public static RightInitialPIDFLConfig rightInitialPidflConfig = new RightInitialPIDFLConfig();
    public static LeftInitialHoldPIDFL leftInitialHoldPidfl = new LeftInitialHoldPIDFL();
    public static RightInitialHoldPIDFL rightInitialHoldPidfl = new RightInitialHoldPIDFL();
    public static FullLiftConfig fullLiftConfig = new FullLiftConfig();

    public enum FullLiftState implements State {
        INIT(0),
        OFF(0),
        FULL_LIFT(-14000);

        FullLiftState(double value) {
            setValue(value);
        }
    }

    public enum LeftPtoState implements State {
        UP(0.4),
        DOWN(1),
        MANUAL(-1);

        LeftPtoState(double value) {
            setValue(value);
        }
    }

    public enum RightPtoState implements State {
        UP(0.5),
        DOWN(0.3),
        MANUAL(-1);

        RightPtoState(double value) {
            setValue(value);
        }
    }

    public enum InitialState implements State {
        LIFT(700),
        DISABLED(LIFT.getValue()),
        ZERO(0),
        HOLD_BELLYPAN(-150);

        InitialState(double value) {
            setValue(value);
        }
    }

    private double leftPtoPosition = 0.0;
    private double rightPtoPosition = 0.0;
    private double leftInitialPosition = 0.0;
    private double rightInitialPosition = 0.0;

    private final double[] cachedLeftInitial = nanArray(5);
    private final double[] cachedRightInitial = nanArray(5);
    private final double[] cachedLeft = nanArray(5);
    private final double[] cachedRight = nanArray(5);

    public double fullLiftTargetPosition = 0;
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
        setStates(FullLiftState.INIT, LeftPtoState.UP, RightPtoState.UP, InitialState.HOLD_BELLYPAN);
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
            updateController(leftInitialPidfl, leftInitialPidflConfig.p, leftInitialPidflConfig.i, leftInitialPidflConfig.d, leftInitialPidflConfig.f, leftInitialPidflConfig.l, cachedLeftInitial);
            updateController(rightInitialPidfl, rightInitialPidflConfig.p, rightInitialPidflConfig.i, rightInitialPidflConfig.d, rightInitialPidflConfig.f, rightInitialPidflConfig.l, cachedRightInitial);
        } else {
            updateController(leftInitialPidfl, leftInitialHoldPidfl.p, leftInitialHoldPidfl.i, leftInitialHoldPidfl.d, leftInitialHoldPidfl.f, leftInitialHoldPidfl.l, cachedLeftInitial);
            updateController(rightInitialPidfl, rightInitialHoldPidfl.p, rightInitialHoldPidfl.i, rightInitialHoldPidfl.d, rightInitialHoldPidfl.f, rightInitialHoldPidfl.l, cachedRightInitial);
        }

        leftInitialPidfl.setTarget(initialState.getValue() + leftPidflOffset);
        leftInitialPidfl.updatePosition(leftInitialPosition);
        rightInitialPidfl.setTarget(initialState.getValue() + rightPidflOffset);
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
            updateController(leftPidfl, leftPidflConfig.p, leftPidflConfig.i, leftPidflConfig.d, leftPidflConfig.f, leftPidflConfig.l, cachedLeft);
            updateController(rightPidfl, rightPidflConfig.p, rightPidflConfig.i, rightPidflConfig.d, rightPidflConfig.f, rightPidflConfig.l, cachedRight);

            fullLiftTargetPosition = getState(FullLiftState.class).getValue();
            leftPidfl.setTarget(fullLiftTargetPosition);
            leftPidfl.updatePosition(-dt.getFl().getCurrentPosition());
            rightPidfl.setTarget(fullLiftTargetPosition);
            rightPidfl.updatePosition(-dt.getFr().getCurrentPosition());

            fullLiftConfig.leftPower = -1 * Math.max(-1, Math.min(1, leftPidfl.calculate())) * fullLiftConfig.leftMultiplier;
            fullLiftConfig.rightPower = -1 * Math.max(-1, Math.min(1, rightPidfl.calculate())) * fullLiftConfig.rightMultiplier;

            if (fullLiftConfig.enableLinearDeceleration) {
                double leftDecelMultiplier = computeDecelMultiplier(leftPidfl.getError(),
                        fullLiftConfig.decelerationDistanceTicks, fullLiftConfig.minimumMultiplier);
                double rightDecelMultiplier = computeDecelMultiplier(rightPidfl.getError(),
                        fullLiftConfig.decelerationDistanceTicks, fullLiftConfig.minimumMultiplier);
                fullLiftConfig.leftPower *= leftDecelMultiplier;
                fullLiftConfig.rightPower *= rightDecelMultiplier;
            }

            // Must happen in read() so the targets land before Drivetrain.write() runs.
            dt.setRawTargets(fullLiftConfig.leftPower, fullLiftConfig.leftPower,
                    fullLiftConfig.rightPower, fullLiftConfig.rightPower);
        }
    }

    private static double[] nanArray(int size) {
        double[] a = new double[size];
        Arrays.fill(a, Double.NaN);
        return a;
    }

    private static void updateController(PidController controller,
            double p, double i, double d, double f, double l, double[] cache) {
        if (p != cache[0] || i != cache[1] || d != cache[2] || f != cache[3] || l != cache[4]) {
            controller.setGains(p, i, d, l);
            controller.kPosition = f;
            cache[0] = p; cache[1] = i; cache[2] = d; cache[3] = f; cache[4] = l;
        }
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
        if (getState(InitialState.class) == InitialState.DISABLED) {
            disableServosForEndgame = true;
            leftInitial.setPwmDisable();
            rightInitial.setPwmDisable();
        } else {
            disableServosForEndgame = false;
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

            log("Left Endgame Offset", "%.1f", leftPidflOffset);
            log("Right Endgame Offset", "%.1f", rightPidflOffset);

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
                logDashboard("Left Initial Target", "%.2f", getState(InitialState.class).getValue());
                logDashboard("Right Initial Target", "%.2f", getState(InitialState.class).getValue());
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
