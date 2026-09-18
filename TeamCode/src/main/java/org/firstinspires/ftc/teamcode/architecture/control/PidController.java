package org.firstinspires.ftc.teamcode.architecture.control;

import com.qualcomm.robotcore.util.ElapsedTime;

/** PID with feed-forward (kV, kPosition) and a static-friction kick. Fields are public for live {@code @Config} tuning. */
public class PidController {
    public double kP = 0, kI = 0, kD = 0;

    public double kS = 0;
    public double kV = 0;
    public double kPosition = 0;

    public double integralMax = Double.POSITIVE_INFINITY;
    public boolean resetIntegralOnTargetChange = true;

    /** True = D on measurement (no derivative kick on setpoint steps). Default = D on error. */
    public boolean derivativeOnMeasurement = false;

    /** |error| <= this skips the static-friction kick AND freezes the integrator. */
    public double staticDeadband = 0.0;

    public double outputMin = Double.NEGATIVE_INFINITY;
    public double outputMax = Double.POSITIVE_INFINITY;

    public double positionTolerance = 0.0;

    private boolean continuous = false;
    private double minInput = 0;
    private double maxInput = 0;

    private double previousError;
    private double previousPosition;
    private double error;
    private double integralSum;
    private double position;
    private double target;
    private double targetVelocity;
    private double errorDerivative;
    private final ElapsedTime timer;
    private boolean firstUpdate = true;

    public PidController() {
        timer = new ElapsedTime();
    }

    /** Wrap error to take the shortest path across a continuous range (e.g. heading in [-π, π]). */
    public void enableContinuousInput(double minInput, double maxInput) {
        if (maxInput <= minInput) {
            throw new IllegalArgumentException("maxInput must be > minInput");
        }
        this.continuous = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    public void disableContinuousInput() {
        this.continuous = false;
    }

    public boolean isContinuousInputEnabled() {
        return continuous;
    }

    public void setTarget(double newTarget) {
        if (resetIntegralOnTargetChange && Math.abs(newTarget - target) > 1e-9) {
            integralSum = 0;
        }
        this.target = newTarget;
    }

    public double getTarget() { return target; }

    public void setTargetVelocity(double velocity) { this.targetVelocity = velocity; }
    public double getTargetVelocity() { return targetVelocity; }

    public double calculate(double measurement) {
        updatePosition(measurement);
        return calculate();
    }

    /** Uses the current state — call {@link #updatePosition} first. */
    public double calculate() {
        double proportional = error * kP;
        double integral     = integralSum * kI;
        double derivative   = errorDerivative * kD;
        double positionFF   = target * kPosition;
        double velocityFF   = targetVelocity * kV;
        double staticKick   = Math.abs(error) <= staticDeadband ? 0 : Math.signum(error) * kS;

        double output = proportional + integral + derivative + positionFF + velocityFF + staticKick;
        return Math.max(outputMin, Math.min(outputMax, output));
    }

    public void updatePosition(double update) {
        previousPosition = position;
        position = update;
        previousError = error;
        error = wrapError(target - position);

        double deltaTimeSeconds = timer.seconds();
        timer.reset();

        if (firstUpdate) {
            firstUpdate = false;
            errorDerivative = 0;
            return;
        }

        if (deltaTimeSeconds > 0) {
            // Both deltas must go through wrapError: unwrapped, a continuous-input controller sees a
            // ~range jump at the wrap boundary and spikes D. No-op when continuous is disabled.
            if (derivativeOnMeasurement) {
                errorDerivative = -wrapError(position - previousPosition) / deltaTimeSeconds;
            } else {
                errorDerivative = wrapError(error - previousError) / deltaTimeSeconds;
            }
        } else {
            errorDerivative = 0;
        }

        if (Math.abs(error) > staticDeadband) {
            integralSum += error * deltaTimeSeconds;
            if (Math.abs(integralSum) > integralMax) {
                integralSum = Math.signum(integralSum) * integralMax;
            }
        }
    }

    public void update(double target, double measurement) {
        setTarget(target);
        updatePosition(measurement);
    }

    public void reset() {
        previousError = 0;
        previousPosition = 0;
        error = 0;
        integralSum = 0;
        position = 0;
        target = 0;
        targetVelocity = 0;
        errorDerivative = 0;
        firstUpdate = true;
        timer.reset();
    }

    public double getError() { return error; }
    public double getErrorDerivative() { return errorDerivative; }
    public double getIntegralSum() { return integralSum; }
    public double getPosition() { return position; }

    public boolean atSetpoint() {
        return Math.abs(error) <= positionTolerance;
    }

    public boolean atSetpoint(double tolerance) {
        return Math.abs(error) <= tolerance;
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setGains(double kP, double kI, double kD, double kS) {
        setGains(kP, kI, kD);
        this.kS = kS;
    }

    public void setIntegralMax(double integralMax) {
        this.integralMax = integralMax;
    }

    public PidController withGains(double kP, double kI, double kD) {
        setGains(kP, kI, kD);
        return this;
    }

    public PidController withGains(double kP, double kI, double kD, double kS) {
        setGains(kP, kI, kD, kS);
        return this;
    }

    public PidController withFeedforward(double kV, double kPosition) {
        this.kV = kV;
        this.kPosition = kPosition;
        return this;
    }

    public PidController withIntegralMax(double max) {
        this.integralMax = max;
        return this;
    }

    public PidController withStaticDeadband(double deadband) {
        this.staticDeadband = deadband;
        return this;
    }

    public PidController withOutputBounds(double min, double max) {
        if (max <= min) throw new IllegalArgumentException("max must be > min");
        this.outputMin = min;
        this.outputMax = max;
        return this;
    }

    public PidController withPositionTolerance(double tolerance) {
        this.positionTolerance = tolerance;
        return this;
    }

    public PidController withContinuousInput(double minInput, double maxInput) {
        enableContinuousInput(minInput, maxInput);
        return this;
    }

    private double wrapError(double e) {
        if (!continuous) return e;
        double range = maxInput - minInput;
        double halfRange = range / 2.0;
        // Double-mod shifts into [-halfRange, halfRange) despite Java's signed %.
        return ((e + halfRange) % range + range) % range - halfRange;
    }
}
