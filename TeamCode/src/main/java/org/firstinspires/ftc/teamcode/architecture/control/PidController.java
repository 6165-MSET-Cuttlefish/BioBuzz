package org.firstinspires.ftc.teamcode.architecture.control;

import com.qualcomm.robotcore.util.ElapsedTime;

/** PID with a position feed-forward (kPosition) and a static-friction kick. */
public class PidController {
    public double kP = 0, kI = 0, kD = 0;

    public double kS = 0;
    public double kPosition = 0;

    public boolean resetIntegralOnTargetChange = true;

    /** True = D on measurement (no derivative kick on setpoint steps). Default = D on error. */
    public boolean derivativeOnMeasurement = false;

    /** |error| <= this skips the static-friction kick AND freezes the integrator. */
    public double staticDeadband = 0.0;

    private boolean continuous = false;
    private double minInput = 0;
    private double maxInput = 0;

    private double previousError;
    private double previousPosition;
    private double error;
    private double integralSum;
    private double position;
    private double target;
    private double errorDerivative;
    private final ElapsedTime timer;
    private boolean firstUpdate = true;

    public PidController() {
        timer = new ElapsedTime();
    }

    public void setTarget(double newTarget) {
        if (resetIntegralOnTargetChange && Math.abs(newTarget - target) > 1e-9) {
            integralSum = 0;
        }
        this.target = newTarget;
    }

    /** Uses the current state — call {@link #updatePosition} first. */
    public double calculate() {
        double proportional = error * kP;
        double integral     = integralSum * kI;
        double derivative   = errorDerivative * kD;
        double positionFF   = target * kPosition;
        double staticKick   = Math.abs(error) <= staticDeadband ? 0 : Math.signum(error) * kS;

        return proportional + integral + derivative + positionFF + staticKick;
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
            // Both deltas go through wrapError, or a continuous-input controller spikes D at the wrap boundary.
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
        errorDerivative = 0;
        firstUpdate = true;
        timer.reset();
    }

    public double getError() { return error; }

    public void setGains(double kP, double kI, double kD, double kS) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
    }

    public PidController withGains(double kP, double kI, double kD, double kS) {
        setGains(kP, kI, kD, kS);
        return this;
    }

    /** Wrap error to take the shortest path across a continuous range (e.g. heading in [-π, π]). */
    public PidController withContinuousInput(double minInput, double maxInput) {
        if (maxInput <= minInput) {
            throw new IllegalArgumentException("maxInput must be > minInput");
        }
        this.continuous = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
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
