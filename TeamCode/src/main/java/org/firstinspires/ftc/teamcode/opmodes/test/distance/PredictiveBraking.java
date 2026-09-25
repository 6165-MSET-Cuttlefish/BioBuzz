package org.firstinspires.ftc.teamcode.opmodes.test.distance;

import com.acmerobotics.dashboard.config.Config;

/**
 * Caps approach power so the robot settles on {@link Tuning#stopDistanceCm} instead of hitting the
 * obstacle. Readings are stale by a whole ping, so it brakes on a dead-reckoned model (kinematic law)
 * or on the reading aged forward (legacy curve), then the settle trim closes the loop at rest.
 * Measure {@link Tuning#brakingDecelCmPerSec2} as v² / (2·coast) from a power-cut coast at a known
 * closing speed. On overshoot, lower it, or raise {@link Tuning#reactionSeconds} if the miss is constant
 * across speeds; under the legacy curve raise {@link Tuning#stopLeadSeconds}.
 */
public class PredictiveBraking {

    public interface DistanceReader {
        /** Return {@link Double#NaN} for an invalid read. */
        double getDistanceCm();
    }

    @Config("Braking Law")
    public static class Tuning {
        public static double stopDistanceCm = 25;
        /** Legacy curve: full power beyond this, exponential decay inside it. */
        public static double fullPowerAboveCm = 80;
        public static double powerAtThreshold = 0.9;
        /** Legacy curve steepness: ~0 is a linear ramp, higher sheds speed sooner after the threshold. */
        public static double decelRate = 2.0;
        /** The driver's fixed ping/wait delay; the time a value then sits unchanged is measured on top. */
        public static double sensorLatencySeconds = 0.10;
        /** Legacy curve: travel time the stop is brought forward by (motor response plus coast). */
        public static double stopLeadSeconds = 0.15;
        /** Low-pass on the closing-speed estimate, 0-1. */
        public static double velocityFilter = 0.4;
        /** Power floor inside the braking zone, below which the drivetrain stalls. */
        public static double minCreepPower = 0.10;
        public static boolean latchOnStop = true;
        /** At rest, creep in or out until within this of stopDistanceCm; 0 disables. */
        public static double settleToleranceCm = 3;
        public static double settlePower = 0.18;
        /** Reverse power pulsed at the stop distance to kill momentum; 0 disables. */
        public static double reverseBrakePower = 0;
        public static double reverseBrakeMinClosingCmPerSec = 15;
        public static int reverseBrakeMaxMs = 250;
        /** Power when stalled short of the stop, since minCreepPower may sit below breakaway friction; 0 disables. */
        public static double antiStallPower = 0.25;
        public static int stallTimeoutMs = 400;
        /** Solve for the brake point from speed instead of the legacy distance-keyed curve. */
        public static boolean useKinematicLaw = true;
        public static double brakingDecelCmPerSec2 = 150;
        /** Speed at full power; converts a target speed into a motor power. */
        public static double topSpeedCmPerSec = 150;
        /** Dead time between commanding the brake and the robot slowing. */
        public static double reactionSeconds = 0.12;
        /** How hard a fresh reading pulls the dead-reckoned model, 0-1; 1 brakes on the lagged reading. */
        public static double sensorTrust = 0.15;
        public static double minValidCm = 2;
        /** At or beyond this there is treated as no obstacle. */
        public static double maxValidCm = 400;
        public static boolean enabled = true;
    }

    private static final double REST_SPEED_CM_PER_SEC = 2;

    private final DistanceReader reader;

    private double lastDistanceCm = Tuning.maxValidCm;
    private double closingVelocityCmPerSec = 0;
    private boolean stopped = false;

    private double lastSampleCm = Double.NaN;
    private long lastSampleNanos = 0;

    private long reverseBrakeStartNanos = 0;
    private boolean reverseBraking = false;

    private double modelDistanceCm = 0;
    private double modelVelocityCmPerSec = 0;
    private boolean modelValid = false;
    private boolean brakeEngaged = false;
    private long lastUpdateNanos = 0;

    public PredictiveBraking(DistanceReader reader) {
        this.reader = reader;
    }

    /** Call once per loop, before {@link #clampApproachPower}. */
    public void read() {
        double d = reader.getDistanceCm();
        if (Double.isNaN(d) || d < Tuning.minValidCm) {
            return;
        }

        // Differentiate only changed readings: a pinging sensor repeats one value between pings.
        boolean freshSample = Double.isNaN(lastSampleCm) || d != lastSampleCm;
        if (freshSample && !Double.isNaN(lastSampleCm)) {
            double dtSec = (System.nanoTime() - lastSampleNanos) / 1e9;
            if (dtSec > 0) {
                double instantaneous = (lastSampleCm - d) / dtSec;
                closingVelocityCmPerSec += Tuning.velocityFilter * (instantaneous - closingVelocityCmPerSec);
            }
        }
        if (freshSample) {
            lastSampleCm = d;
            lastSampleNanos = System.nanoTime();
        } else if (isStalled()) {
            // Only a changed reading refreshes the estimate, so a stopped robot would keep its old speed.
            closingVelocityCmPerSec = 0;
        }

        lastDistanceCm = d;
        updateModel(freshSample);
    }

    // While braking, speed follows brakingDecelCmPerSec2: the differentiated reading lags by more than the whole stop.
    private void updateModel(boolean freshSample) {
        long now = System.nanoTime();
        double dtSec = lastUpdateNanos == 0 ? 0 : (now - lastUpdateNanos) / 1e9;
        lastUpdateNanos = now;

        if (!modelValid) {
            modelDistanceCm = lastDistanceCm;
            modelVelocityCmPerSec = closingVelocityCmPerSec;
            modelValid = true;
            return;
        }

        if (brakeEngaged) {
            modelVelocityCmPerSec = Math.max(0, modelVelocityCmPerSec - Tuning.brakingDecelCmPerSec2 * dtSec);
        } else {
            modelVelocityCmPerSec += Tuning.velocityFilter * (closingVelocityCmPerSec - modelVelocityCmPerSec);
        }
        modelDistanceCm = Math.max(0, modelDistanceCm - modelVelocityCmPerSec * dtSec);

        if (freshSample) {
            // Age the sample forward by the sensor latency, or each correction drags the model back by that lag.
            double impliedNow = lastSampleCm - modelVelocityCmPerSec * Tuning.sensorLatencySeconds;
            modelDistanceCm += Tuning.sensorTrust * (impliedNow - modelDistanceCm);
        }
    }

    public double getModelDistanceCm() {
        return modelDistanceCm;
    }

    public double getModelVelocityCmPerSec() {
        return modelVelocityCmPerSec;
    }

    public double getStoppingDistanceCm() {
        double v = modelVelocityCmPerSec;
        if (v <= 0 || Tuning.brakingDecelCmPerSec2 <= 0) {
            return 0;
        }
        return v * Tuning.reactionSeconds + (v * v) / (2 * Tuning.brakingDecelCmPerSec2);
    }

    private boolean isStalled() {
        return lastSampleNanos != 0
                && (System.nanoTime() - lastSampleNanos) / 1e6 >= Tuning.stallTimeoutMs;
    }

    public double getDistanceCm() {
        return lastDistanceCm;
    }

    private double sampleAgeSec() {
        if (lastSampleNanos == 0) {
            return Tuning.sensorLatencySeconds;
        }
        return Tuning.sensorLatencySeconds + (System.nanoTime() - lastSampleNanos) / 1e9;
    }

    /** The reading corrected for its own age. */
    public double getCurrentDistanceCm() {
        return Math.max(0, lastDistanceCm - closingVelocityCmPerSec * sampleAgeSec());
    }

    /** Positive means approaching. */
    public double getClosingVelocityCmPerSec() {
        return closingVelocityCmPerSec;
    }

    public double getPredictedDistanceCm() {
        return Math.max(0, getCurrentDistanceCm() - closingVelocityCmPerSec * Tuning.stopLeadSeconds);
    }

    public boolean isBraking() {
        if (!Tuning.enabled) {
            return false;
        }
        return Tuning.useKinematicLaw
                ? brakeEngaged
                : getPredictedDistanceCm() < Tuning.fullPowerAboveCm;
    }

    private double controlDistanceCm() {
        return Tuning.useKinematicLaw ? modelDistanceCm : getPredictedDistanceCm();
    }

    public boolean isStopped() {
        return stopped;
    }

    public void resetStop() {
        stopped = false;
        reverseBraking = false;
        brakeEngaged = false;
        modelValid = false;
    }

    /** Telemetry only: unlike {@link #clampApproachPower} it leaves the stop latch and reverse pulse alone. */
    public double getPowerCeiling() {
        if (!Tuning.enabled || lastDistanceCm >= Tuning.maxValidCm) {
            return 1.0;
        }
        double control = controlDistanceCm();
        if (control <= Tuning.stopDistanceCm || stopped) {
            return 0;
        }
        if (Tuning.useKinematicLaw) {
            return Math.max(kinematicCeiling(control), stallEscapePower());
        }
        if (control >= Tuning.fullPowerAboveCm) {
            return 1.0;
        }
        return Math.max(curveCeiling(control), stallEscapePower());
    }

    /** Positive power approaches the obstacle; negative passes through unchanged. */
    public double clampApproachPower(double requestedPower) {
        if (!Tuning.enabled || requestedPower <= 0 || lastDistanceCm >= Tuning.maxValidCm) {
            return requestedPower;
        }

        double control = controlDistanceCm();

        boolean atStop = control <= Tuning.stopDistanceCm;
        if (atStop) {
            stopped = Tuning.latchOnStop;
        }
        if (atStop || stopped) {
            brakeEngaged = false;
            double pulse = reverseBrakePulse();
            return pulse != 0 ? pulse : settleCorrection();
        }
        reverseBraking = false;

        if (Tuning.useKinematicLaw) {
            // Latched: as the modelled speed falls the brake point recedes, so re-deciding would chatter.
            brakeEngaged |= control - Tuning.stopDistanceCm <= getStoppingDistanceCm();
            if (!brakeEngaged) {
                return requestedPower;
            }
            double ceiling = Math.max(kinematicCeiling(control), stallEscapePower());
            return Math.min(requestedPower, ceiling);
        }

        if (control >= Tuning.fullPowerAboveCm) {
            return requestedPower;
        }
        return Math.min(requestedPower, Math.max(curveCeiling(control), stallEscapePower()));
    }

    private static double kinematicCeiling(double controlDistanceCm) {
        double remaining = controlDistanceCm - Tuning.stopDistanceCm;
        if (remaining <= 0 || Tuning.topSpeedCmPerSec <= 0) {
            return 0;
        }
        double targetSpeed = Math.sqrt(2 * Math.max(0, Tuning.brakingDecelCmPerSec2) * remaining);
        double ceiling = targetSpeed / Tuning.topSpeedCmPerSec;
        return Math.max(Tuning.minCreepPower, Math.min(1.0, ceiling));
    }

    private boolean isAtRest() {
        return isStalled() && Math.abs(modelVelocityCmPerSec) < REST_SPEED_CM_PER_SEC;
    }

    // Gated on isAtRest(): while moving the raw reading is stale and would drive the robot through its own stop.
    private double settleCorrection() {
        if (Tuning.settleToleranceCm <= 0 || Tuning.settlePower <= 0 || !isAtRest()) {
            return 0;
        }
        double error = lastDistanceCm - Tuning.stopDistanceCm;
        if (Math.abs(error) <= Tuning.settleToleranceCm) {
            return 0;
        }
        double power = Math.min(1.0, Tuning.settlePower);
        return error > 0 ? power : -power;
    }

    public boolean isSettling() {
        return stopped && settleCorrection() != 0;
    }

    private double stallEscapePower() {
        if (Tuning.antiStallPower <= 0 || !isAtRest()) {
            return 0;
        }
        return Math.min(1.0, Tuning.antiStallPower);
    }

    public boolean isStallEscaping() {
        return stallEscapePower() > 0 && !stopped && lastDistanceCm > Tuning.stopDistanceCm;
    }

    /** Callers must have already established stopDistanceCm < predicted < fullPowerAboveCm. */
    private static double curveCeiling(double predicted) {
        double t = (predicted - Tuning.stopDistanceCm) / (Tuning.fullPowerAboveCm - Tuning.stopDistanceCm);
        double ceiling = Tuning.minCreepPower + (Tuning.powerAtThreshold - Tuning.minCreepPower) * decayShape(t);
        return Math.max(Tuning.minCreepPower, Math.min(1.0, ceiling));
    }

    // Time-boxed, not run until the speed estimate reads zero: that only refreshes on a new sample.
    private double reverseBrakePulse() {
        if (Tuning.reverseBrakePower <= 0) {
            reverseBraking = false;
            return 0;
        }
        if (!reverseBraking) {
            if (closingVelocityCmPerSec < Tuning.reverseBrakeMinClosingCmPerSec) {
                return 0;
            }
            reverseBraking = true;
            reverseBrakeStartNanos = System.nanoTime();
        }
        if ((System.nanoTime() - reverseBrakeStartNanos) / 1e6 >= Tuning.reverseBrakeMaxMs) {
            return 0;
        }
        return -Math.min(1.0, Tuning.reverseBrakePower);
    }

    public boolean isReverseBraking() {
        return reverseBraking
                && Tuning.reverseBrakePower > 0
                && (System.nanoTime() - reverseBrakeStartNanos) / 1e6 < Tuning.reverseBrakeMaxMs;
    }

    private static double decayShape(double t) {
        double k = Tuning.decelRate;
        if (Math.abs(k) < 1e-6) {
            return t;
        }
        return (Math.exp(k * t) - 1) / (Math.exp(k) - 1);
    }
}
