package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;

/**
 * Scales a requested drive power down as the robot nears an obstacle, so holding full speed at a
 * wall decelerates smoothly and settles on {@link Tuning#stopDistanceCm} instead of colliding.
 *
 * Sensor-agnostic on purpose — which distance sensor we end up on isn't decided yet, so this takes
 * a {@link DistanceReader} lambda instead of a concrete sensor type. See
 * {@link PredictiveBrakingUltrasonicTest} and {@link PredictiveBrakingLaserTest}.
 *
 * <h2>Why "predictive"</h2>
 * The distance reading is always stale — the MB1242 only produces a new sample every ~100 ms, and
 * at 150 cm/s the robot covers 15 cm in that time. Braking on the raw reading therefore always
 * brakes late by roughly one sample period, which no amount of curve tuning can fix. So closing
 * speed is estimated by differentiating successive readings, and every decision is made on a
 * corrected distance rather than on what the sensor last said. Two separate corrections apply, and
 * conflating them is what makes this thing miss its target:
 * <ol>
 *   <li>{@link #getCurrentDistanceCm()} — the reading minus its own age
 *       ({@link Tuning#sensorLatencySeconds} plus the measured time it has sat unchanged). This
 *       recovers where the robot is <em>now</em>. It is a measurement fix, not a preference.</li>
 *   <li>{@link #getPredictedDistanceCm()} — that, minus {@link Tuning#stopLeadSeconds} of further
 *       travel, covering motor response plus the coast after power is cut. This is where the robot
 *       ends up if the brakes go on this instant, and it is what the stop triggers on.</li>
 * </ol>
 *
 * <h2>Why it still won't be exact, and what fixes that</h2>
 * Coast distance depends on battery charge, floor grip and robot weight, so no open-loop lead time
 * lands on the target every run. {@link Tuning#settleToleranceCm} closes the loop: once stopped, the
 * robot creeps in or backs out until the reading actually sits on {@link Tuning#stopDistanceCm}.
 * That, not the curve, is what guarantees the final position.
 *
 * <h2>The kinematic law ({@link Tuning#useKinematicLaw}, the default)</h2>
 * Rather than reacting to each reading, this solves for where the brake has to start and then flies
 * the stop on its own physics. Distance and speed are dead-reckoned every loop
 * ({@link #getModelDistanceCm()}), and once braking, speed follows
 * {@link Tuning#brakingDecelCmPerSec2} instead of the sensor — the differentiated reading lags by
 * more than the whole stop takes, so during the one manoeuvre that matters it reports a speed the
 * robot no longer has. Readings only trim drift, weighted by {@link Tuning#sensorTrust}.
 * <pre>
 *   stoppingDistance = v·reactionSeconds + v² / (2·brakingDecel)   // brake here
 *   targetSpeed(d)   = sqrt(2·brakingDecel·(d - stopDistanceCm))   // taper to here
 *   ceiling          = targetSpeed / topSpeedCmPerSec
 * </pre>
 * Both lines come from the same constant, so the trigger and the taper agree by construction. The
 * payoff is that stopping distance now scales with v² as it physically must: one tune holds across
 * approach speeds, where a distance-keyed curve can only ever be right at one of them.
 *
 * <h3>Measuring brakingDecelCmPerSec2</h3>
 * Don't guess it. Drive at a known speed (read {@code closing cm/s} while cruising), cut power, and
 * measure the coast in cm. Then {@code a = v² / (2·coast)} — 150 cm/s stopping in 75 cm is 150 cm/s².
 * Too low and it brakes absurdly early; too high and it brakes late and overshoots.
 *
 * <h2>The legacy exponential curve ({@code useKinematicLaw = false})</h2>
 * Full requested power all the way in to {@link Tuning#fullPowerAboveCm}. Inside that, power decays
 * exponentially from {@link Tuning#powerAtThreshold} down to zero at {@link Tuning#stopDistanceCm}:
 * <pre>
 *   t       = (predicted - stopDistanceCm) / (fullPowerAboveCm - stopDistanceCm)  // 1 at threshold, 0 at stop
 *   shape   = (exp(decelRate * t) - 1) / (exp(decelRate) - 1)                     // 1 at threshold, 0 at stop
 *   ceiling = minCreepPower + (powerAtThreshold - minCreepPower) * shape
 * </pre>
 * {@link Tuning#decelRate} is the only shape knob:
 * <ul>
 *   <li>→0 — straight linear ramp from powerAtThreshold to zero.</li>
 *   <li>2 — power falls off quickly just inside the threshold, then eases into a slower approach.</li>
 *   <li>5+ — power collapses almost immediately at the threshold, then a long crawl in.</li>
 * </ul>
 * So raise decelRate to shed speed harder and sooner after the threshold, lower it toward zero for
 * an even ramp. Where braking <em>begins</em> is fullPowerAboveCm; decelRate only controls the
 * shape between there and the stop.
 *
 * <h2>If it overshoots</h2>
 * Under the kinematic law an overshoot means the modelled deceleration is optimistic: lower
 * {@link Tuning#brakingDecelCmPerSec2} so it allows itself more room, or raise
 * {@link Tuning#reactionSeconds} if the miss is roughly constant rather than growing with speed.
 * Under the legacy curve the knob is {@link Tuning#stopLeadSeconds}. Either way the settle trim
 * still lands the final position; the tuning only decides how gracefully it arrives.
 *
 * Apart from the reverse pulse and the settle trim this only pulls power down. It never adds power
 * to an approach and never touches a negative (backing away) request, so the driver can always
 * reverse out of a corner.
 */
public class PredictiveBraking {

    public interface DistanceReader {
        /** Latest distance reading, in centimeters. Return {@link Double#NaN} for an invalid read. */
        double getDistanceCm();
    }

    // Slothboard registers @Config classes by getSimpleName(), which for a nested class is just
    // "Tuning" — every nested Tuning in the project would collide on one key and all but one would
    // vanish from the dashboard. Nested @Config classes must carry an explicit unique name.
    @Config("Braking Law")
    public static class Tuning {
        /** Standoff the robot should settle on. Power is hard-zeroed at or inside this. */
        public static double stopDistanceCm = 25;
        /** Full requested power at any distance beyond this. Braking only happens inside it. */
        public static double fullPowerAboveCm = 80;
        /** Power the instant the robot crosses fullPowerAboveCm — the top of the decay curve. */
        public static double powerAtThreshold = 0.9;
        /** Exponential steepness inside the threshold. Higher sheds speed sooner. See class javadoc. */
        public static double decelRate = 2.0;
        /**
         * Fixed delay between the echo bouncing and the value arriving here — the driver's ping/wait
         * cycle. Time the value then sat unchanged is measured, not guessed, and added on top.
         */
        public static double sensorLatencySeconds = 0.10;
        /**
         * Travel time the stop is brought forward by, covering motor response plus the coast after
         * power is cut. This is the overshoot knob: raise it if the robot stops past the target.
         */
        public static double stopLeadSeconds = 0.15;
        /** Low-pass on the closing-speed estimate, 0-1. Lower is smoother but laggier. */
        public static double velocityFilter = 0.4;
        /** Power floor inside the braking zone — below this the drivetrain stalls instead of creeping in. */
        public static double minCreepPower = 0.10;
        /** Once stopped, stay stopped, instead of creeping and re-braking around the target. */
        public static boolean latchOnStop = true;
        /**
         * After stopping, creep in or back out until the reading is within this of stopDistanceCm.
         * This is what actually guarantees the final position — the open-loop approach can only ever
         * get close, since coast distance varies with battery, floor and load. 0 disables.
         */
        public static double settleToleranceCm = 3;
        /** Power for the settle creep. Must be above the drivetrain's breakaway friction to do anything. */
        public static double settlePower = 0.18;
        /**
         * Reverse power pulsed at the stop distance to kill momentum the coast-down can't. 0 disables
         * (pure coast). This is the only knob that shortens the stop itself rather than moving where
         * the slowdown begins.
         */
        public static double reverseBrakePower = 0;
        /** Skip the reverse pulse when already closing slower than this — coasting settles it. */
        public static double reverseBrakeMinClosingCmPerSec = 15;
        /** Hard cap on a reverse pulse, so a bad velocity estimate can't drive the robot backwards. */
        public static int reverseBrakeMaxMs = 250;
        /**
         * Power commanded when the robot has stalled short of the stop distance — the creep floor is
         * a fraction of full power and a heavy robot can sit below its own breakaway friction there.
         * 0 disables the escape.
         */
        public static double antiStallPower = 0.25;
        /** Distance stops changing for this long inside the braking zone and we call it a stall. */
        public static int stallTimeoutMs = 400;
        /**
         * Use the kinematic brake-point law instead of the distance-keyed exponential curve. The
         * curve prescribes the same power at the same distance whatever the speed, so it can only be
         * right at one approach speed; this solves for where the brake must start.
         */
        public static boolean useKinematicLaw = true;
        /**
         * Deceleration the drivetrain actually achieves braking, cm/s². The one number the stopping
         * distance is solved from — measure it (see class javadoc) rather than guessing.
         */
        public static double brakingDecelCmPerSec2 = 150;
        /** Speed at full power, cm/s. Converts a target speed back into a motor power. */
        public static double topSpeedCmPerSec = 150;
        /** Dead time between commanding the brake and the robot actually slowing. */
        public static double reactionSeconds = 0.12;
        /**
         * How hard a fresh reading pulls the dead-reckoned position back, 0-1. Low: the model coasts
         * on its own physics through the brake and the lagged sensor only trims drift. 1 would hand
         * control back to the delayed reading, which is the problem this exists to avoid.
         */
        public static double sensorTrust = 0.15;
        /** Readings below this are sensor noise, not "on top of the wall." */
        public static double minValidCm = 2;
        /** Readings at or beyond this are out of the sensor's useful range — treated as no obstacle. */
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

    /** Poll the sensor. Call once per loop, before {@link #clampApproachPower}. */
    public void read() {
        double d = reader.getDistanceCm();
        if (Double.isNaN(d) || d < Tuning.minValidCm) {
            // A bad single sample keeps the last good distance rather than snapping to 0, so one
            // flaky read can't slam the brakes on for a loop.
            return;
        }

        // Only differentiate when the value actually moved. A ping/wait/read sensor republishes the
        // same number every loop between pings, so differentiating per-loop would read as "velocity
        // zero" most loops and then spike on the loop the sample lands.
        if (!Double.isNaN(lastSampleCm) && d != lastSampleCm) {
            double dtSec = (System.nanoTime() - lastSampleNanos) / 1e9;
            if (dtSec > 0) {
                double instantaneous = (lastSampleCm - d) / dtSec;
                closingVelocityCmPerSec += Tuning.velocityFilter * (instantaneous - closingVelocityCmPerSec);
            }
        }
        boolean freshSample = Double.isNaN(lastSampleCm) || d != lastSampleCm;
        if (freshSample) {
            lastSampleCm = d;
            lastSampleNanos = System.nanoTime();
        } else if (isStalled()) {
            // The estimate is only refreshed by a *changed* reading, so a robot that has stopped
            // would otherwise keep the closing speed it had when it was still moving — inflating the
            // prediction and hiding the stall from the escape below.
            closingVelocityCmPerSec = 0;
        }

        lastDistanceCm = d;
        updateModel(freshSample);
    }

    /**
     * Dead-reckons distance and speed forward every loop. While braking, speed follows
     * {@link Tuning#brakingDecelCmPerSec2} rather than the sensor: the differentiated reading lags
     * by more than the whole stop takes, so during the one manoeuvre that matters it describes a
     * speed the robot no longer has. Physics propagates at loop rate; the sensor only trims drift.
     */
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
            // The sample describes where the robot was one fixed pipeline delay ago, so age it
            // forward on the model's own speed before comparing — otherwise every correction would
            // drag the estimate backwards by exactly the lag we are trying to cancel.
            double impliedNow = lastSampleCm - modelVelocityCmPerSec * Tuning.sensorLatencySeconds;
            modelDistanceCm += Tuning.sensorTrust * (impliedNow - modelDistanceCm);
        }
    }

    /** Dead-reckoned distance to the obstacle — the value the kinematic law actually brakes on. */
    public double getModelDistanceCm() {
        return modelDistanceCm;
    }

    /** Dead-reckoned closing speed. Follows the decel model through a brake, not the lagging sensor. */
    public double getModelVelocityCmPerSec() {
        return modelVelocityCmPerSec;
    }

    /** Distance needed to stop from the current modelled speed, reaction dead time included. */
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

    /** Raw latest distance, in cm — as the sensor reported it, staleness included. */
    public double getDistanceCm() {
        return lastDistanceCm;
    }

    /** Age of the newest reading: the driver's fixed ping/wait plus however long it has sat unchanged. */
    private double sampleAgeSec() {
        if (lastSampleNanos == 0) {
            return Tuning.sensorLatencySeconds;
        }
        return Tuning.sensorLatencySeconds + (System.nanoTime() - lastSampleNanos) / 1e9;
    }

    /**
     * Best estimate of where the robot is <em>now</em>, correcting the reading for its own age. The
     * raw reading always lags reality while closing, which is why braking on it undershoots the
     * target by roughly one sample period's travel.
     */
    public double getCurrentDistanceCm() {
        return Math.max(0, lastDistanceCm - closingVelocityCmPerSec * sampleAgeSec());
    }

    /** Estimated closing speed in cm/s. Positive means approaching the obstacle. */
    public double getClosingVelocityCmPerSec() {
        return closingVelocityCmPerSec;
    }

    /** Where the robot will be by the time a power change issued now has taken effect, in cm. */
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

    /** Distance-keyed reference used by whichever law is active. */
    private double controlDistanceCm() {
        return Tuning.useKinematicLaw ? modelDistanceCm : getPredictedDistanceCm();
    }

    /** True once the stop distance has been reached and {@link Tuning#latchOnStop} is holding it there. */
    public boolean isStopped() {
        return stopped;
    }

    /** Release the stop latch so the robot may approach again. */
    public void resetStop() {
        stopped = false;
        reverseBraking = false;
        brakeEngaged = false;
        modelValid = false;
    }

    /**
     * The power ceiling the curve imposes right now, ignoring what the driver asked for. Telemetry
     * only — unlike {@link #clampApproachPower} this touches neither the stop latch nor the reverse
     * pulse timer, so reading it can't perturb what the robot does.
     */
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

    /**
     * Clamps a requested power toward this sensor's facing direction (positive = approaching).
     * Negative power (driving away) passes through unchanged.
     */
    public double clampApproachPower(double requestedPower) {
        if (!Tuning.enabled || requestedPower <= 0 || lastDistanceCm >= Tuning.maxValidCm) {
            return requestedPower;
        }

        double control = controlDistanceCm();

        if (control <= Tuning.stopDistanceCm) {
            stopped = Tuning.latchOnStop;
            brakeEngaged = false;
            double pulse = reverseBrakePulse();
            return pulse != 0 ? pulse : settleCorrection();
        }
        if (stopped) {
            brakeEngaged = false;
            double pulse = reverseBrakePulse();
            return pulse != 0 ? pulse : settleCorrection();
        }
        reverseBraking = false;

        if (Tuning.useKinematicLaw) {
            // Latching rather than re-deciding each loop: once braking, the modelled speed is falling,
            // so the brake point would test further and further away and the brake would chatter off.
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

    /**
     * Fastest the robot may be going at this distance and still stop on target, converted to a power
     * via {@link Tuning#topSpeedCmPerSec}. v = sqrt(2·a·d) is the same kinematics as the brake point,
     * so the taper and the trigger agree by construction instead of being tuned against each other.
     */
    private static double kinematicCeiling(double controlDistanceCm) {
        double remaining = controlDistanceCm - Tuning.stopDistanceCm;
        if (remaining <= 0 || Tuning.topSpeedCmPerSec <= 0) {
            return 0;
        }
        double targetSpeed = Math.sqrt(2 * Math.max(0, Tuning.brakingDecelCmPerSec2) * remaining);
        double ceiling = targetSpeed / Tuning.topSpeedCmPerSec;
        return Math.max(Tuning.minCreepPower, Math.min(1.0, ceiling));
    }

    /**
     * True only once the robot has actually come to rest: the reading has stopped changing and the
     * model agrees there is no speed left.
     */
    private boolean isAtRest() {
        return isStalled() && Math.abs(modelVelocityCmPerSec) < REST_SPEED_CM_PER_SEC;
    }

    /**
     * Closed-loop trim once the robot has stopped: creep whichever way lands on stopDistanceCm.
     *
     * Gated on {@link #isAtRest()} because it acts on the raw reading. That reading still shows the
     * pre-brake distance for one sensor delay after the brake starts, so running this while still
     * moving would command *forward* power exactly when the brake is due and power the robot through
     * its own stop — an overshoot on every run, regardless of how well the brake itself is tuned.
     * Once at rest the reading is no longer stale and driving to it is what makes the final position
     * independent of how good the coast estimate was.
     */
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

    /** True while the settle trim is creeping toward the target rather than holding still. */
    public boolean isSettling() {
        return stopped && settleCorrection() != 0;
    }

    /**
     * Extra power to break a stall short of the stop distance. The curve floors at minCreepPower,
     * which is not necessarily enough to move the robot; without this the approach can die a few cm
     * out and the latch never fires.
     */
    private double stallEscapePower() {
        if (Tuning.antiStallPower <= 0 || !isAtRest()) {
            return 0;
        }
        return Math.min(1.0, Tuning.antiStallPower);
    }

    /** True while the anti-stall escape is overriding the curve's power floor. */
    public boolean isStallEscaping() {
        return stallEscapePower() > 0 && !stopped && lastDistanceCm > Tuning.stopDistanceCm;
    }

    /** Callers must have already established stopDistanceCm < predicted < fullPowerAboveCm. */
    private static double curveCeiling(double predicted) {
        double t = (predicted - Tuning.stopDistanceCm) / (Tuning.fullPowerAboveCm - Tuning.stopDistanceCm);
        double ceiling = Tuning.minCreepPower + (Tuning.powerAtThreshold - Tuning.minCreepPower) * decayShape(t);
        return Math.max(Tuning.minCreepPower, Math.min(1.0, ceiling));
    }

    /**
     * Reverse power for this loop, or 0 to coast. The pulse is time-boxed rather than run until the
     * velocity estimate reads zero: that estimate only refreshes on a fresh sensor sample, so waiting
     * on it would keep driving backwards through every loop in between.
     */
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

    /** True while a reverse-brake pulse is actively driving the robot backwards. */
    public boolean isReverseBraking() {
        return reverseBraking
                && Tuning.reverseBrakePower > 0
                && (System.nanoTime() - reverseBrakeStartNanos) / 1e6 < Tuning.reverseBrakeMaxMs;
    }

    /** Maps t (1 at the braking threshold, 0 at the stop distance) onto the same 1..0 range. */
    private static double decayShape(double t) {
        double k = Tuning.decelRate;
        if (Math.abs(k) < 1e-6) {
            return t;
        }
        return (Math.exp(k * t) - 1) / (Math.exp(k) - 1);
    }
}
