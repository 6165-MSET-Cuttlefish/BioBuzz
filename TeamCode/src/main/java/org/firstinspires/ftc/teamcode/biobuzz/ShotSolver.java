// Portions (drag-corrected time of flight, phase-delay projection, look-ahead iteration) adapted from
// Mechanical-Advantage/RobotCode2026Public LaunchCalculator:
// Copyright (c) 2025-2026 Littleton Robotics, http://github.com/Mechanical-Advantage
// Use of that code is governed by an MIT-style license.

package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField.Cell;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField.CellOpening;

import java.util.ArrayDeque;

/**
 * Shooting on the move for any number of turrets, holding no hardware. Field frame: Pedro inches, heading and
 * omega counter-clockwise. Call {@link #solve} once per loop.
 */
public final class ShotSolver {

    public static final double GRAVITY_IN_PER_S2 = 386.09;
    private static final int ENTRY_SAMPLES = 60;

    /** Robot state this loop; set the commanded velocity to NaN when there is none. */
    public static final class Motion {
        public double xIn, yIn, headingRad;
        public double vxInPerS, vyInPerS, omegaRadPerS;
        public double commandVxInPerS = Double.NaN, commandVyInPerS = Double.NaN;

        public Motion() {}

        public Motion(double xIn, double yIn, double headingRad, double vxInPerS, double vyInPerS, double omegaRadPerS) {
            this.xIn = xIn;
            this.yIn = yIn;
            this.headingRad = headingRad;
            this.vxInPerS = vxInPerS;
            this.vyInPerS = vyInPerS;
            this.omegaRadPerS = omegaRadPerS;
        }
    }

    public enum Invalid { NONE, NOT_FINITE, NOT_CONVERGED, TOO_CLOSE, TOO_FAR, TURRET_RANGE, TURRET_RATE, TOO_FAST, EXIT_OFFSET, BEHIND_OPENING, NO_ENTRY, RISING, MISSES_OPENING, BAD_ZONE, KEEP_OUT }

    public static final class Result {
        public Cell cell;
        public ShooterGeometry.Element element;
        public double pivotXIn, pivotYIn;
        public double shooterVxInPerS, shooterVyInPerS;
        public double targetXIn, targetYIn, targetZIn;
        public double virtualXIn, virtualYIn;
        public double distanceIn;
        /** Rate of change of the virtual distance; negative while closing. */
        public double distanceRateInPerS;
        public int iterations;
        public double tofS, effTofS;
        public double turretDeg;
        public double analyticRateDegPerS, numericalRateDegPerS, rateDegPerS;
        public double rpm, hoodPosition = 0.5;
        /** How far the measured flywheel is below {@link #rpm}; the hood is read from the table's droop points for it. */
        public double droopRpm;
        double hoodDistanceIn;
        public double entryUIn = Double.NaN, entryWIn = Double.NaN, entryVzInPerS = Double.NaN;
        public Invalid invalid = Invalid.NONE;

        public boolean isValid() {
            return invalid == Invalid.NONE;
        }

        void invalidate(Invalid reason) {
            if (invalid == Invalid.NONE) invalid = reason;
        }
    }

    private static final class RateFilter {
        private final ArrayDeque<double[]> history = new ArrayDeque<>();
        private Cell cell;

        double update(Cell cell, double nowS, double angleDeg, double windowS) {
            if (cell != this.cell) {
                history.clear();
                this.cell = cell;
            }
            if (!history.isEmpty()) {
                double last = history.peekLast()[1];
                angleDeg = last + wrapDeg(angleDeg - last);
            }
            history.addLast(new double[] {nowS, angleDeg});
            while (history.size() > 2 && nowS - history.peekFirst()[0] > windowS) history.removeFirst();
            double[] oldest = history.peekFirst();
            double dt = nowS - oldest[0];
            return dt > 0 ? (angleDeg - oldest[1]) / dt : 0;
        }
    }

    private final ShooterGeometry[] shooters;
    private final RateFilter[] filters;
    private final Result[] results;
    private final ShotTuning.ShooterTuning[] tunings;
    private final double[] measuredRpm;
    private double filteredVxInPerS, filteredVyInPerS, filteredOmegaRadPerS;
    private double lastSolveS = Double.NaN;
    private double predictedXIn, predictedYIn, predictedHeadingRad, predictedVxInPerS, predictedVyInPerS, predictedOmegaRadPerS;

    public ShotSolver(ShooterGeometry... shooters) {
        this.shooters = shooters.clone();
        filters = new RateFilter[shooters.length];
        results = new Result[shooters.length];
        tunings = new ShotTuning.ShooterTuning[shooters.length];
        measuredRpm = new double[shooters.length];
        java.util.Arrays.fill(measuredRpm, Double.NaN);
        for (int i = 0; i < shooters.length; i++) {
            filters[i] = new RateFilter();
            results[i] = new Result();
        }
    }

    public void setGeometry(int shooter, ShooterGeometry geometry) {
        shooters[shooter] = geometry;
    }

    /** Trims, leads and the turret rate limit for a shooter; none until set. */
    public void setTuning(int shooter, ShotTuning.ShooterTuning tuning) {
        tunings[shooter] = tuning;
    }

    /** The flywheel's measured speed, for hood droop compensation; NaN (the default) for none. */
    public void setMeasuredRpm(int shooter, double rpm) {
        measuredRpm[shooter] = rpm;
    }

    public Result result(int shooter) {
        return results[shooter];
    }

    /** Solves every shooter, shooter i at {@code cells[i]} of this alliance, and returns the cached results. */
    public Result[] solve(Motion motion, Cell[] cells, boolean blue, double fieldWidthIn, double nowS) {
        predict(motion, nowS);
        boolean tooFast = Math.hypot(predictedVxInPerS, predictedVyInPerS) > ShotTuning.maxShotSpeedInPerS;
        for (int i = 0; i < shooters.length; i++) {
            CellOpening opening = BioBuzzField.cellOpening(cells[i], blue, fieldWidthIn);
            Result r = new Result();
            r.cell = cells[i];
            ShotTuning.ShooterTuning t = tunings[i];
            solveOne(shooters[i], t, opening, r, results[i].turretDeg, fieldWidthIn);
            if (t != null) {
                r.turretDeg += t.angleTrimDeg;
                r.rpm += t.rpmTrim;
            }
            if (Double.isFinite(measuredRpm[i])) r.droopRpm = Math.max(0, r.rpm - measuredRpm[i]);
            r.hoodPosition = lookupHood(shooters[i].element.tuning().table, r.hoodDistanceIn, r.droopRpm) + (t == null ? 0 : t.hoodTrim);
            r.numericalRateDegPerS = filters[i].update(cells[i], nowS, r.turretDeg, ShotTuning.numericalWindowS);
            r.rateDegPerS = ShotTuning.rateMethod == ShotTuning.RateMethod.NUMERICAL ? r.numericalRateDegPerS : r.analyticRateDegPerS;
            if (tooFast) r.invalidate(Invalid.TOO_FAST);
            if (t != null && Math.abs(r.rateDegPerS) > t.maxTurretRateDegPerS) r.invalidate(Invalid.TURRET_RATE);
            if (!Double.isFinite(r.turretDeg) || !Double.isFinite(r.rateDegPerS) || !Double.isFinite(r.rpm) || !Double.isFinite(r.hoodPosition)) {
                // Hold the last commands: a NaN must never reach a servo or the flywheel loop.
                r.invalid = Invalid.NOT_FINITE;
                r.turretDeg = results[i].turretDeg;
                r.rpm = results[i].rpm;
                r.hoodPosition = results[i].hoodPosition;
                r.rateDegPerS = 0;
            }
            results[i] = r;
        }
        applyKeepOuts();
        return results.clone();
    }

    private void predict(Motion m, double nowS) {
        double dtS = nowS - lastSolveS;
        lastSolveS = nowS;
        double keep = ShotTuning.velocityFilterS > 0 && dtS > 0 && dtS < 1 ? Math.exp(-dtS / ShotTuning.velocityFilterS) : 0;
        filteredVxInPerS = m.vxInPerS + keep * (filteredVxInPerS - m.vxInPerS);
        filteredVyInPerS = m.vyInPerS + keep * (filteredVyInPerS - m.vyInPerS);
        filteredOmegaRadPerS = m.omegaRadPerS + keep * (filteredOmegaRadPerS - m.omegaRadPerS);
        double vx = filteredVxInPerS;
        double vy = filteredVyInPerS;
        double omega = filteredOmegaRadPerS;
        if (Double.isFinite(m.commandVxInPerS) && Double.isFinite(m.commandVyInPerS)) {
            vx += ShotTuning.alpha * (m.commandVxInPerS - vx);
            vy += ShotTuning.alpha * (m.commandVyInPerS - vy);
        }
        if (!ShotTuning.sotmEnabled) {
            vx = 0;
            vy = 0;
            omega = 0;
        }
        double dt = ShotTuning.phaseDelayS;
        double h = m.headingRad;
        double forward = vx * Math.cos(h) + vy * Math.sin(h);
        double left = -vx * Math.sin(h) + vy * Math.cos(h);
        double dTheta = omega * dt;
        double s;
        double c;
        if (Math.abs(dTheta) < 1e-9) {
            s = 1;
            c = 0;
        } else {
            s = Math.sin(dTheta) / dTheta;
            c = (1 - Math.cos(dTheta)) / dTheta;
        }
        double dForward = (forward * s - left * c) * dt;
        double dLeft = (forward * c + left * s) * dt;
        predictedXIn = m.xIn + dForward * Math.cos(h) - dLeft * Math.sin(h);
        predictedYIn = m.yIn + dForward * Math.sin(h) + dLeft * Math.cos(h);
        predictedHeadingRad = h + dTheta;
        predictedVxInPerS = vx * Math.cos(dTheta) - vy * Math.sin(dTheta);
        predictedVyInPerS = vx * Math.sin(dTheta) + vy * Math.cos(dTheta);
        predictedOmegaRadPerS = omega;
    }

    private void solveOne(ShooterGeometry g, ShotTuning.ShooterTuning t, CellOpening opening, Result r, double previousTurretDeg, double fieldWidthIn) {
        ShotTuning.ElementTuning e = g.element.tuning();
        r.element = g.element;
        double h = predictedHeadingRad;
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double offX = g.pivotForwardIn * cos - g.pivotLeftIn * sin;
        double offY = g.pivotForwardIn * sin + g.pivotLeftIn * cos;
        double px = predictedXIn + offX;
        double py = predictedYIn + offY;
        double ux = predictedVxInPerS - predictedOmegaRadPerS * offY;
        double uy = predictedVyInPerS + predictedOmegaRadPerS * offX;
        r.pivotXIn = px;
        r.pivotYIn = py;
        r.shooterVxInPerS = ux;
        r.shooterVyInPerS = uy;

        double[] aim = opening.aimPoint(ShotTuning.aimDepthIn);
        r.targetXIn = aim[0];
        r.targetYIn = aim[1];
        r.targetZIn = aim[2];

        double distance = Math.hypot(aim[0] - px, aim[1] - py);
        boolean converged = false;
        double vtx = aim[0];
        double vty = aim[1];
        for (int k = 0; k < ShotTuning.maxIterations; k++) {
            r.iterations = k + 1;
            r.tofS = lookup(e.table, distance, Column.TOF);
            r.effTofS = dragTime(r.tofS, e.kDragPerS) + g.transferTimeS;
            vtx = aim[0] - ux * r.effTofS;
            vty = aim[1] - uy * r.effTofS;
            double next = Math.hypot(vtx - px, vty - py);
            boolean done = Math.abs(next - distance) < ShotTuning.convergenceIn;
            distance = next;
            if (done) {
                converged = true;
                break;
            }
        }
        r.virtualXIn = vtx;
        r.virtualYIn = vty;
        r.distanceIn = distance;
        if (!Double.isFinite(distance) || !Double.isFinite(r.tofS)) {
            r.invalidate(Invalid.NOT_FINITE);
            r.turretDeg = previousTurretDeg;
            r.rpm = Double.NaN;
            r.hoodDistanceIn = Double.NaN;
            return;
        }
        if (!converged) r.invalidate(Invalid.NOT_CONVERGED);

        double[] range = tableRange(e.table);
        if (distance < Math.max(range[0], e.minDistanceIn)) r.invalidate(Invalid.TOO_CLOSE);
        if (distance > Math.min(range[1], e.maxDistanceIn)) r.invalidate(Invalid.TOO_FAR);

        double rx = vtx - px;
        double ry = vty - py;
        r.distanceRateInPerS = -(ux * rx + uy * ry) / distance;
        r.rpm = lead(e.table, distance, Column.RPM, r.distanceRateInPerS, t == null ? 0 : t.flywheelLeadS);
        r.hoodDistanceIn = distance + r.distanceRateInPerS * (t == null ? 0 : t.hoodLeadS);
        double bearing = Math.atan2(ry, rx);
        double ratio = g.exitLeftIn / distance;
        if (Math.abs(ratio) >= 1) {
            r.invalidate(Invalid.EXIT_OFFSET);
            ratio = Math.copySign(1, ratio);
        }
        bearing -= Math.asin(ratio);
        double raw = wrapDeg(g.sign * (Math.toDegrees(bearing - h) - g.zeroBearingDeg));
        r.turretDeg = inRange(raw, previousTurretDeg, g);
        if (Double.isNaN(r.turretDeg)) {
            r.invalidate(Invalid.TURRET_RANGE);
            r.turretDeg = clampToRange(raw, g);
        }

        double fieldRate = fieldBearingRateRadPerS(ux, uy, rx, ry);
        r.analyticRateDegPerS = g.sign * Math.toDegrees(fieldRate - predictedOmegaRadPerS);

        checkEntry(g, e, opening, r, px, py, ux, uy, aim);

        if (BioBuzzField.underHive(predictedXIn, predictedYIn, ShotTuning.hiveBadZoneMarginIn, fieldWidthIn)) {
            r.invalidate(Invalid.BAD_ZONE);
        }
    }

    /**
     * Flies the ball under linear drag and checks it crosses the opening's plane inward, inside the outline less the
     * ball's radius and the rim clearance, and (if required) falling.
     */
    private static void checkEntry(ShooterGeometry g, ShotTuning.ElementTuning e, CellOpening opening, Result r,
                                   double px, double py, double ux, double uy, double[] aim) {
        double k = e.kDragPerS;
        double t = r.tofS;
        double reach = dragTime(t, k);
        double ex = px + ux * g.transferTimeS;
        double ey = py + uy * g.transferTimeS;
        double ez = g.exitHeightIn;
        double dirX = (r.virtualXIn - px) / r.distanceIn;
        double dirY = (r.virtualYIn - py) / r.distanceIn;
        double horizontalSpeed = r.distanceIn / reach;
        double v0x = horizontalSpeed * dirX + ux;
        double v0y = horizontalSpeed * dirY + uy;
        double g0 = GRAVITY_IN_PER_S2;
        double v0z = k > 0
                ? (aim[2] - ez + g0 / k * t) / reach - g0 / k
                : (aim[2] - ez + 0.5 * g0 * t * t) / t;

        double[] o = opening.point(0, 0, 0);
        double before = planeSide(opening, o, ex, ey, ez);
        if (!(before > 0)) {
            r.invalidate(Invalid.BEHIND_OPENING);
            return;
        }
        double lo = 0;
        double hi = Double.NaN;
        for (int i = 1; i <= ENTRY_SAMPLES; i++) {
            double ti = t * i / ENTRY_SAMPLES;
            double[] p = flight(ex, ey, ez, v0x, v0y, v0z, k, ti);
            if (planeSide(opening, o, p[0], p[1], p[2]) <= 0) {
                hi = ti;
                break;
            }
            lo = ti;
        }
        if (Double.isNaN(hi)) {
            r.invalidate(Invalid.NO_ENTRY);
            return;
        }
        for (int i = 0; i < 40; i++) {
            double mid = (lo + hi) / 2;
            double[] p = flight(ex, ey, ez, v0x, v0y, v0z, k, mid);
            if (planeSide(opening, o, p[0], p[1], p[2]) > 0) lo = mid;
            else hi = mid;
        }
        double[] p = flight(ex, ey, ez, v0x, v0y, v0z, k, hi);
        double decay = k > 0 ? Math.exp(-k * hi) : 1;
        double vx = v0x * decay;
        double vy = v0y * decay;
        double vz = k > 0 ? (v0z + g0 / k) * decay - g0 / k : v0z - g0 * hi;
        double dx = p[0] - o[0];
        double dy = p[1] - o[1];
        double dz = p[2] - o[2];
        r.entryUIn = dx * opening.u[0] + dy * opening.u[1] + dz * opening.u[2];
        r.entryWIn = dx * opening.w[0] + dy * opening.w[1] + dz * opening.w[2];
        r.entryVzInPerS = vz;
        if (ShotTuning.requireDescendingEntry && vz >= 0) r.invalidate(Invalid.RISING);
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double incidence = -(vx * opening.n[0] + vy * opening.n[1] + vz * opening.n[2]) / speed;
        // A sphere crossing at a slant sweeps r / cos(incidence) along its path.
        double margin = g.element.diameterIn / 2 / Math.max(incidence, 0.33) + ShotTuning.rimClearanceIn;
        if (!CellOpening.inside(r.entryUIn, r.entryWIn, margin)) r.invalidate(Invalid.MISSES_OPENING);
    }

    /** d/dt of the bearing atan2(ry, rx) from a point moving at (vx, vy) to a fixed target r away: r' = -v. */
    public static double fieldBearingRateRadPerS(double vx, double vy, double rx, double ry) {
        return (vx * ry - vy * rx) / (rx * rx + ry * ry);
    }

    /** Ball position {@code t} seconds after launch under linear drag {@code k}. */
    public static double[] flight(double x, double y, double z, double vx, double vy, double vz, double k, double t) {
        double reach = dragTime(t, k);
        double g0 = GRAVITY_IN_PER_S2;
        double fz = k > 0 ? z + (vz + g0 / k) * reach - g0 / k * t : z + vz * t - 0.5 * g0 * t * t;
        return new double[] {x + vx * reach, y + vy * reach, fz};
    }

    private static double planeSide(CellOpening opening, double[] o, double x, double y, double z) {
        return (x - o[0]) * opening.n[0] + (y - o[1]) * opening.n[1] + (z - o[2]) * opening.n[2];
    }

    /** How far a unit initial velocity carries in {@code t} under linear drag {@code k}: (1 - e^-kt) / k. */
    public static double dragTime(double t, double k) {
        return k > 0 ? (1 - Math.exp(-k * t)) / k : t;
    }

    private void applyKeepOuts() {
        if (results.length < 2) return;
        Result pollen = null;
        Result nectar = null;
        for (Result r : results) {
            if (r.element == ShooterGeometry.Element.POLLEN) pollen = r;
            if (r.element == ShooterGeometry.Element.NECTAR) nectar = r;
        }
        if (pollen == null || nectar == null) return;
        for (ShotTuning.KeepOut k : ShotTuning.keepOuts) {
            if (k.pollenMinDeg == 0 && k.pollenMaxDeg == 0 && k.nectarMinDeg == 0 && k.nectarMaxDeg == 0) continue;
            boolean pollenIn = pollen.turretDeg >= k.pollenMinDeg && pollen.turretDeg <= k.pollenMaxDeg;
            boolean nectarIn = nectar.turretDeg >= k.nectarMinDeg && nectar.turretDeg <= k.nectarMaxDeg;
            if (pollenIn && nectarIn) nectar.invalidate(Invalid.KEEP_OUT);
        }
    }

    /** The equivalent of {@code angleDeg} (mod 360) inside the turret's limits nearest the last command, or NaN. */
    private static double inRange(double angleDeg, double previousDeg, ShooterGeometry g) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double a = angleDeg + 360 * k;
            if (a < g.minAngleDeg || a > g.maxAngleDeg) continue;
            if (Double.isNaN(best) || Math.abs(a - previousDeg) < Math.abs(best - previousDeg)) best = a;
        }
        return best;
    }

    private static double clampToRange(double angleDeg, ShooterGeometry g) {
        double toMin = Math.abs(wrapDeg(angleDeg - g.minAngleDeg));
        double toMax = Math.abs(wrapDeg(angleDeg - g.maxAngleDeg));
        return toMin < toMax ? g.minAngleDeg : g.maxAngleDeg;
    }

    /** The table's value where the distance will be {@code leadS} from now, so a lagging flywheel or hood arrives on time. */
    private static double lead(ShotTuning.ShotPoint[] table, double distanceIn, Column c, double rateInPerS, double leadS) {
        return lookup(table, distanceIn + rateInPerS * leadS, c);
    }

    private enum Column { RPM, HOOD, TOF }

    private static double value(ShotTuning.ShotPoint p, Column c) {
        switch (c) {
            case RPM: return p.rpm;
            case HOOD: return p.hoodPosition;
            default: return p.tofS;
        }
    }

    /**
     * Linear interpolation between the distances with points (distance > 0), clamped to the ends. Each distance's
     * value comes from its highest-rpm point; its hood, with the flywheel {@code droopRpm} below that, from
     * interpolating by rpm among the points at that distance.
     */
    static double lookup(ShotTuning.ShotPoint[] table, double distanceIn, Column c, double droopRpm) {
        double lo = Double.NEGATIVE_INFINITY;
        double hi = Double.POSITIVE_INFINITY;
        for (ShotTuning.ShotPoint p : table) {
            if (p.distanceIn <= 0) continue;
            if (p.distanceIn <= distanceIn && p.distanceIn > lo) lo = p.distanceIn;
            if (p.distanceIn >= distanceIn && p.distanceIn < hi) hi = p.distanceIn;
        }
        boolean none = Double.isNaN(distanceIn) || (Double.isInfinite(lo) && Double.isInfinite(hi));
        if (none) return Double.NaN;
        if (Double.isInfinite(lo)) return atDistance(table, hi, c, droopRpm);
        if (Double.isInfinite(hi) || hi == lo) return atDistance(table, lo, c, droopRpm);
        double a = atDistance(table, lo, c, droopRpm);
        double b = atDistance(table, hi, c, droopRpm);
        return a + (distanceIn - lo) / (hi - lo) * (b - a);
    }

    static double lookup(ShotTuning.ShotPoint[] table, double distanceIn, Column c) {
        return lookup(table, distanceIn, c, 0);
    }

    private static double atDistance(ShotTuning.ShotPoint[] table, double distanceIn, Column c, double droopRpm) {
        ShotTuning.ShotPoint primary = null;
        for (ShotTuning.ShotPoint p : table) {
            if (p.distanceIn == distanceIn && (primary == null || p.rpm > primary.rpm)) primary = p;
        }
        if (c != Column.HOOD || !(droopRpm > 0)) return value(primary, c);
        double rpm = primary.rpm - droopRpm;
        ShotTuning.ShotPoint below = null;
        ShotTuning.ShotPoint above = primary;
        for (ShotTuning.ShotPoint p : table) {
            if (p.distanceIn != distanceIn) continue;
            if (p.rpm <= rpm && (below == null || p.rpm > below.rpm)) below = p;
            if (p.rpm >= rpm && p.rpm < above.rpm) above = p;
        }
        if (below == null || below == above) return above.hoodPosition;
        return below.hoodPosition + (rpm - below.rpm) / (above.rpm - below.rpm) * (above.hoodPosition - below.hoodPosition);
    }

    public static double lookupRpm(ShotTuning.ShotPoint[] table, double distanceIn) {
        return lookup(table, distanceIn, Column.RPM);
    }

    public static double lookupHood(ShotTuning.ShotPoint[] table, double distanceIn) {
        return lookup(table, distanceIn, Column.HOOD);
    }

    public static double lookupHood(ShotTuning.ShotPoint[] table, double distanceIn, double droopRpm) {
        return lookup(table, distanceIn, Column.HOOD, droopRpm);
    }

    public static double lookupTof(ShotTuning.ShotPoint[] table, double distanceIn) {
        return lookup(table, distanceIn, Column.TOF);
    }

    private static double[] tableRange(ShotTuning.ShotPoint[] table) {
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (ShotTuning.ShotPoint p : table) {
            if (p.distanceIn <= 0) continue;
            min = Math.min(min, p.distanceIn);
            max = Math.max(max, p.distanceIn);
        }
        return new double[] {min, max};
    }

    static double wrapDeg(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
}
