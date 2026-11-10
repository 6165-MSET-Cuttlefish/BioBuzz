package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField.Cell;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField.CellOpening;
import org.firstinspires.ftc.teamcode.biobuzz.ShooterGeometry.Element;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver.Invalid;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver.Motion;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver.Result;
import org.firstinspires.ftc.teamcode.biobuzz.ShotTuning.ShotPoint;
import org.junit.Before;
import org.junit.Test;

public class ShotSolverTest {

    private static final double W = 141.5;
    private static final double FACING_AUDIENCE = 1.5 * Math.PI;
    private static final Cell[] UP = {Cell.STARTS_UP, Cell.STARTS_UP};

    @Before
    public void setUp() {
        ShotTuning.sotmEnabled = true;
        ShotTuning.alpha = 0.2;
        ShotTuning.phaseDelayS = 0;
        ShotTuning.velocityFilterS = 0;
        ShotTuning.maxShotSpeedInPerS = 1000;
        ShotTuning.maxIterations = 50;
        ShotTuning.convergenceIn = 1e-6;
        ShotTuning.rateMethod = ShotTuning.RateMethod.ANALYTIC;
        ShotTuning.numericalWindowS = 0.1;
        ShotTuning.aimDepthIn = 3;
        ShotTuning.rimClearanceIn = 1;
        ShotTuning.requireDescendingEntry = true;
        ShotTuning.hiveBadZoneMarginIn = 6;
        ShotTuning.keepOuts = new ShotTuning.KeepOut[0];
        ShotTuning.pollen = new ShotTuning.ElementTuning(
                new ShotPoint(20, 2000, 0.40, 0.55), new ShotPoint(140, 3000, 0.60, 0.85), new ShotPoint());
        ShotTuning.nectar = new ShotTuning.ElementTuning(
                new ShotPoint(20, 2600, 0.30, 0.60), new ShotPoint(140, 3800, 0.70, 0.95));
    }

    private static ShooterGeometry shooter(Element element, double forwardIn, double leftIn) {
        return new ShooterGeometry(element.name(), element, forwardIn, leftIn, 0, 14,
                ShooterGeometry.BACK_ZERO_BEARING_DEG, true, -90, 90, 0.1);
    }

    private static ShotSolver solver(double leftA, double leftB) {
        return new ShotSolver(shooter(Element.POLLEN, 0, leftA), shooter(Element.NECTAR, 0, leftB));
    }

    private static double[] aim() {
        return BioBuzzField.cellOpening(Cell.STARTS_UP, false, W).aimPoint(ShotTuning.aimDepthIn);
    }

    private static double fieldBearingDeg(Result r, ShooterGeometry g, double headingRad) {
        return Math.toDegrees(headingRad) + g.zeroBearingDeg + r.turretDeg / g.sign;
    }

    private static double wrap(double deg) {
        return ShotSolver.wrapDeg(deg);
    }

    @Test
    public void stationaryAimsAtTheCellWithNoFeedforward() {
        ShotSolver s = solver(0, 0);
        Motion m = new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0);
        s.solve(m, UP, false, W, 0);
        Result[] rs = s.solve(m, UP, false, W, 0.01);
        double[] aim = aim();
        for (Result r : rs) {
            assertTrue(r.invalid.toString(), r.isValid());
            assertEquals(aim[0], r.virtualXIn, 1e-9);
            assertEquals(aim[1], r.virtualYIn, 1e-9);
            assertEquals(0, r.analyticRateDegPerS, 1e-9);
            assertEquals(0, r.numericalRateDegPerS, 1e-9);
            assertEquals(0, r.turretDeg, 1e-6);
        }
    }

    @Test
    public void pureRotationFeedforwardIsMinusOmega() {
        ShotSolver s = solver(0, 0);
        double omega = 1.3;
        Result[] rs = s.solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, omega), UP, false, W, 0);
        for (Result r : rs) assertEquals(-Math.toDegrees(omega), r.analyticRateDegPerS, 1e-9);
    }

    @Test
    public void bearingRateSign() {
        double rate = ShotSolver.fieldBearingRateRadPerS(10, 0, 0, 100);
        assertTrue(rate > 0);
        double dt = 1e-4;
        double numeric = (Math.atan2(100, -10 * dt) - Math.atan2(100, 0)) / dt;
        assertEquals(numeric, rate, 1e-6);
    }

    @Test
    public void analyticAndNumericalAgreeOverAStrafe() {
        ShotSolver s = solver(4, -4);
        double vx = 20;
        double t = 0;
        Result[] rs = null;
        for (int i = 0; i <= 100; i++) {
            t = i * 0.01;
            rs = s.solve(new Motion(40 + vx * t, 10, FACING_AUDIENCE, vx, 0, 0), UP, false, W, t);
        }
        for (Result r : rs) {
            assertTrue(Math.abs(r.analyticRateDegPerS) > 5);
            assertEquals(r.analyticRateDegPerS, r.numericalRateDegPerS, 0.05 * Math.abs(r.analyticRateDegPerS));
        }
    }

    @Test
    public void strafeTowardPlusXTurnsTheTurretCounterClockwise() {
        ShotSolver s = solver(0, 0);
        Result r = s.solve(new Motion(58, 10, FACING_AUDIENCE, 10, 0, 0), UP, false, W, 0)[0];
        assertTrue(r.analyticRateDegPerS > 0);
    }

    @Test
    public void offsetTurretsSeeOppositeSpinVelocities() {
        ShotSolver s = solver(6, -6);
        Result[] rs = s.solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 2), UP, false, W, 0);
        assertEquals(-rs[0].shooterVxInPerS, rs[1].shooterVxInPerS, 1e-9);
        assertEquals(-rs[0].shooterVyInPerS, rs[1].shooterVyInPerS, 1e-9);
        assertEquals(12, Math.abs(rs[0].shooterVyInPerS), 1e-9);
        assertNotEquals(rs[0].virtualYIn, rs[1].virtualYIn, 0.1);
    }

    @Test
    public void parallaxEachTurretPointsAtTheCellFromItsOwnPivot() {
        ShotSolver s = solver(6, -6);
        Result[] rs = s.solve(new Motion(58, 30, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0);
        double[] aim = aim();
        ShooterGeometry[] gs = {shooter(Element.POLLEN, 0, 6), shooter(Element.NECTAR, 0, -6)};
        assertTrue(Math.abs(rs[0].turretDeg - rs[1].turretDeg) > 5);
        for (int i = 0; i < 2; i++) {
            double expected = Math.toDegrees(Math.atan2(aim[1] - rs[i].pivotYIn, aim[0] - rs[i].pivotXIn));
            assertEquals(0, wrap(fieldBearingDeg(rs[i], gs[i], FACING_AUDIENCE) - expected), 1e-6);
        }
    }

    @Test
    public void exitOffsetPutsTheLaunchLineThroughTheTarget() {
        double exitLeft = 3;
        ShooterGeometry g = new ShooterGeometry("P", Element.POLLEN, 0, 0, exitLeft, 14,
                ShooterGeometry.BACK_ZERO_BEARING_DEG, true, -90, 90, 0.1);
        Result r = new ShotSolver(g).solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0), new Cell[] {Cell.STARTS_UP}, false, W, 0)[0];
        double theta = Math.toRadians(fieldBearingDeg(r, g, FACING_AUDIENCE));
        double ex = r.pivotXIn - exitLeft * Math.sin(theta);
        double ey = r.pivotYIn + exitLeft * Math.cos(theta);
        double[] aim = aim();
        double cross = Math.cos(theta) * (aim[1] - ey) - Math.sin(theta) * (aim[0] - ex);
        assertEquals(0, cross, 1e-6);
    }

    @Test
    public void eachElementUsesItsOwnTable() {
        ShotSolver s = solver(0, 0);
        Result[] rs = s.solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0);
        assertEquals(Element.POLLEN, rs[0].element);
        assertEquals(Element.NECTAR, rs[1].element);
        assertEquals(ShotSolver.lookupRpm(ShotTuning.pollen.table, rs[0].distanceIn), rs[0].rpm, 1e-9);
        assertEquals(ShotSolver.lookupRpm(ShotTuning.nectar.table, rs[1].distanceIn), rs[1].rpm, 1e-9);
        assertEquals(ShotSolver.lookupHood(ShotTuning.nectar.table, rs[1].distanceIn), rs[1].hoodPosition, 1e-9);
        assertNotEquals(rs[0].rpm, rs[1].rpm, 1);
        assertNotEquals(rs[0].tofS, rs[1].tofS, 1e-3);
        assertEquals(Element.POLLEN, ShooterGeometry.pollen().element);
        assertEquals(Element.NECTAR, ShooterGeometry.nectar().element);
    }

    @Test
    public void tableLookupInterpolatesAndClamps() {
        ShotPoint[] t = ShotTuning.pollen.table;
        assertEquals(2500, ShotSolver.lookupRpm(t, 80), 1e-9);
        assertEquals(2000, ShotSolver.lookupRpm(t, 5), 1e-9);
        assertEquals(3000, ShotSolver.lookupRpm(t, 500), 1e-9);
    }

    @Test
    public void risingShotIsInvalid() {
        ShotTuning.pollen = new ShotTuning.ElementTuning(new ShotPoint(20, 2000, 0.4, 0.2), new ShotPoint(140, 3000, 0.6, 0.3));
        Result r = solver(0, 0).solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0)[0];
        assertEquals(Invalid.RISING, r.invalid);
        assertTrue(r.entryVzInPerS > 0);
    }

    @Test
    public void shotThatWouldHitTheRimIsInvalid() {
        ShotTuning.rimClearanceIn = 6;
        Result r = solver(0, 0).solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0)[0];
        assertEquals(Invalid.MISSES_OPENING, r.invalid);
    }

    @Test
    public void shotFromBehindTheOpeningIsInvalid() {
        ShotTuning.hiveBadZoneMarginIn = -100;
        Result r = solver(0, 0).solve(new Motion(58, 60, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0)[0];
        assertFalse(r.isValid());
    }

    @Test
    public void underTheHiveIsABadZone() {
        Result r = solver(0, 0).solve(new Motion(58, 80, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0)[0];
        assertFalse(r.isValid());
        assertTrue(BioBuzzField.underHive(58, 80, 0, W));
    }

    @Test
    public void allianceMirroring() {
        for (Cell cell : Cell.values()) {
            CellOpening red = BioBuzzField.cellOpening(cell, false, W);
            CellOpening blue = BioBuzzField.cellOpening(cell, true, W);
            double[] r = red.aimPoint(3);
            double[] b = blue.aimPoint(3);
            assertEquals(W - r[0], b[0], 1e-9);
            assertEquals(W - r[1], b[1], 1e-9);
            assertEquals(r[2], b[2], 1e-9);
            assertEquals(-red.n[1], blue.n[1], 1e-9);
            assertEquals(red.n[2], blue.n[2], 1e-9);
        }
        CellOpening redUp = BioBuzzField.cellOpening(Cell.STARTS_UP, false, W);
        assertTrue(redUp.bottomCentreYIn < W / 2);
        assertTrue(redUp.n[1] < 0);
        assertTrue(redUp.bottomCentreXIn < W / 2);
        assertEquals(Math.sin(Math.toRadians(30)), redUp.n[2], 1e-12);
    }

    @Test
    public void openingHeightsMatchTheManual() {
        CellOpening o = BioBuzzField.cellOpening(Cell.STARTS_UP, false, W);
        assertEquals(53.5, o.point(0, 0, 0)[2], 1e-9);
        assertEquals(65.6, o.point(0, 14, 0)[2], 0.05);
    }

    @Test
    public void movingTowardShortensAndAwayLengthens() {
        ShotSolver s = solver(0, 0);
        double still = s.solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0)[0].distanceIn;
        double toward = new ShotSolver(shooter(Element.POLLEN, 0, 0))
                .solve(new Motion(58, 10, FACING_AUDIENCE, 0, 30, 0), new Cell[] {Cell.STARTS_UP}, false, W, 0)[0].distanceIn;
        double away = new ShotSolver(shooter(Element.POLLEN, 0, 0))
                .solve(new Motion(58, 10, FACING_AUDIENCE, 0, -30, 0), new Cell[] {Cell.STARTS_UP}, false, W, 0)[0].distanceIn;
        assertTrue(toward < still);
        assertTrue(away > still);
    }

    @Test
    public void sotmOffAimsAsIfStationary() {
        ShotTuning.sotmEnabled = false;
        Result r = solver(0, 0).solve(new Motion(58, 10, FACING_AUDIENCE, 30, 20, 1), UP, false, W, 0)[0];
        double[] aim = aim();
        assertEquals(aim[0], r.virtualXIn, 1e-9);
        assertEquals(aim[1], r.virtualYIn, 1e-9);
        assertEquals(0, r.analyticRateDegPerS, 1e-9);
    }

    @Test
    public void commandedVelocityIsBlendedIn() {
        ShotTuning.alpha = 0.5;
        Motion m = new Motion(58, 10, FACING_AUDIENCE, 10, 0, 0);
        m.commandVxInPerS = 30;
        m.commandVyInPerS = 0;
        Result r = solver(0, 0).solve(m, UP, false, W, 0)[0];
        assertEquals(20, r.shooterVxInPerS, 1e-9);
    }

    @Test
    public void phaseDelayFollowsTheArc() {
        ShotTuning.phaseDelayS = 0.5;
        double forward = 20;
        double omega = 1.5;
        double h0 = 0.3;
        Motion m = new Motion(50, 10, h0, forward * Math.cos(h0), forward * Math.sin(h0), omega);
        Result r = new ShotSolver(shooter(Element.POLLEN, 0, 0)).solve(m, new Cell[] {Cell.STARTS_UP}, false, W, 0)[0];
        double x = 50;
        double y = 10;
        double h = h0;
        int steps = 200000;
        double dt = 0.5 / steps;
        for (int i = 0; i < steps; i++) {
            x += forward * Math.cos(h + omega * dt / 2) * dt;
            y += forward * Math.sin(h + omega * dt / 2) * dt;
            h += omega * dt;
        }
        assertEquals(x, r.pivotXIn, 1e-6);
        assertEquals(y, r.pivotYIn, 1e-6);
    }

    @Test
    public void numericalFilterResetsOnCellChange() {
        ShotTuning.hiveBadZoneMarginIn = -100;
        ShotSolver s = solver(0, 0);
        Motion m = new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0);
        s.solve(m, UP, false, W, 0);
        Result r = s.solve(m, new Cell[] {Cell.STARTS_DOWN, Cell.STARTS_DOWN}, false, W, 0.01)[0];
        assertEquals(0, r.numericalRateDegPerS, 1e-9);
    }

    @Test
    public void closingRobotLeadsTheFlywheelDownTheTable() {
        ShotTuning.ShooterTuning t = new ShotTuning.ShooterTuning();
        t.flywheelLeadS = 0.2;
        ShotSolver s = new ShotSolver(shooter(Element.POLLEN, 0, 0));
        s.setTuning(0, t);
        Result r = s.solve(new Motion(58, 10, FACING_AUDIENCE, 0, 30, 0), new Cell[] {Cell.STARTS_UP}, false, W, 0)[0];
        assertEquals(-30, r.distanceRateInPerS, 1e-6);
        assertEquals(ShotSolver.lookupRpm(ShotTuning.pollen.table, r.distanceIn - 6), r.rpm, 1e-6);
        assertEquals(ShotSolver.lookupHood(ShotTuning.pollen.table, r.distanceIn), r.hoodPosition, 1e-9);
    }

    @Test
    public void trimsShiftTheCommands() {
        ShotTuning.ShooterTuning t = new ShotTuning.ShooterTuning();
        t.rpmTrim = 50;
        t.hoodTrim = 0.02;
        t.angleTrimDeg = 1.5;
        Motion m = new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0);
        Result plain = new ShotSolver(shooter(Element.POLLEN, 0, 0)).solve(m, new Cell[] {Cell.STARTS_UP}, false, W, 0)[0];
        ShotSolver s = new ShotSolver(shooter(Element.POLLEN, 0, 0));
        s.setTuning(0, t);
        Result r = s.solve(m, new Cell[] {Cell.STARTS_UP}, false, W, 0)[0];
        assertEquals(plain.rpm + 50, r.rpm, 1e-9);
        assertEquals(plain.hoodPosition + 0.02, r.hoodPosition, 1e-9);
        assertEquals(plain.turretDeg + 1.5, r.turretDeg, 1e-9);
    }

    @Test
    public void tooFastAndTurretRateLimitsInvalidate() {
        ShotTuning.maxShotSpeedInPerS = 20;
        Result fast = solver(0, 0).solve(new Motion(58, 10, FACING_AUDIENCE, 30, 0, 0), UP, false, W, 0)[0];
        assertEquals(Invalid.TOO_FAST, fast.invalid);

        ShotTuning.maxShotSpeedInPerS = 1000;
        ShotTuning.ShooterTuning t = new ShotTuning.ShooterTuning();
        t.maxTurretRateDegPerS = 100;
        ShotSolver s = new ShotSolver(shooter(Element.POLLEN, 0, 0));
        s.setTuning(0, t);
        Result spin = s.solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 3), new Cell[] {Cell.STARTS_UP}, false, W, 0)[0];
        assertEquals(Invalid.TURRET_RATE, spin.invalid);
    }

    @Test
    public void velocityFilterSmoothsAStep() {
        ShotTuning.velocityFilterS = 0.1;
        ShotSolver s = new ShotSolver(shooter(Element.POLLEN, 0, 0));
        Cell[] one = {Cell.STARTS_UP};
        s.solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0), one, false, W, 0);
        Result r = s.solve(new Motion(58, 10, FACING_AUDIENCE, 40, 0, 0), one, false, W, 0.1)[0];
        assertEquals(40 * (1 - Math.exp(-1)), r.shooterVxInPerS, 1e-9);
    }

    @Test
    public void emptyTableHoldsTheLastCommandsAndIsInvalid() {
        ShotSolver s = solver(0, 0);
        Motion m = new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0);
        Result good = s.solve(m, UP, false, W, 0)[0];
        ShotTuning.pollen = new ShotTuning.ElementTuning(new ShotPoint());
        Result r = s.solve(m, UP, false, W, 0.01)[0];
        assertEquals(Invalid.NOT_FINITE, r.invalid);
        assertEquals(good.rpm, r.rpm, 1e-9);
        assertEquals(good.hoodPosition, r.hoodPosition, 1e-9);
        assertEquals(good.turretDeg, r.turretDeg, 1e-9);
    }

    @Test
    public void hoodFollowsTheDroopPointsAtEachDistance() {
        ShotPoint[] t = {
                new ShotPoint(40, 2800, 0.50, 0.6), new ShotPoint(40, 2700, 0.56, 0), new ShotPoint(40, 2600, 0.66, 0),
                new ShotPoint(80, 3200, 0.30, 0.8), new ShotPoint(80, 3100, 0.34, 0),
                new ShotPoint()};
        assertEquals(2800, ShotSolver.lookupRpm(t, 40), 1e-9);
        assertEquals(0.6, ShotSolver.lookupTof(t, 40), 1e-9);
        assertEquals(3000, ShotSolver.lookupRpm(t, 60), 1e-9);
        assertEquals(0.40, ShotSolver.lookupHood(t, 60), 1e-9);
        assertEquals(0.50, ShotSolver.lookupHood(t, 40, 0), 1e-9);
        assertEquals(0.53, ShotSolver.lookupHood(t, 40, 50), 1e-9);
        assertEquals(0.61, ShotSolver.lookupHood(t, 40, 150), 1e-9);
        assertEquals(0.66, ShotSolver.lookupHood(t, 40, 900), 1e-9);
        assertEquals(0.34, ShotSolver.lookupHood(t, 80, 900), 1e-9);
        assertEquals((0.56 + 0.34) / 2, ShotSolver.lookupHood(t, 60, 100), 1e-9);
    }

    @Test
    public void measuredFlywheelSpeedDroopsTheHood() {
        ShotTuning.pollen = new ShotTuning.ElementTuning(
                new ShotPoint(20, 2000, 0.40, 0.55), new ShotPoint(20, 1800, 0.50, 0),
                new ShotPoint(140, 3000, 0.60, 0.85), new ShotPoint(140, 2800, 0.70, 0));
        ShotSolver s = new ShotSolver(shooter(Element.POLLEN, 0, 0));
        Motion m = new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0);
        Cell[] one = {Cell.STARTS_UP};
        Result nominal = s.solve(m, one, false, W, 0)[0];
        s.setMeasuredRpm(0, nominal.rpm + 40);
        assertEquals(nominal.hoodPosition, s.solve(m, one, false, W, 0.01)[0].hoodPosition, 1e-9);
        s.setMeasuredRpm(0, nominal.rpm - 100);
        Result sagged = s.solve(m, one, false, W, 0.02)[0];
        assertEquals(100, sagged.droopRpm, 1e-9);
        assertEquals(nominal.hoodPosition + 0.05, sagged.hoodPosition, 1e-9);
        assertEquals(nominal.rpm, sagged.rpm, 1e-9);
    }

    @Test
    public void keepOutInvalidatesNectar() {
        ShotTuning.KeepOut k = new ShotTuning.KeepOut();
        k.pollenMinDeg = -10;
        k.pollenMaxDeg = 10;
        k.nectarMinDeg = -10;
        k.nectarMaxDeg = 10;
        ShotTuning.keepOuts = new ShotTuning.KeepOut[] {k};
        Result[] rs = solver(0, 0).solve(new Motion(58, 10, FACING_AUDIENCE, 0, 0, 0), UP, false, W, 0);
        assertTrue(rs[0].isValid());
        assertEquals(Invalid.KEEP_OUT, rs[1].invalid);
    }

    @Test
    public void targetOutsideTurretLimitsIsInvalid() {
        Result r = solver(0, 0).solve(new Motion(58, 10, Math.PI / 2, 0, 0, 0), UP, false, W, 0)[0];
        assertEquals(Invalid.TURRET_RANGE, r.invalid);
    }

    @Test
    public void convergedShotLandsInTheOpening() {
        ShotTuning.pollen.kDragPerS = 0.4;
        ShotTuning.nectar.kDragPerS = 0.6;
        ShotTuning.phaseDelayS = 0;
        ShooterGeometry[] gs = {shooter(Element.POLLEN, -3, 6), shooter(Element.NECTAR, -3, -6)};
        ShotSolver s = new ShotSolver(gs);
        Motion m = new Motion(52, 14, FACING_AUDIENCE + 0.2, 25, 18, 0.8);
        Result[] rs = s.solve(m, UP, false, W, 0);
        double[] aim = aim();
        CellOpening opening = BioBuzzField.cellOpening(Cell.STARTS_UP, false, W);
        double g0 = ShotSolver.GRAVITY_IN_PER_S2;
        for (int i = 0; i < 2; i++) {
            Result r = rs[i];
            assertTrue(r.invalid.toString(), r.isValid());
            assertTrue(r.iterations < 50);
            double k = gs[i].element.tuning().kDragPerS;
            double reach = ShotSolver.dragTime(r.tofS, k);
            double dirX = (r.virtualXIn - r.pivotXIn) / r.distanceIn;
            double dirY = (r.virtualYIn - r.pivotYIn) / r.distanceIn;
            double vx = r.distanceIn / reach * dirX + r.shooterVxInPerS;
            double vy = r.distanceIn / reach * dirY + r.shooterVyInPerS;
            double vz = (aim[2] - 14 + g0 / k * r.tofS) / reach - g0 / k;
            double x = r.pivotXIn + r.shooterVxInPerS * gs[i].transferTimeS;
            double y = r.pivotYIn + r.shooterVyInPerS * gs[i].transferTimeS;
            double z = 14;
            int steps = 100000;
            double dt = r.tofS / steps;
            boolean crossed = false;
            for (int j = 0; j < steps; j++) {
                double side = (x - opening.bottomCentreXIn) * opening.n[0] + (y - opening.bottomCentreYIn) * opening.n[1] + (z - opening.bottomCentreZIn) * opening.n[2];
                if (!crossed && side <= 0) {
                    crossed = true;
                    double du = (x - opening.bottomCentreXIn) * opening.u[0];
                    double dw = (y - opening.bottomCentreYIn) * opening.w[1] + (z - opening.bottomCentreZIn) * opening.w[2];
                    assertTrue(CellOpening.inside(du, dw, gs[i].element.diameterIn / 2));
                    assertTrue(vz < 0);
                }
                vx -= k * vx * dt;
                vy -= k * vy * dt;
                vz -= (g0 + k * vz) * dt;
                x += vx * dt;
                y += vy * dt;
                z += vz * dt;
            }
            assertTrue(crossed);
            assertEquals(aim[0], x, 0.3);
            assertEquals(aim[1], y, 0.3);
            assertEquals(aim[2], z, 0.3);
        }
    }
}
