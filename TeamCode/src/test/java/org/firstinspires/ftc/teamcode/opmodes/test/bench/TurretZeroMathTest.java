package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.opmodes.test.bench.TurretZeroMath.Version;
import org.junit.Test;

public class TurretZeroMathTest {
    private static final double EPS = 1e-9;
    private static final double TURN = 360 / TurretZeroMath.ENCODER_TURNS_PER_TURRET_TURN;

    private static double encoderAt(double turretDeg, double offsetDeg) {
        return TurretZeroMath.wrap360(TurretZeroMath.ENCODER_DIRECTION * (turretDeg - offsetDeg)
                * TurretZeroMath.ENCODER_TURNS_PER_TURRET_TURN);
    }

    @Test
    public void v1PulseIsOneMicrosecondPerPosition() {
        assertEquals(0, TurretZeroMath.pulseToEncoderDeg(1, Version.V1), EPS);
        assertEquals(90, TurretZeroMath.pulseToEncoderDeg(257, Version.V1), EPS);
        assertEquals(180, TurretZeroMath.pulseToEncoderDeg(513, Version.V1), EPS);
        assertEquals(360.0 * 1023 / 1024, TurretZeroMath.pulseToEncoderDeg(1024, Version.V1), EPS);
    }

    @Test
    public void v2PulseSpansItsTwelveBitRange() {
        assertEquals(0, TurretZeroMath.pulseToEncoderDeg(3.884, Version.V2), EPS);
        assertEquals(360.0 * 4095 / 4096, TurretZeroMath.pulseToEncoderDeg(998.06, Version.V2), EPS);
        double step = (998.06 - 3.884) / 4095;
        assertEquals(180, TurretZeroMath.pulseToEncoderDeg(3.884 + 2048 * step, Version.V2), EPS);
        assertTrue(Math.abs(TurretZeroMath.pulseToEncoderDeg(500, Version.V2)
                - TurretZeroMath.pulseToEncoderDeg(500, Version.V1)) > 1);
    }

    @Test
    public void lastAndFirstPulseAreOneStepApartAcrossTheWrap() {
        double last = TurretZeroMath.pulseToEncoderDeg(1024, Version.V1);
        double first = TurretZeroMath.pulseToEncoderDeg(1, Version.V1);
        assertEquals(360.0 / 1024, TurretZeroMath.wrapErrorDeg(first, last), EPS);
        assertEquals(-360.0 / 1024, TurretZeroMath.wrapErrorDeg(last, first), EPS);
        assertEquals(0, TurretZeroMath.pulseToEncoderDeg(1025, Version.V1), EPS);
    }

    @Test
    public void plausiblePulsesAllowTenMicrosecondsOfSlackAndRejectNoPulse() {
        assertTrue(TurretZeroMath.pulsePlausible(1, Version.V1.minUs, Version.V1.maxUs));
        assertTrue(TurretZeroMath.pulsePlausible(1034, Version.V1.minUs, Version.V1.maxUs));
        assertFalse(TurretZeroMath.pulsePlausible(1035, Version.V1.minUs, Version.V1.maxUs));
        assertFalse(TurretZeroMath.pulsePlausible(0, Version.V1.minUs, Version.V1.maxUs));
        assertFalse(TurretZeroMath.pulsePlausible(-3, Version.V2.minUs, Version.V2.maxUs));
    }

    @Test
    public void wrapErrorFoldsIntoPlusMinus180() {
        assertEquals(0.3, TurretZeroMath.wrapErrorDeg(0.2, 359.9), 1e-9);
        assertEquals(-0.3, TurretZeroMath.wrapErrorDeg(359.9, 0.2), 1e-9);
        assertEquals(-180, TurretZeroMath.wrapErrorDeg(180, 0), EPS);
        assertEquals(10, TurretZeroMath.wrapErrorDeg(730, 0), EPS);
        assertEquals(0, TurretZeroMath.wrap360(-1e-15), EPS);
    }

    @Test
    public void countToAngle() {
        assertEquals(360, TurretZeroMath.quadratureDeg(0, 8192, 1), EPS);
        assertEquals(90, TurretZeroMath.quadratureDeg(100, 2148, 1), EPS);
        assertEquals(-90, TurretZeroMath.quadratureDeg(100, 2148, -1), EPS);
        assertEquals(-720, TurretZeroMath.quadratureDeg(16384, 0, 1), EPS);
    }

    @Test
    public void zeroOffsetIsTheSameWhereverTheShaftStops() {
        double offset = 37.5;
        for (int count : new int[] {-20000, -1, 0, 4096, 123456}) {
            for (int sign : new int[] {-1, 1}) {
                double absolute = TurretZeroMath.wrap360(offset + TurretZeroMath.quadratureDeg(0, count, sign));
                assertEquals(0, TurretZeroMath.wrapErrorDeg(TurretZeroMath.zeroOffsetDeg(absolute, count, sign), offset),
                        1e-9);
            }
        }
    }

    @Test
    public void turretAngleFromAbsoluteRoundTripsOverTheTravel() {
        for (double offset : new double[] {0, 23.4, -151.2}) {
            for (double turret = -200; turret <= 200; turret += 0.7) {
                double encoder = encoderAt(turret, offset);
                assertEquals(turret, TurretZeroMath.turretFromAbsolute(encoder, turret, offset), 1e-9);
            }
        }
    }

    @Test
    public void picksTheRightTurnAtTheWrap() {
        double offset = 10;
        double atWrap = offset - 0.01 * 21 / 50;
        double justPast = offset + 0.01 * 21 / 50;
        assertTrue(encoderAt(atWrap, offset) < 0.02);
        assertTrue(encoderAt(justPast, offset) > 359.98);
        for (double turret : new double[] {atWrap, justPast, offset + TURN - 0.004, offset + TURN + 0.004}) {
            double encoder = encoderAt(turret, offset);
            assertEquals(turret, TurretZeroMath.turretFromAbsolute(encoder, turret + 70, offset), 1e-9);
            assertEquals(turret, TurretZeroMath.turretFromAbsolute(encoder, turret - 70, offset), 1e-9);
        }
    }

    @Test
    public void aRoughAngle75DegreesOffStillPicksTheRightTurn() {
        assertEquals(75.6, TURN / 2, 1e-9);
        for (double turret = -150; turret <= 150; turret += 5) {
            double encoder = encoderAt(turret, 0);
            assertEquals(turret, TurretZeroMath.turretFromAbsolute(encoder, turret + 75, 0), 1e-9);
            assertEquals(turret, TurretZeroMath.turretFromAbsolute(encoder, turret - 75, 0), 1e-9);
        }
    }

    @Test
    public void aRoughAngle76DegreesOrMoreOffPicksTheWrongTurn() {
        for (double turret = -150; turret <= 150; turret += 5) {
            double encoder = encoderAt(turret, 0);
            assertEquals(turret + TURN, TurretZeroMath.turretFromAbsolute(encoder, turret + 76, 0), 1e-9);
            assertEquals(turret - TURN, TurretZeroMath.turretFromAbsolute(encoder, turret - 76, 0), 1e-9);
            assertEquals(turret + TURN, TurretZeroMath.turretFromAbsolute(encoder, turret + 100, 0), 1e-9);
            assertEquals(turret - TURN, TurretZeroMath.turretFromAbsolute(encoder, turret - 140, 0), 1e-9);
        }
    }

    @Test
    public void theWrongDirectionGivesTheWrongTurretAngle() {
        int wrong = 0;
        for (double turret = -150; turret <= 150; turret += 5) {
            double encoder = encoderAt(turret, 0);
            double flipped = TurretZeroMath.turretFromAbsolute(encoder, turret, 0,
                    TurretZeroMath.ENCODER_TURNS_PER_TURRET_TURN, -TurretZeroMath.ENCODER_DIRECTION);
            if (Math.abs(flipped - turret) > 1) wrong++;
        }
        assertTrue(wrong > 55);
    }

    private static int[] counts(double... turns) {
        int[] out = new int[turns.length];
        for (int i = 0; i < turns.length; i++) out[i] = (int) Math.round(turns[i] * TurretZeroMath.COUNTS_PER_TURN);
        return out;
    }

    private static double maxError(double zeroDeg, int[] counts, double[] absolute, int sign) {
        double worst = 0;
        for (int i = 0; i < counts.length; i++) {
            worst = Math.max(worst, Math.abs(TurretZeroMath.agreementErrorDeg(zeroDeg, counts[0], absolute[i], counts[i],
                    sign)));
        }
        return worst;
    }

    @Test
    public void agreementIsExactForTheRightSignAndLargeForTheNegativeControls() {
        int[] counts = counts(0, 0.25, 0.6, 1.1, 1.5, 1.9, 2.3, 1.7, 0.9, 0.3);
        for (int sign : new int[] {-1, 1}) {
            double zero = 211.0;
            double[] absolute = new double[counts.length];
            for (int i = 0; i < counts.length; i++) {
                absolute[i] = TurretZeroMath.wrap360(zero + TurretZeroMath.quadratureDeg(counts[0], counts[i], sign));
            }
            assertEquals(0, maxError(zero, counts, absolute, sign), 1e-9);
            assertTrue(maxError(zero, counts, absolute, -sign) > 20);
            assertTrue(maxError(zero + 180, counts, absolute, sign) > 20);
        }
    }

    @Test
    public void aStationaryShaftCannotFailTheFlippedSignControl() {
        int[] counts = counts(0.4, 0.4, 0.4, 0.4);
        double[] absolute = {90, 90, 90, 90};
        assertEquals(0, maxError(90, counts, absolute, 1), 1e-9);
        assertEquals(0, maxError(90, counts, absolute, -1), 1e-9);
    }
}
