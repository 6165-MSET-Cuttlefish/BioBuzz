package org.firstinspires.ftc.teamcode.opmodes.test.bench;

/** Angles for the nectar turret's Through Bore: its absolute pulse width, the hub's count, and the turret turn. */
final class TurretZeroMath {
    static final int COUNTS_PER_TURN = 8192;
    static final double ENCODER_TURNS_PER_TURRET_TURN = 50.0 / 21.0;
    static final int ENCODER_DIRECTION = -1;
    static final double PULSE_SLACK_US = 10;

    /** The absolute output of each Through Bore: first and last pulse of one turn, and the positions in a turn. */
    enum Version {
        V1(1, 1024, 1024),
        V2(3.884, 998.06, 4096);

        final double minUs;
        final double maxUs;
        final int positions;

        Version(double minUs, double maxUs, int positions) {
            this.minUs = minUs;
            this.maxUs = maxUs;
            this.positions = positions;
        }
    }

    private TurretZeroMath() {}

    /** Encoder degrees in [0, 360): minUs reads 0 and maxUs the last position before the wrap. */
    static double pulseToEncoderDeg(double pulseUs, double minUs, double maxUs, int positions) {
        double turnUs = (maxUs - minUs) * positions / (positions - 1);
        return wrap360((pulseUs - minUs) / turnUs * 360);
    }

    static double pulseToEncoderDeg(double pulseUs, Version version) {
        return pulseToEncoderDeg(pulseUs, version.minUs, version.maxUs, version.positions);
    }

    /** False for no pulse (0) and for widths more than PULSE_SLACK_US outside the encoder's range. */
    static boolean pulsePlausible(double pulseUs, double minUs, double maxUs) {
        return pulseUs > 0 && pulseUs >= minUs - PULSE_SLACK_US && pulseUs <= maxUs + PULSE_SLACK_US;
    }

    /** Encoder degrees turned since count0, unwrapped; sign is +1 when the count rises with the absolute angle. */
    static double quadratureDeg(int count0, int count, int sign) {
        return sign * (double) (count - count0) * 360.0 / COUNTS_PER_TURN;
    }

    /** a - b in [-180, 180). */
    static double wrapErrorDeg(double a, double b) {
        return wrap360(a - b + 180) - 180;
    }

    static double wrap360(double deg) {
        double r = deg % 360;
        if (r < 0) r += 360;
        return r >= 360 ? 0 : r;
    }

    /** The encoder angle at count 0, from one absolute read and the count at that moment. */
    static double zeroOffsetDeg(double absoluteDeg, int count, int sign) {
        return wrap360(absoluteDeg - quadratureDeg(0, count, sign));
    }

    /** How far the angle counted from the zero rest is from a rest's absolute angle. */
    static double agreementErrorDeg(double zeroAbsoluteDeg, int zeroCount, double absoluteDeg, int count, int sign) {
        return wrapErrorDeg(absoluteDeg, zeroAbsoluteDeg + quadratureDeg(zeroCount, count, sign));
    }

    /**
     * The turret angle nearest roughTurretDeg at which the encoder reads encoderDeg. offsetDeg is a turret angle where
     * it reads 0; the reading repeats every 360 / ENCODER_TURNS_PER_TURRET_TURN = 151.2 degrees of turret.
     */
    static double turretFromAbsolute(double encoderDeg, double roughTurretDeg, double offsetDeg) {
        return turretFromAbsolute(encoderDeg, roughTurretDeg, offsetDeg, ENCODER_TURNS_PER_TURRET_TURN,
                ENCODER_DIRECTION);
    }

    static double turretFromAbsolute(double encoderDeg, double roughTurretDeg, double offsetDeg,
                                     double encoderTurnsPerTurretTurn, int direction) {
        double roughEncoderDeg = direction * (roughTurretDeg - offsetDeg) * encoderTurnsPerTurretTurn;
        double turns = Math.round((roughEncoderDeg - encoderDeg) / 360);
        return offsetDeg + direction * (encoderDeg + 360 * turns) / encoderTurnsPerTurretTurn;
    }
}
