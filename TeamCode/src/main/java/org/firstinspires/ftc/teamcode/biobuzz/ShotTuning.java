package org.firstinspires.ftc.teamcode.biobuzz;

import com.acmerobotics.dashboard.config.Config;

/** Every shooting-on-the-move tunable. Values marked TODO are placeholders until tuned on the robot. */
@Config("Shot Tuning")
public final class ShotTuning {
    private ShotTuning() {}

    public enum RateMethod { ANALYTIC, NUMERICAL }

    /**
     * One distance's settings; a point at distance 0 is unused. Extra points at the same distance with a lower rpm
     * are droop points: the hood to use there when the flywheel has sagged to that rpm (their tofS is ignored).
     */
    public static class ShotPoint {
        public double distanceIn = 0;
        public double rpm = 0;
        public double hoodPosition = 0.5;
        public double tofS = 0;

        public ShotPoint() {}

        public ShotPoint(double distanceIn, double rpm, double hoodPosition, double tofS) {
            this.distanceIn = distanceIn;
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
            this.tofS = tofS;
        }
    }

    public static class ElementTuning {
        /** Linear drag rate (1/s); 0 is no drag. TODO: tune. */
        public double kDragPerS = 0;
        public double minDistanceIn = 0;
        public double maxDistanceIn = 1000;
        public ShotPoint[] table;

        public ElementTuning(ShotPoint... table) {
            this.table = table;
        }
    }

    public static class ShooterTuning {
        /** Turret pivot from the robot's centre: forward and left. TODO: tune. */
        public double pivotForwardIn = 0;
        public double pivotLeftIn = 0;
        /** Launch point's sideways offset from the turret's centre line, left positive. TODO: tune. */
        public double exitLeftIn = 0;
        /** Launch point's height above the TILES. TODO: tune. */
        public double exitHeightIn = 14;
        public double minAngleDeg = -90;
        public double maxAngleDeg = 90;
        /**
         * Extra lead added to the flight time. A turret that keeps tracking until the ball leaves needs none, so
         * raise it only if moving shots still land ahead of travel after phaseDelayS and servoLagS are tuned.
         */
        public double transferTimeS = 0;
        /** Servo lead time: command leads the solved angle by rateFF times this. TODO: tune. */
        public double servoLagS = 0.05;
        /** Fastest the turret can turn; a shot needing more is invalid. TODO: tune. */
        public double maxTurretRateDegPerS = 300;
        /** Flywheel target leads the table by its rate of change times this while the target rpm rises. TODO: tune. */
        public double flywheelSpinUpLeadS = 0;
        /** The same while the target rpm falls; a flywheel that only coasts down needs more. TODO: tune. */
        public double flywheelSpinDownLeadS = 0;
        /** Hood target leads the table by its rate of change times this. TODO: tune. */
        public double hoodLeadS = 0;
        public double rpmTrim = 0;
        public double hoodTrim = 0;
        public double angleTrimDeg = 0;
        public double angleToleranceDeg = 2;
        public double rpmTolerance = 75;
    }

    /** Off: stationary auto-aim (robot velocity taken as zero). */
    public static boolean sotmEnabled = false;
    /** Weight toward the commanded velocity; 0 when there is no command (e.g. auto paths). */
    public static double alpha = 0.2;
    public static double phaseDelayS = 0.03;
    /** Low-pass time constant on the measured velocity; 0 is unfiltered. TODO: tune. */
    public static double velocityFilterS = 0;
    /** Below this speed the robot counts as still, fading to full SOTM at twice it; 0 is off. TODO: tune. */
    public static double minSotmSpeedInPerS = 2;
    /** Robot speed above which no shot is valid. TODO: tune. */
    public static double maxShotSpeedInPerS = 60;
    /** How long a shot may drop out of ready before feeding stops. */
    public static double readyDropoutS = 0.1;
    public static int maxIterations = 10;
    public static double convergenceIn = 0.1;
    public static RateMethod rateMethod = RateMethod.ANALYTIC;
    public static double numericalWindowS = 0.1;

    /** Half-width of the strip along the audience-to-rear midline where the target CELL doesn't switch. */
    public static double cellSwitchBandIn = 3;

    /** Aim this far inside the CELL past the centre of its opening. */
    public static double aimDepthIn = 3;
    /** Clearance from the opening's edge beyond the ball's radius. TODO: tune. */
    public static double rimClearanceIn = 1;
    public static boolean requireDescendingEntry = true;
    /** Grows the no-shoot box around the HIVE frame. TODO: tune. */
    public static double hiveBadZoneMarginIn = 6;

    public static ElementTuning pollen = new ElementTuning(emptyTable(20));
    public static ElementTuning nectar = new ElementTuning(emptyTable(20));
    public static ShooterTuning pollenShooter = new ShooterTuning();
    public static ShooterTuning nectarShooter = new ShooterTuning();

    private static ShotPoint[] emptyTable(int size) {
        ShotPoint[] table = new ShotPoint[size];
        for (int i = 0; i < size; i++) table[i] = new ShotPoint();
        return table;
    }

    // TODO: tune: placeholder rows (distanceIn, rpm, hoodPosition, tofS) that only keep the solver's maths exercised; rows 5 to 19 are free.
    static {
        pollen.kDragPerS = 0;
        pollen.minDistanceIn = 0;
        pollen.maxDistanceIn = 1000;
        pollen.table[0] = new ShotPoint(36, 2500, 0.5, 0.55);
        pollen.table[1] = new ShotPoint(60, 2800, 0.5, 0.62);
        pollen.table[2] = new ShotPoint(84, 3100, 0.5, 0.70);
        pollen.table[3] = new ShotPoint(108, 3400, 0.5, 0.80);
        pollen.table[4] = new ShotPoint(132, 3700, 0.5, 0.90);
    }

    // TODO: tune: placeholder rows, as pollen's.
    static {
        nectar.kDragPerS = 0;
        nectar.minDistanceIn = 0;
        nectar.maxDistanceIn = 1000;
        nectar.table[0] = new ShotPoint(36, 2500, 0.5, 0.55);
        nectar.table[1] = new ShotPoint(60, 2800, 0.5, 0.62);
        nectar.table[2] = new ShotPoint(84, 3100, 0.5, 0.70);
        nectar.table[3] = new ShotPoint(108, 3400, 0.5, 0.80);
        nectar.table[4] = new ShotPoint(132, 3700, 0.5, 0.90);
    }

    // TODO: tune.
    static {
        pollenShooter.pivotForwardIn = 0; // X and Y of the turret position, positive is left and up
        pollenShooter.pivotLeftIn = 0; // ^^^
        pollenShooter.exitLeftIn = 0;
        pollenShooter.exitHeightIn = 14;
        pollenShooter.minAngleDeg = -90;
        pollenShooter.maxAngleDeg = 90;
        pollenShooter.transferTimeS = 0;
        pollenShooter.servoLagS = 0.05;
        pollenShooter.maxTurretRateDegPerS = 300;
        pollenShooter.flywheelSpinUpLeadS = 0;
        pollenShooter.flywheelSpinDownLeadS = 0;
        pollenShooter.hoodLeadS = 0;
        pollenShooter.rpmTrim = 0;
        pollenShooter.hoodTrim = 0;
        pollenShooter.angleTrimDeg = 0;
        pollenShooter.angleToleranceDeg = 2;
        pollenShooter.rpmTolerance = 75;
    }

    // TODO: tune.
    static {
        nectarShooter.pivotForwardIn = 0;
        nectarShooter.pivotLeftIn = 0;
        nectarShooter.exitLeftIn = 0;
        nectarShooter.exitHeightIn = 14;
        nectarShooter.minAngleDeg = -90;
        nectarShooter.maxAngleDeg = 90;
        nectarShooter.transferTimeS = 0;
        nectarShooter.servoLagS = 0.05;
        nectarShooter.maxTurretRateDegPerS = 300;
        nectarShooter.flywheelSpinUpLeadS = 0;
        nectarShooter.flywheelSpinDownLeadS = 0;
        nectarShooter.hoodLeadS = 0;
        nectarShooter.rpmTrim = 0;
        nectarShooter.hoodTrim = 0;
        nectarShooter.angleTrimDeg = 0;
        nectarShooter.angleToleranceDeg = 2;
        nectarShooter.rpmTolerance = 75;
    }

    /**
     * Turret angle pairs that collide: while the pollen turret is in [pollenMin, pollenMax] and the nectar turret in
     * [nectarMin, nectarMax], the nectar shooter is marked invalid. All zeros is unused. TODO: fill once mounted.
     */
    public static class KeepOut {
        public double pollenMinDeg = 0;
        public double pollenMaxDeg = 0;
        public double nectarMinDeg = 0;
        public double nectarMaxDeg = 0;
    }

    public static KeepOut[] keepOuts = {new KeepOut(), new KeepOut()};
}
