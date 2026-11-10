package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * One shooter as {@link ShotSolver} sees it. Robot frame: forward and left of the robot's centre. A turret angle is
 * {@code sign * (robot-relative bearing - zeroBearingDeg)}, with robot-relative bearings counter-clockwise from forward.
 */
public final class ShooterGeometry {

    /** §9.8: POLLEN is a 2.8 in ball, NECTAR 3.6 in. */
    public enum Element {
        POLLEN(2.8),
        NECTAR(3.6);

        public final double diameterIn;

        Element(double diameterIn) {
            this.diameterIn = diameterIn;
        }

        public ShotTuning.ElementTuning tuning() {
            return this == POLLEN ? ShotTuning.pollen : ShotTuning.nectar;
        }
    }

    /** Our turrets: 0 at the robot's back, positive counter-clockwise (toward the robot's right). */
    public static final double BACK_ZERO_BEARING_DEG = 180;

    public final String name;
    public final Element element;
    public final double pivotForwardIn;
    public final double pivotLeftIn;
    public final double exitLeftIn;
    public final double exitHeightIn;
    public final double zeroBearingDeg;
    public final double sign;
    public final double minAngleDeg;
    public final double maxAngleDeg;
    public final double transferTimeS;

    public ShooterGeometry(String name, Element element, double pivotForwardIn, double pivotLeftIn, double exitLeftIn,
                           double exitHeightIn, double zeroBearingDeg, boolean counterClockwisePositive,
                           double minAngleDeg, double maxAngleDeg, double transferTimeS) {
        if (!(minAngleDeg < maxAngleDeg)) {
            throw new IllegalArgumentException(String.format("%s turret limits [%.1f, %.1f] are empty", name, minAngleDeg, maxAngleDeg));
        }
        this.name = name;
        this.element = element;
        this.pivotForwardIn = pivotForwardIn;
        this.pivotLeftIn = pivotLeftIn;
        this.exitLeftIn = exitLeftIn;
        this.exitHeightIn = exitHeightIn;
        this.zeroBearingDeg = zeroBearingDeg;
        this.sign = counterClockwisePositive ? 1 : -1;
        this.minAngleDeg = minAngleDeg;
        this.maxAngleDeg = maxAngleDeg;
        this.transferTimeS = transferTimeS;
    }

    public static ShooterGeometry fromTuning(String name, Element element, ShotTuning.ShooterTuning t) {
        return new ShooterGeometry(name, element, t.pivotForwardIn, t.pivotLeftIn, t.exitLeftIn, t.exitHeightIn,
                BACK_ZERO_BEARING_DEG, true, t.minAngleDeg, t.maxAngleDeg, t.transferTimeS);
    }

    public static ShooterGeometry pollen() {
        return fromTuning("Pollen", Element.POLLEN, ShotTuning.pollenShooter);
    }

    public static ShooterGeometry nectar() {
        return fromTuning("Nectar", Element.NECTAR, ShotTuning.nectarShooter);
    }
}
