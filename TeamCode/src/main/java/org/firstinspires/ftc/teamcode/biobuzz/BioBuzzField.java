package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldSymmetry;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.planner.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.planner.Region;

import java.util.Arrays;
import java.util.List;

/** BIOBUZZ field facts from the Competition Manual, in Pedro coordinates authored for RED. */
public final class BioBuzzField {
    private BioBuzzField() {}

    /** Red's ALLIANCE AREA is on the audience's left wall and blue's on the right; the field is point-symmetric. */
    public static final FieldSymmetry SYMMETRY = FieldSymmetry.ROTATE_180;

    public static final String IMAGE = "/images/biobuzz.png";

    public static void configure() {
        FieldConfig.setSymmetry(SYMMETRY);
        FieldConfig.setImage(IMAGE, FieldConfig.fieldWidthInches, 1);
    }

    // Figs 9-9/9-10: HIVE centres 25.5 in apart; a CELL's centre is 15.44 in from the pivot along a bar tilted 30°, 13.37 in plan.
    private static final double HIVE_PIVOT_FROM_CENTRE_IN = 12.75;
    private static final double CELL_FROM_PIVOT_IN = 13.37;

    /** The centre, in plan, of this alliance's HIVE CELL on the audience side, or else the rear one. */
    public static Pose ownCell(boolean audienceSide) {
        double centre = FieldConfig.fieldWidthInches / 2;
        double pivotX = FieldPose.forAlliance(centre - HIVE_PIVOT_FROM_CENTRE_IN, centre, 0).x();
        return new Pose(pivotX, audienceSide ? centre - CELL_FROM_PIVOT_IN : centre + CELL_FROM_PIVOT_IN);
    }

    /** Fig 10-2: RED's CELL on the audience side starts up; BLUE's starts up on the rear side, its ROTATE_180 image. */
    public enum Cell {
        STARTS_UP(-1),
        STARTS_DOWN(1);

        /** RED's: -1 is the audience side (y below the centre), +1 the rear. */
        final int redSide;

        Cell(int redSide) {
            this.redSide = redSide;
        }

        public Cell other() {
            return this == STARTS_UP ? STARTS_DOWN : STARTS_UP;
        }
    }

    /** This alliance's CELL whose opening faces the audience half of the field, or else the rear half. */
    public static Cell cellFacing(boolean audienceHalf, boolean blue) {
        return audienceHalf != blue ? Cell.STARTS_UP : Cell.STARTS_DOWN;
    }

    // Fig 9-9: a 42.91 in bar, CELL opening at its end, square to it; Fig 9-10: the scorable CELL tilts 30° up, its opening 53.5 to 65.6 in above the TILES.
    private static final double CELL_TILT_RAD = Math.toRadians(30);
    private static final double OPENING_FROM_PIVOT_ALONG_BAR_IN = 42.91 / 2; // TODO: verify against the field CAD
    public static final double OPENING_BOTTOM_HEIGHT_IN = 53.5;
    // Fig 9-11: a pentagon 20 in wide, 14 in tall, its straight sides 7.61 in.
    public static final double OPENING_WIDTH_IN = 20;
    public static final double OPENING_SHOULDER_IN = 7.61;
    public static final double OPENING_HEIGHT_IN = 14;

    /**
     * A scorable CELL's opening in Pedro coordinates: {@code u} runs across it along the field's x, {@code w} up its face
     * from the bottom edge, {@code n} out of it. Its outline in (u, w) runs from (-10, 0) to (10, 0), the shoulders at
     * 7.61 in, the apex at (0, 14).
     */
    public static final class CellOpening {
        public final double bottomCentreXIn, bottomCentreYIn, bottomCentreZIn;
        public final double[] u, w, n;

        CellOpening(double x, double y, double z, double[] u, double[] w, double[] n) {
            bottomCentreXIn = x;
            bottomCentreYIn = y;
            bottomCentreZIn = z;
            this.u = u;
            this.w = w;
            this.n = n;
        }

        public double[] point(double uIn, double wIn, double outIn) {
            return new double[] {
                    bottomCentreXIn + uIn * u[0] + wIn * w[0] + outIn * n[0],
                    bottomCentreYIn + uIn * u[1] + wIn * w[1] + outIn * n[1],
                    bottomCentreZIn + uIn * u[2] + wIn * w[2] + outIn * n[2]};
        }

        /** The pentagon's centroid, {@code depthIn} inside the CELL. */
        public double[] aimPoint(double depthIn) {
            return point(0, openingCentroidWIn(), -depthIn);
        }

        /** Whether (uIn, wIn) lies inside the outline with {@code marginIn} to spare from every edge. */
        public static boolean inside(double uIn, double wIn, double marginIn) {
            double half = OPENING_WIDTH_IN / 2;
            if (wIn < marginIn || Math.abs(uIn) > half - marginIn) return false;
            double rise = OPENING_HEIGHT_IN - OPENING_SHOULDER_IN;
            double slant = Math.hypot(half, rise);
            // Distance below the roof edge from (half, shoulder) to (0, height), measured square to it.
            double belowRoof = ((OPENING_HEIGHT_IN - wIn) * half - Math.abs(uIn) * rise) / slant;
            return belowRoof >= marginIn;
        }
    }

    static double openingCentroidWIn() {
        double rectArea = OPENING_WIDTH_IN * OPENING_SHOULDER_IN;
        double triArea = OPENING_WIDTH_IN * (OPENING_HEIGHT_IN - OPENING_SHOULDER_IN) / 2;
        double triCentroid = OPENING_SHOULDER_IN + (OPENING_HEIGHT_IN - OPENING_SHOULDER_IN) / 3;
        return (rectArea * OPENING_SHOULDER_IN / 2 + triArea * triCentroid) / (rectArea + triArea);
    }

    /** This alliance's {@code cell}, as it lies when scorable, on a field {@code widthIn} wide. */
    public static CellOpening cellOpening(Cell cell, boolean blue, double widthIn) {
        double centre = widthIn / 2;
        double side = cell.redSide;
        double x = centre - HIVE_PIVOT_FROM_CENTRE_IN;
        double y = centre + side * OPENING_FROM_PIVOT_ALONG_BAR_IN * Math.cos(CELL_TILT_RAD);
        double[] u = {1, 0, 0};
        double[] w = {0, -side * Math.sin(CELL_TILT_RAD), Math.cos(CELL_TILT_RAD)};
        double[] n = {0, side * Math.cos(CELL_TILT_RAD), Math.sin(CELL_TILT_RAD)};
        if (blue) {
            x = widthIn - x;
            y = widthIn - y;
            u = new double[] {-1, 0, 0};
            w = new double[] {0, -w[1], w[2]};
            n = new double[] {0, -n[1], n[2]};
        }
        return new CellOpening(x, y, OPENING_BOTTOM_HEIGHT_IN, u, w, n);
    }

    public static CellOpening cellOpening(Cell cell) {
        return cellOpening(cell, Context.allianceColor == AllianceColor.BLUE, FieldConfig.fieldWidthInches);
    }

    /** The HIVE frame's footprint between its rails, {@code marginIn} larger all round, the same for both alliances. */
    public static boolean underHive(double xIn, double yIn, double marginIn, double widthIn) {
        double centre = widthIn / 2;
        return Math.abs(xIn - centre) <= HIVE_RAIL_FAR_IN + marginIn
                && Math.abs(yIn - centre) <= HIVE_RAIL_HALF_LENGTH_IN + marginIn;
    }

    // Each rail is the foot bar of one A-frame with the feet and legs on it, from FIRST's field CAD, measured from the field's centre.
    private static final double HIVE_RAIL_NEAR_IN = 22.75;
    private static final double HIVE_RAIL_FAR_IN = 24.733;
    private static final double HIVE_RAIL_HALF_LENGTH_IN = 19.4723;

    /**
     * The HIVE frame's two ground rails, one on each side of the centre line and parallel to it: the only parts of
     * the HIVE a robot can't drive under. Centred on the field, so they are the same for both alliances.
     */
    public static List<Obstacle> hiveRails() {
        double centre = FieldConfig.fieldWidthInches / 2;
        double minY = centre - HIVE_RAIL_HALF_LENGTH_IN;
        double maxY = centre + HIVE_RAIL_HALF_LENGTH_IN;
        return Arrays.asList(
                new Obstacle(centre - HIVE_RAIL_FAR_IN, centre - HIVE_RAIL_NEAR_IN, minY, maxY),
                new Obstacle(centre + HIVE_RAIL_NEAR_IN, centre + HIVE_RAIL_FAR_IN, minY, maxY));
    }

    /**
     * This alliance's half (G402: columns A-C are RED's during AUTO), {@code marginIn} in from the walls and the
     * centre line; the ball planner keeps the robot's whole footprint inside it. RED's is x in [m, W/2 - m], BLUE's x in [W/2 + m, W - m].
     */
    public static Region ownHalf(double marginIn) {
        double width = FieldConfig.fieldWidthInches;
        if (!(marginIn >= 0 && 4 * marginIn < width)) {
            throw new IllegalArgumentException(String.format(
                    "wall margin %.1f in leaves no room in half of a %.1f in field", marginIn, width));
        }
        Pose a = FieldPose.forAlliance(marginIn, marginIn, 0);
        Pose b = FieldPose.forAlliance(width / 2 - marginIn, width - marginIn, 0);
        return Region.spanning(a, b);
    }
}
