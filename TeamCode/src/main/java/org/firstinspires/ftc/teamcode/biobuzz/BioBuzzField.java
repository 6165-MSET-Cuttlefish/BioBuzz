package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldSymmetry;
import org.firstinspires.ftc.teamcode.architecture.auto.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.auto.Region;

import java.util.Arrays;
import java.util.List;

/** BIOBUZZ field facts from the Competition Manual, in Pedro coordinates authored for RED. */
public final class BioBuzzField {
    private BioBuzzField() {}

    /** Red's ALLIANCE AREA is on the audience's left wall and blue's on the right; the field is point-symmetric. */
    public static final FieldSymmetry SYMMETRY = FieldSymmetry.ROTATE_180;

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
     * This alliance's half (G402: columns A-C are RED's during AUTO) for the robot's centre, {@code marginIn} in
     * from the walls and the centre line. RED's is x in [m, W/2 - m], BLUE's x in [W/2 + m, W - m].
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
