package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldSymmetry;
import org.firstinspires.ftc.teamcode.architecture.auto.Region;

/** BIOBUZZ field facts from the Competition Manual, in Pedro coordinates authored for RED. */
public final class BioBuzzField {
    private BioBuzzField() {}

    /** Red's ALLIANCE AREA is on the audience's left wall and blue's on the right; the field is point-symmetric. */
    public static final FieldSymmetry SYMMETRY = FieldSymmetry.ROTATE_180;

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
