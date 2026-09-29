package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;

/**
 * Author path geometry for RED; it maps to BLUE through {@link FieldConfig#symmetry()} when
 * {@link Context#allianceColor} is BLUE. Throws on either alliance until the symmetry is set.
 */
public final class FieldPose {
    private FieldPose() {}

    public static Pose forAlliance(double x, double y, double heading) {
        FieldSymmetry symmetry = FieldConfig.symmetry();
        if (Context.allianceColor != AllianceColor.BLUE) return new Pose(x, y, heading);
        return symmetry.toBlue(x, y, heading, FieldConfig.fieldWidthInches);
    }
}
