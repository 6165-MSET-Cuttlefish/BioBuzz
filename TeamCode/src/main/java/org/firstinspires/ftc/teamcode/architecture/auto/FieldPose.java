package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;

public final class FieldPose {
    private FieldPose() {}

    public static Pose forAlliance(double x, double y, double heading) {
        FieldSymmetry symmetry = FieldConfig.symmetry();
        if (Context.allianceColor != AllianceColor.BLUE) return new Pose(x, y, heading);
        return symmetry.toBlue(x, y, heading, FieldConfig.fieldWidthInches);
    }
}
