package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;

/** Author path geometry for RED; these mirror to BLUE when {@link Context#allianceColor} flips. */
public final class FieldPose {
    private FieldPose() {}

    public static Pose forAlliance(double x, double y, double heading) {
        if (Context.allianceColor != AllianceColor.BLUE) return new Pose(x, y, heading);
        return new Pose(FieldConfig.fieldWidthInches - x, y, Angle.normalize(Math.PI - heading));
    }
}
