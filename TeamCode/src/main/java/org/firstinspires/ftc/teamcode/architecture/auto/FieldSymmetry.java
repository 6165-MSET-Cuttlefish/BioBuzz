package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

/** How a game's field maps RED geometry onto BLUE's, in Pedro coordinates on a square field {@code width} wide. */
public enum FieldSymmetry {
    /** Reflection across the centre line x = width / 2: (width - x, y, π - heading). */
    MIRROR_X {
        @Override
        public Pose toBlue(double x, double y, double heading, double width) {
            return new Pose(width - x, y, Angle.normalize(Math.PI - heading));
        }
    },
    /** Rotation by π about the field centre: (width - x, width - y, heading + π). */
    ROTATE_180 {
        @Override
        public Pose toBlue(double x, double y, double heading, double width) {
            return new Pose(width - x, width - y, Angle.normalize(heading + Math.PI));
        }
    };

    public abstract Pose toBlue(double x, double y, double heading, double width);
}
