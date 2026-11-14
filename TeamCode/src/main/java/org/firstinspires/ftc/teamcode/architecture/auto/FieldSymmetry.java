package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

public enum FieldSymmetry {
    MIRROR_X {
        @Override
        public Pose toBlue(double x, double y, double heading, double width) {
            return new Pose(width - x, y, Angle.normalize(Math.PI - heading));
        }
    },
    ROTATE_180 {
        @Override
        public Pose toBlue(double x, double y, double heading, double width) {
            return new Pose(width - x, width - y, Angle.normalize(heading + Math.PI));
        }
    };

    public abstract Pose toBlue(double x, double y, double heading, double width);
}
