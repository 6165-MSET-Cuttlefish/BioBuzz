package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.config.Config;

/** Live-tunable so a smaller test field doesn't need a code change. */
@Config
public final class FieldConfig {
    private FieldConfig() {}

    public static double fieldWidthInches = 141.5;

    // Private so the dashboard can't show or null it; the game's Robot sets it when constructed.
    private static FieldSymmetry symmetry;

    public static void setSymmetry(FieldSymmetry fieldSymmetry) {
        if (fieldSymmetry == null) throw new IllegalArgumentException("field symmetry must not be null");
        symmetry = fieldSymmetry;
    }

    /** EnhancedOpMode calls this at every init, so each OpMode's Robot must set the symmetry again. */
    public static void clearSymmetry() {
        symmetry = null;
    }

    public static FieldSymmetry symmetry() {
        if (symmetry == null) {
            throw new IllegalStateException("FieldConfig symmetry was never set: the game's Robot (or a bare OpMode) "
                    + "calls FieldConfig.setSymmetry(...) before building any FieldPose");
        }
        return symmetry;
    }
}
