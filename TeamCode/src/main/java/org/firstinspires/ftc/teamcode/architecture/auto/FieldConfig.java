package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.config.Config;

/** Live-tunable so a smaller test field doesn't need a code change. */
@Config
public final class FieldConfig {
    private FieldConfig() {}

    public static double fieldWidthInches = 141.5;

    // Private so the dashboard can't show or null it; the game's Robot sets it when constructed.
    private static FieldSymmetry symmetry;
    private static String image;
    private static double imageInches;
    private static double imageOpacity;

    public static void setSymmetry(FieldSymmetry fieldSymmetry) {
        if (fieldSymmetry == null) throw new IllegalArgumentException("field symmetry must not be null");
        symmetry = fieldSymmetry;
    }

    /**
     * The dashboard's field background: a render with Pedro's origin at its bottom-left corner, {@code inches} wide,
     * drawn at {@code opacity} (a light image needs less, or the white robot outline disappears into it).
     */
    public static void setImage(String path, double inches, double opacity) {
        if (path == null || !(inches > 0) || !(opacity > 0 && opacity <= 1)) {
            throw new IllegalArgumentException(String.format(
                    "field image %s must be a path, %s in wide (> 0), at opacity %s (0..1]", path, inches, opacity));
        }
        image = path;
        imageInches = inches;
        imageOpacity = opacity;
    }

    /** EnhancedOpMode calls this at every init, so each OpMode's Robot must set the symmetry and image again. */
    public static void clear() {
        symmetry = null;
        image = null;
    }

    public static String image() {
        return image;
    }

    public static double imageInches() {
        return imageInches;
    }

    public static double imageOpacity() {
        return imageOpacity;
    }

    public static FieldSymmetry symmetry() {
        if (symmetry == null) {
            throw new IllegalStateException("FieldConfig symmetry was never set: the game's Robot (or a bare OpMode) "
                    + "calls FieldConfig.setSymmetry(...) before building any FieldPose");
        }
        return symmetry;
    }
}
