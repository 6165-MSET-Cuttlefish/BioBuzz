package org.firstinspires.ftc.teamcode.modules.vision;

/** Codes match ball_contour_snapscript.py's llpython type codes. */
public enum BallType {
    POLLEN(1, "Pollen", 2.8),
    NECTAR_RED(2, "Red Nectar", 3.6),
    NECTAR_BLUE(3, "Blue Nectar", 3.6);

    public final int code;
    public final String label;
    public final double diameterIn;

    BallType(int code, String label, double diameterIn) {
        this.code = code;
        this.label = label;
        this.diameterIn = diameterIn;
    }

    public static BallType fromCode(int code) {
        for (BallType type : values()) {
            if (type.code == code) return type;
        }
        throw new IllegalArgumentException("Unknown ball type code " + code);
    }
}
