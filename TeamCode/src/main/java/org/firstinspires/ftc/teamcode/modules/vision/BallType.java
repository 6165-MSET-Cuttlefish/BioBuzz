package org.firstinspires.ftc.teamcode.modules.vision;

/** Codes match the ball SnapScripts' llpython type codes. */
public enum BallType {
    POLLEN(1, "Pollen"),
    NECTAR_RED(2, "Red Nectar"),
    NECTAR_BLUE(3, "Blue Nectar");

    public final int code;
    public final String label;

    BallType(int code, String label) {
        this.code = code;
        this.label = label;
    }

    public static BallType fromCode(int code) {
        for (BallType type : values()) {
            if (type.code == code) return type;
        }
        throw new IllegalArgumentException("Unknown ball type code " + code);
    }
}
