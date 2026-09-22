package org.firstinspires.ftc.teamcode.architecture.core;

import com.acmerobotics.dashboard.config.Config;

/** Game-agnostic cross-opmode scratch. Only alliance side belongs here; game-specific shared state goes in a game-specific class. */
@Config
public final class Context {
    private Context() {}

    public static AllianceColor allianceColor = AllianceColor.RED;

    /** Which cell {@code LimelightCamera} watches. Read once at init to pick the Limelight pipeline; changing it later does nothing. */
    public static Cell cell = Cell.RED_1;

    /**
     * The four AprilTags under each cell, and the Limelight pipeline hard-coded to that set.
     *
     * <p>Each pipeline index holds its own copy of the SnapScript with its {@code TAG_IDS} baked in —
     * {@code limelight/pipelines/} — so these indices and those files have to agree.
     */
    public enum Cell {
        RED_1(0, 30, 31, 32, 33),
        RED_2(1, 34, 35, 36, 37),
        BLUE_1(2, 38, 39, 40, 41),
        BLUE_2(3, 42, 43, 44, 45);

        public final int pipeline;
        public final int[] tagIds;
        public final int checksum;

        Cell(int pipeline, int a, int b, int c, int d) {
            this.pipeline = pipeline;
            this.tagIds = new int[] {a, b, c, d};
            this.checksum = a + b + c + d;
        }
    }
}
