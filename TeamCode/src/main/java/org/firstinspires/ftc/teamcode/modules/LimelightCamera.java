package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/**
 * Limelight 3A HIVE-cell tip detector. Per FIRST's "AprilTag Clusters" Tech Tip, this alliance's
 * cluster right-side up ({@code |roll| < 90}) is scorable; upside-down or out of frame for the hold
 * time is tipped. The alliance pipeline is selected once in {@link #init()}, so set
 * {@link Context#allianceColor} in {@code createRobot()}.
 */
@Config
public class LimelightCamera extends Module {

    public static boolean limelightTelemetry = true;

    public static int redPipeline = 1;
    public static int bluePipeline = 2;

    public static long maxStalenessMs = 250;
    public static double checkTipTimeoutMs = 10000;

    private static final String DEFAULT_NAME = "limelight";

    /** Sentinel for "no cluster in view": NaN is not valid JSON, so the scripts can't send it. */
    private static final double ROLL_NONE = 999.0;

    // limelight/generate_pipelines.py regex-parses these, so keep each on one line. The order differs
    // by alliance: RED's low run is the scoring-side cell, BLUE's the audience-side one.
    private static final int[] RED_SCORING_IDS = {30, 31, 32, 33};
    private static final int[] RED_AUDIENCE_IDS = {34, 35, 36, 37};
    private static final int[] BLUE_AUDIENCE_IDS = {38, 39, 40, 41};
    private static final int[] BLUE_SCORING_IDS = {42, 43, 44, 45};

    private static final int RED_CHECKSUM = sum(RED_SCORING_IDS) + sum(RED_AUDIENCE_IDS);
    private static final int BLUE_CHECKSUM = sum(BLUE_AUDIENCE_IDS) + sum(BLUE_SCORING_IDS);

    // llpython layout; must match the slot table in limelight/cell_tip_snapscript.py's docstring.
    private static final int OUT_TIPPED = 0;
    private static final int OUT_SCORABLE = 1;
    private static final int OUT_ROLL_DEG = 2;
    private static final int OUT_VISIBLE_COUNT = 3;
    private static final int OUT_CLUSTER = 4;
    private static final int OUT_OTHER_VISIBLE_COUNT = 5;
    private static final int OUT_ALLIANCE_CHECKSUM = 6;
    private static final int OUT_FRAME_COUNTER = 7;
    private static final int OUT_LENGTH = 8;

    /** Which of this alliance's two HIVE cells is in view; never identifies the alliance. */
    public enum Cluster {
        NONE(-1),
        /** The cell on the scoring-table side. */
        SCORING(0),
        AUDIENCE(1);

        public final int index;

        Cluster(int index) {
            this.index = index;
        }

        static Cluster of(int index) {
            for (Cluster c : values()) if (c.index == index) return c;
            return NONE;
        }
    }

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final Limelight3A limelight;
    private final String name;

    private AllianceColor alliance;
    private boolean polling;

    private boolean fresh;
    private boolean tipped;
    private boolean scorable;
    private double rollDeg;
    private int visibleCount;
    private Cluster cluster = Cluster.NONE;
    private int otherVisibleCount;
    private double stalenessMs;

    public LimelightCamera(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_NAME);
    }

    public LimelightCamera(HardwareMap hardwareMap, String name) {
        super();
        this.name = name;
        this.limelight = hardwareMap.tryGet(Limelight3A.class, name);
    }

    @Override
    protected void initStates() {
        setStates(VisionState.ENABLED);
    }

    @Override
    public void init() {
        alliance = Context.allianceColor;
        if (limelight == null) return;
        limelight.pipelineSwitch(pipelineFor(alliance));
        limelight.start();
        polling = true;
    }

    @Override
    protected void read() {
        if (limelight == null || !polling) return;
        parse(limelight.getLatestResult());
    }

    @Override
    protected void write() {
        if (limelight == null) return;
        boolean shouldPoll = isInAny(VisionState.ENABLED);
        if (shouldPoll == polling) return;
        polling = shouldPoll;
        if (polling) limelight.start();
        else {
            limelight.pause();
            clearVerdict();
        }
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    /** Fixed at init from {@link Context#allianceColor}. */
    public AllianceColor getAlliance() {
        return alliance;
    }

    /**
     * Debounced by the script's hold time. With {@code REQUIRE_SEEN} off, an empty frame also reads
     * tipped, so a camera pointed away from the hive looks like a tipped cell.
     */
    public boolean isTipped() {
        return fresh && tipped;
    }

    /** Undebounced, unlike {@link #isTipped()}. */
    public boolean isScorable() {
        return fresh && scorable;
    }

    public boolean hasVerdict() {
        return fresh;
    }

    public Cluster getCluster() {
        return fresh ? cluster : Cluster.NONE;
    }

    /** {@code |roll| < 90} is right-side up; NaN when no cluster is in view. */
    public double getRollDeg() {
        return fresh && rollDeg != ROLL_NONE ? rollDeg : Double.NaN;
    }

    public int getVisibleCount() {
        return fresh ? visibleCount : 0;
    }

    /** Tags of this alliance's <em>other</em> cell in the current frame. */
    public int getOtherVisibleCount() {
        return fresh ? otherVisibleCount : 0;
    }

    /** False when absent, unplugged, booting or paused. */
    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    public LimelightCamera requireDevice() {
        if (limelight == null) {
            throw new IllegalStateException("No \"" + name + "\" in the robot configuration; "
                    + "add the Limelight 3A to the hub config before using cell tip detection.");
        }
        return this;
    }

    private static int sum(int[] ids) {
        int total = 0;
        for (int id : ids) total += id;
        return total;
    }

    private static int pipelineFor(AllianceColor alliance) {
        return alliance == AllianceColor.BLUE ? bluePipeline : redPipeline;
    }

    private static int checksumFor(AllianceColor alliance) {
        return alliance == AllianceColor.BLUE ? BLUE_CHECKSUM : RED_CHECKSUM;
    }

    private void parse(LLResult result) {
        applyVerdict(result == null ? null : result.getPythonOutput(),
                result == null ? Double.NaN : result.getStaleness());
    }

    private void applyVerdict(double[] out, double stalenessMs) {
        this.stalenessMs = stalenessMs;

        fresh = alliance != null
                && out != null
                && out.length >= OUT_LENGTH
                && stalenessMs <= maxStalenessMs
                && (int) Math.round(out[OUT_ALLIANCE_CHECKSUM]) == checksumFor(alliance);
        if (!fresh) {
            clearVerdict();
            return;
        }

        tipped = out[OUT_TIPPED] != 0;
        scorable = out[OUT_SCORABLE] != 0;
        rollDeg = out[OUT_ROLL_DEG];
        visibleCount = (int) Math.round(out[OUT_VISIBLE_COUNT]);
        cluster = Cluster.of((int) Math.round(out[OUT_CLUSTER]));
        otherVisibleCount = (int) Math.round(out[OUT_OTHER_VISIBLE_COUNT]);
    }

    private void clearVerdict() {
        fresh = false;
        tipped = false;
        scorable = false;
        rollDeg = Double.NaN;
        visibleCount = 0;
        cluster = Cluster.NONE;
        otherVisibleCount = 0;
    }

    @Override
    protected void onTelemetry() {
        if (!limelightTelemetry) return;
        if (limelight == null) {
            log("Limelight", "NOT CONFIGURED (\"%s\")", name);
            return;
        }
        logDashboard("Alliance", "%s pipeline %d", alliance, pipelineFor(alliance));
        if (!fresh) {
            // Cached reads only: getStatus() is a blocking HTTP GET on the loop thread.
            boolean linked = limelight.isConnected();
            log("Cell", "NO VERDICT (%s, pipeline %s, staleness %.0fms)",
                    linked ? "linked" : "no link",
                    linked ? String.valueOf(limelight.getLatestResult().getPipelineIndex()) : "?",
                    stalenessMs);
            return;
        }
        logDashboard("Cell", tipped ? "TIPPED" : (scorable ? "SCORABLE" : "not scorable, within hold"));
        logDashboard("Cluster", "%s %d/4 visible", cluster, visibleCount);
        logDashboard("Roll", cluster == Cluster.NONE ? "--" : String.format("%.1fdeg", rollDeg));
        log("Other cluster", otherVisibleCount + "/4 visible");
    }
}
