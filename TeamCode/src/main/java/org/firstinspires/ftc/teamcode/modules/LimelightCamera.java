package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/**
 * Limelight 3A AprilTag subsystem: reports whether this alliance's HIVE cell is scorable.
 *
 * <p>The verdict is visibility. A cell that is up shows its AprilTag cluster to the camera; a cell
 * that has tipped down points its cluster away, so it leaves the frame entirely rather than staying
 * readable upside-down. Tags of this alliance in frame means scorable, none of them for the hold
 * time means tipped. The cluster's roll — its rotation about the camera's viewing axis, the same
 * angle the manual's {@code |roll| < 90} test uses — is measured and reported either way, and the
 * script's {@code REQUIRE_UPRIGHT} can make it gate the verdict too, for the case where a tipped
 * cluster stays in view instead of vanishing.
 *
 * <p>There is one Limelight pipeline per alliance, holding its own copy of the SnapScript with that
 * alliance's eight tag ids (both clusters) and the roll test baked in, so the hub writes nothing
 * over the link: it selects the pipeline for {@link Context#allianceColor} once at {@link #init()}
 * and then only reads {@code llpython}. The alliance-colour check the manual asks for is structural
 * — a pipeline only knows its own alliance's ids, so the other alliance's clusters can never be
 * targeted.
 *
 * <p>Because the pipeline is chosen at init, {@link Context#allianceColor} has to be set before the
 * OpMode initializes (in {@code createRobot()}); changing it later is ignored.
 */
@Config
public class LimelightCamera extends Module {

    public static boolean limelightTelemetry = true;

    /** Limelight pipeline holding the RED SnapScript (tags 30-37: 30-33 scoring side, 34-37 audience). */
    public static int redPipeline = 1;
    /** Limelight pipeline holding the BLUE SnapScript (tags 38-45: 38-41 audience side, 42-45 scoring). */
    public static int bluePipeline = 2;

    /** A result older than this is ignored — the Limelight has stalled, rebooted or lost its link. */
    public static long maxStalenessMs = 250;
    /** How long {@code RobotActions.checkTip} waits for a tip before giving up. */
    public static double checkTipTimeoutMs = 10000;

    private static final String DEFAULT_NAME = "limelight";

    /** The scripts send this in place of a roll when no cluster is in view; NaN is not valid JSON. */
    private static final double ROLL_NONE = 999.0;

    // The two HIVE cells per alliance, by which side of the field they sit on. The runs are not in
    // the same order for both alliances — RED's low run is the scoring-side cell, BLUE's is the
    // audience-side one — which is why the cluster code on the wire is the side, not the position.
    // scripts/generate-limelight-pipelines.py reads these to build the pipelines.
    private static final int[] RED_SCORING_IDS = {30, 31, 32, 33};
    private static final int[] RED_AUDIENCE_IDS = {34, 35, 36, 37};
    private static final int[] BLUE_AUDIENCE_IDS = {38, 39, 40, 41};
    private static final int[] BLUE_SCORING_IDS = {42, 43, 44, 45};

    private static final int RED_CHECKSUM = sum(RED_SCORING_IDS) + sum(RED_AUDIENCE_IDS);
    private static final int BLUE_CHECKSUM = sum(BLUE_AUDIENCE_IDS) + sum(BLUE_SCORING_IDS);

    // llpython (Limelight → hub), 8 doubles. Mirrored in every limelight/pipelines/ script.
    private static final int OUT_TIPPED = 0;
    private static final int OUT_SCORABLE = 1;
    private static final int OUT_ROLL_DEG = 2;
    private static final int OUT_VISIBLE_COUNT = 3;
    private static final int OUT_CLUSTER = 4;
    private static final int OUT_OTHER_VISIBLE_COUNT = 5;
    private static final int OUT_ALLIANCE_CHECKSUM = 6;
    private static final int OUT_FRAME_COUNTER = 7;
    private static final int OUT_LENGTH = 8;

    /**
     * Which of the alliance's two HIVE cells is in view; {@code NONE} when neither is.
     *
     * <p>Never the alliance. Alliance comes from {@link Context#allianceColor} and nothing else —
     * a pipeline only holds one alliance's ids, so a cluster code cannot distinguish alliances,
     * only which of that alliance's two cells the camera is pointed at. That does imply which end
     * of the field the robot is at (audience end vs scoring-table end), since each cell faces its
     * own end, but it says nothing about which alliance the robot is on.
     */
    public enum Cluster {
        NONE(-1),
        /** The cell on the scoring-table side — RED 30-33, BLUE 42-45. */
        SCORING(0),
        /** The cell on the audience side — RED 34-37, BLUE 38-41. */
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

    /** The alliance this OpMode is targeting, fixed at init from {@link Context#allianceColor}. */
    public AllianceColor getAlliance() {
        return alliance;
    }

    /**
     * Whether the cell has tipped down — nothing of this alliance's cluster has been scorable for
     * the hold time baked into the pipeline's script. A single tag coming back into view clears it
     * at once. False whenever the answer isn't current or isn't about this alliance.
     *
     * <p>With the script's {@code REQUIRE_SEEN} off (the default) an empty frame reports tipped once
     * the hold elapses, so a camera pointed away from the hive reads the same as a tipped cell.
     */
    public boolean isTipped() {
        return fresh && tipped;
    }

    /** Alias of {@link #isTipped()}, for robot code that reads better as a check than a state query. */
    public boolean checkTip() {
        return isTipped();
    }

    /** Whether enough of this alliance's cluster is in frame to score, undebounced. */
    public boolean isScorable() {
        return fresh && scorable;
    }

    /** True once a current verdict is in hand — false means "don't know yet". */
    public boolean hasVerdict() {
        return fresh;
    }

    /** Which of the alliance's clusters the verdict is about. */
    public Cluster getCluster() {
        return fresh ? cluster : Cluster.NONE;
    }

    /** Roll of the cluster in view, in degrees; {@code |roll| < 90} is right-side up. NaN if none. Reported, not acted on unless the script gates on it. */
    public double getRollDeg() {
        return fresh && rollDeg != ROLL_NONE ? rollDeg : Double.NaN;
    }

    /** How many of the in-view cluster's four tags are in the current frame. */
    public int getVisibleCount() {
        return fresh ? visibleCount : 0;
    }

    /** How many tags of this alliance's <em>other</em> cell are in the current frame. */
    public int getOtherVisibleCount() {
        return fresh ? otherVisibleCount : 0;
    }

    public boolean isPresent() {
        return limelight != null;
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
        double[] out = result == null ? null : result.getPythonOutput();
        stalenessMs = result == null ? Double.NaN : result.getStaleness();

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
            LLStatus status = limelight.getStatus();
            log("Cell", "NO VERDICT (pipeline %d, %.0ffps, staleness %.0fms)",
                    status.getPipelineIndex(), status.getFps(), stalenessMs);
            return;
        }
        logDashboard("Cell", tipped ? "TIPPED" : (scorable ? "SCORABLE" : "not scorable, within hold"));
        logDashboard("Cluster", "%s %d/4 visible", cluster, visibleCount);
        logDashboard("Roll", cluster == Cluster.NONE ? "--" : String.format("%.1fdeg", rollDeg));
        log("Other cluster", otherVisibleCount + "/4 visible");
    }
}
