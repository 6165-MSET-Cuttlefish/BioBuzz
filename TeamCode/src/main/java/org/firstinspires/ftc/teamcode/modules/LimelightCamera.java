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
 * Limelight 3A AprilTag subsystem: reports which of this alliance's two HIVE cells is scorable.
 *
 * <p>A cell's AprilTag cluster reads right-side up when the cell is up and accepting scoring
 * elements, and upside-down when it has tipped down and cannot be scored. The camera decides which
 * is which from the cluster's roll — its rotation about the camera's own viewing axis — and the
 * |roll| &lt; 90 test the game manual describes.
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

    /** Limelight pipeline holding the RED SnapScript (tags 30-37). */
    public static int redPipeline = 1;
    /** Limelight pipeline holding the BLUE SnapScript (tags 38-45). */
    public static int bluePipeline = 2;

    /** A result older than this is ignored — the Limelight has stalled, rebooted or lost its link. */
    public static long maxStalenessMs = 250;
    /** How long {@code RobotActions.checkTip} waits for a tip before giving up. */
    public static double checkTipTimeoutMs = 10000;

    private static final String DEFAULT_NAME = "limelight";

    /** The scripts send this in place of a roll when no cluster is in view; NaN is not valid JSON. */
    private static final double ROLL_NONE = 999.0;

    private static final int RED_CHECKSUM = 30 + 31 + 32 + 33 + 34 + 35 + 36 + 37;
    private static final int BLUE_CHECKSUM = 38 + 39 + 40 + 41 + 42 + 43 + 44 + 45;

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

    /** Which of the alliance's two clusters is in view; {@code NONE} when neither is. */
    public enum Cluster {
        NONE(-1),
        /** The lower of the alliance's two id runs — 30-33 for RED, 38-41 for BLUE. */
        LOW_IDS(0),
        /** The upper run — 34-37 for RED, 42-45 for BLUE. */
        HIGH_IDS(1);

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
     * Whether the cell in view has tipped down — its cluster has read upside-down
     * ({@code |roll| >= 90}) continuously for the hold time baked into the pipeline's script. False
     * whenever the answer isn't current or isn't about this alliance.
     */
    public boolean isTipped() {
        return fresh && tipped;
    }

    /** Alias of {@link #isTipped()}, for robot code that reads better as a check than a state query. */
    public boolean checkTip() {
        return isTipped();
    }

    /** Whether a right-side-up cluster of this alliance is in view, i.e. a cell that can be scored. */
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

    /** Roll of the cluster in view, in degrees; {@code |roll| < 90} is right-side up. NaN if none. */
    public double getRollDeg() {
        return fresh && rollDeg != ROLL_NONE ? rollDeg : Double.NaN;
    }

    /** How many of the in-view cluster's four tags are in the current frame. */
    public int getVisibleCount() {
        return fresh ? visibleCount : 0;
    }

    /** How many tags of this alliance's <em>other</em> cluster are in the current frame. */
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
        logDashboard("Cell", tipped ? "TIPPED" : (scorable ? "SCORABLE" : "unknown"));
        logDashboard("Cluster", "%s %d/4 visible", cluster, visibleCount);
        logDashboard("Roll", cluster == Cluster.NONE ? "--" : String.format("%.1fdeg", rollDeg));
        log("Other cluster", otherVisibleCount + "/4 visible");
    }
}
