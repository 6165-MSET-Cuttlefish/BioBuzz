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
 * Limelight 3A HIVE-cell tip detector. Inverting FIRST's "AprilTag Clusters" Tech Tip, this
 * alliance's cluster upside-down ({@code |roll| >= 90}) is scorable; right-side up or out of frame
 * is tipped, each after the script's hold time. The script sends only that verdict and an alliance checksum. The alliance
 * pipeline is selected once in {@link #init()}, so set {@link Context#allianceColor} in
 * {@code createRobot()}.
 */
@Config
public class LimelightCamera extends Module {

    public static boolean limelightTelemetry = true;

    public static int redPipeline = 1;
    public static int bluePipeline = 2;

    public static long maxStalenessMs = 250;
    public static double checkTipTimeoutMs = 10000;

    private static final String DEFAULT_NAME = "limelight";
    private static final String WRONG_PIPELINE_WARNING = "incorrect pipeline! please switch manually";

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
    private static final int OUT_ALLIANCE_CHECKSUM = 1;
    private static final int OUT_LENGTH = 2;

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
    private boolean wrongPipeline;
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

    /** An empty frame also reads tipped, so a camera pointed away looks like a tipped cell. */
    public boolean isTipped() {
        return fresh && tipped;
    }

    public boolean isScorable() {
        return fresh && !tipped;
    }

    public boolean hasVerdict() {
        return fresh;
    }

    /** The Limelight is answering, but not with this alliance's cell-tip script. */
    public boolean isWrongPipeline() {
        return wrongPipeline;
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
        wrongPipeline = !fresh && alliance != null && result != null
                && stalenessMs <= maxStalenessMs;
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
    }

    private void clearVerdict() {
        fresh = false;
        tipped = false;
        wrongPipeline = false;
    }

    @Override
    protected void onTelemetry() {
        if (!limelightTelemetry) return;
        if (limelight == null) {
            log("Limelight", "NOT CONFIGURED (\"%s\")", name);
            return;
        }
        logDashboard("Cell-tip pipeline", "%d (%s)", pipelineFor(alliance), alliance);
        if (wrongPipeline) log("Warning", WRONG_PIPELINE_WARNING);
        if (!fresh) {
            // Cached reads only: getStatus() is a blocking HTTP GET on the loop thread.
            boolean linked = limelight.isConnected();
            log("Cell", "NO VERDICT (%s, pipeline %s, staleness %.0fms)",
                    linked ? "linked" : "no link",
                    linked ? String.valueOf(limelight.getLatestResult().getPipelineIndex()) : "?",
                    stalenessMs);
            return;
        }
        logDashboard("Cell", tipped ? "TIPPED" : "SCORABLE");
    }
}
