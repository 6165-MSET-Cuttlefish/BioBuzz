package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/** HIVE-cell tip verdict from the cell-tip SnapScript; the alliance pipeline is fixed at init. */
@Config
public class LimelightCamera extends Module {

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

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final Limelight3A limelight;

    private AllianceColor alliance;
    private boolean polling;

    private LLResult result;
    private boolean fresh;
    private boolean tipped;
    private boolean wrongPipeline;
    private double stalenessMs;

    public LimelightCamera(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_NAME);
    }

    public LimelightCamera(HardwareMap hardwareMap, String name) {
        super();
        this.limelight = hardwareMap.get(Limelight3A.class, name);
    }

    @Override
    protected void initStates() {
        setStates(VisionState.ENABLED);
    }

    @Override
    public void init() {
        alliance = Context.allianceColor;
        limelight.pipelineSwitch(pipelineFor(alliance));
        limelight.start();
        polling = true;
    }

    @Override
    protected void read() {
        if (!polling) return;
        result = limelight.getLatestResult();
        parse();
    }

    @Override
    protected void write() {
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
        limelight.stop();
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

    private void parse() {
        // getLatestResult() re-synthesizes a null, but a malformed poll landing mid-call can still return one.
        if (result == null) {
            stalenessMs = Double.NaN;
            clearVerdict();
            return;
        }
        stalenessMs = result.getStaleness();
        double[] out = result.getPythonOutput();
        fresh = alliance != null
                && stalenessMs <= maxStalenessMs
                && (int) Math.round(out[OUT_ALLIANCE_CHECKSUM]) == checksumFor(alliance);
        tipped = fresh && out[OUT_TIPPED] != 0;
        // Before the first real poll the SDK serves a synthetic empty result with ~0 staleness.
        wrongPipeline = !fresh && alliance != null && stalenessMs <= maxStalenessMs
                && limelight.isConnected();
    }

    private void clearVerdict() {
        fresh = false;
        tipped = false;
        wrongPipeline = false;
    }

    @Override
    protected void onTelemetry() {
        logDashboard("Cell-tip pipeline", "%d (%s)", pipelineFor(alliance), alliance);
        if (wrongPipeline) log("Warning", WRONG_PIPELINE_WARNING);
        if (!fresh) {
            // Cached reads only: getStatus() is a blocking HTTP GET on the loop thread.
            boolean linked = limelight.isConnected();
            log("Cell", "NO VERDICT (%s, pipeline %s, staleness %.0fms)",
                    linked ? "linked" : "no link",
                    linked && result != null ? String.valueOf(result.getPipelineIndex()) : "?",
                    stalenessMs);
            return;
        }
        log("Cell", tipped ? "TIPPED" : "SCORABLE");
    }
}
