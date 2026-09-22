package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

import java.util.Arrays;

/**
 * Limelight 3A AprilTag subsystem: watches the four tags under one cell and reports whether that
 * cell has tipped over onto them.
 *
 * <p>Nothing here detects anything and nothing here is configured at run time. Each cell has its
 * own Limelight pipeline holding its own copy of the SnapScript with that cell's four tag ids baked
 * in ({@code limelight/pipelines/}), so the hub's whole job is to select the pipeline named by
 * {@link Context#cell} once at {@link #init()} and then read the verdict. The link carries no
 * {@code llrobot} traffic at all — one array read per loop, nothing written.
 *
 * <p>Because the cell is fixed at init, {@link Context#cell} has to be set before the OpMode
 * initializes (in {@code createRobot()}, like the alliance colour); changing it later is ignored.
 * The script echoes the checksum of the tag set it was built with, and a verdict whose checksum
 * doesn't match {@link Context#cell} is discarded — that is what catches a pipeline index pointing
 * at the wrong script.
 */
@Config
public class LimelightCamera extends Module {

    public static boolean limelightTelemetry = true;

    /** A result older than this is ignored — the Limelight has stalled, rebooted or lost its link. */
    public static long maxStalenessMs = 250;
    /** How long {@code RobotActions.checkTip} waits for a tip before giving up. */
    public static double checkTipTimeoutMs = 10000;

    private static final String DEFAULT_NAME = "limelight";

    // llpython (Limelight → hub), 8 doubles. Mirrored in every limelight/pipelines/ script.
    private static final int OUT_TIPPED = 0;
    private static final int OUT_VISIBLE_COUNT = 1;
    private static final int OUT_VISIBLE_MASK = 2;
    private static final int OUT_HIDDEN_SECONDS = 3;
    private static final int OUT_SEEN = 4;
    private static final int OUT_TAG_CHECKSUM = 5;
    private static final int OUT_DETECTED_COUNT = 6;
    private static final int OUT_FRAME_COUNTER = 7;
    private static final int OUT_LENGTH = 8;

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final Limelight3A limelight;
    private final String name;

    private Context.Cell cell;
    private boolean polling;

    private boolean fresh;
    private boolean tipped;
    private boolean seenSinceArm;
    private int visibleCount;
    private int visibleMask;
    private double hiddenSeconds;
    private int detectedTagCount;
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
        cell = Context.cell;
        if (limelight == null) return;
        limelight.pipelineSwitch(cell.pipeline);
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

    /** The cell this OpMode is watching, fixed at init from {@link Context#cell}. */
    public Context.Cell getCell() {
        return cell;
    }

    /**
     * Whether the cell has tipped — false while any of its four tags is in view, true once all four
     * have been out of view for the hold time baked into the pipeline's script. Also false whenever
     * the answer isn't current or isn't about {@link Context#cell}.
     */
    public boolean isTipped() {
        return fresh && tipped;
    }

    /** Alias of {@link #isTipped()}, for robot code that reads better as a check than a state query. */
    public boolean checkTip() {
        return isTipped();
    }

    /** True once a current verdict is in hand — false means "don't know yet". */
    public boolean hasVerdict() {
        return fresh;
    }

    /** Whether any of the cell's tags has been seen since the pipeline started. */
    public boolean hasSeenCell() {
        return fresh && seenSinceArm;
    }

    /** How many of the cell's four tags are in the current frame. */
    public int getVisibleCount() {
        return fresh ? visibleCount : 0;
    }

    /** Bit {@code i} set ⇒ {@code getCell().tagIds[i]} is in the current frame. */
    public int getVisibleMask() {
        return fresh ? visibleMask : 0;
    }

    /** Seconds since any of the cell's tags was last seen; 0 while one is visible. */
    public double getHiddenSeconds() {
        return fresh ? hiddenSeconds : 0;
    }

    /** AprilTags of any id in the current frame — a sanity check that the pipeline sees the field at all. */
    public int getDetectedTagCount() {
        return fresh ? detectedTagCount : 0;
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

    private void parse(LLResult result) {
        double[] out = result == null ? null : result.getPythonOutput();
        stalenessMs = result == null ? Double.NaN : result.getStaleness();

        fresh = cell != null
                && out != null
                && out.length >= OUT_LENGTH
                && stalenessMs <= maxStalenessMs
                && (int) Math.round(out[OUT_TAG_CHECKSUM]) == cell.checksum;
        if (!fresh) {
            clearVerdict();
            return;
        }

        tipped = out[OUT_TIPPED] != 0;
        visibleCount = (int) Math.round(out[OUT_VISIBLE_COUNT]);
        visibleMask = (int) Math.round(out[OUT_VISIBLE_MASK]);
        hiddenSeconds = out[OUT_HIDDEN_SECONDS];
        seenSinceArm = out[OUT_SEEN] != 0;
        detectedTagCount = (int) Math.round(out[OUT_DETECTED_COUNT]);
    }

    private void clearVerdict() {
        fresh = false;
        tipped = false;
        seenSinceArm = false;
        visibleCount = 0;
        visibleMask = 0;
        hiddenSeconds = 0;
        detectedTagCount = 0;
    }

    private String maskString() {
        char[] flags = new char[4];
        for (int i = 0; i < flags.length; i++) flags[i] = ((visibleMask >> i) & 1) == 1 ? 'X' : '.';
        return new String(flags);
    }

    @Override
    protected void onTelemetry() {
        if (!limelightTelemetry) return;
        if (limelight == null) {
            log("Limelight", "NOT CONFIGURED (\"%s\")", name);
            return;
        }
        logDashboard("Cell", "%s pipeline %d %s", cell, cell.pipeline, Arrays.toString(cell.tagIds));
        if (!fresh) {
            LLStatus status = limelight.getStatus();
            log("Tip", "NO VERDICT (pipeline %d, %.0ffps, staleness %.0fms)",
                    status.getPipelineIndex(), status.getFps(), stalenessMs);
            return;
        }
        logDashboard("Tip", tipped ? "TIPPED" : "upright");
        logDashboard("Tags visible", "%d/4 %s", visibleCount, maskString());
        logDashboard("Hidden", "%.2fs", hiddenSeconds);
        log("Tags in frame", detectedTagCount);
        if (!seenSinceArm) log("Tip", "cell never seen since the pipeline started");
    }
}
