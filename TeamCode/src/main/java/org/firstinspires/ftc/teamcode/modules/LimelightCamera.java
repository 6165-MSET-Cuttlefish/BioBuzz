package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

import java.util.Arrays;

/**
 * Limelight 3A AprilTag subsystem: watches the four tags under one cell and reports whether that
 * cell has tipped over onto them.
 *
 * <p>Nothing here detects anything. The tag detection, the per-tag matching and the
 * "hidden long enough to count" timer all run on the Limelight, in
 * {@code limelight/cell_tip_snapscript.py}; the Control Hub only hands the script the four tag ids
 * it should watch ({@code llrobot}) and reads back its verdict ({@code llpython}). That keeps the
 * OpMode loop's share of this to two array copies, and it keeps the debounce on the device that
 * actually knows its own frame times — a hub loop running at 40 Hz cannot time a 0.25 s window
 * against a 90 fps camera without smearing it.
 *
 * <p>The script echoes the checksum of the tag set it is currently answering about, so a verdict
 * left over from the previously watched cell is never mistaken for one about this one.
 */
@Config
public class LimelightCamera extends Module {

    public static boolean limelightTelemetry = true;

    /** Index of the pipeline running {@code cell_tip_snapscript.py}. */
    public static int snapScriptPipeline = 0;
    /** All four of the watched cell's tags hidden this long ⇒ tipped. Timed on the Limelight. */
    public static double hiddenHoldSeconds = 0.25;
    /** A result older than this is ignored — the Limelight has stalled, rebooted or lost its link. */
    public static long maxStalenessMs = 250;
    /** Tag quads smaller than this (px²) are ignored on the Limelight; 0 keeps the script's default. */
    public static double minTagAreaPx = 0;
    /** Require the watched cell's tags to have been seen at least once before a tip can be reported. */
    public static boolean requireSeenBeforeTip = false;
    /** How long {@code RobotActions.checkTip} waits for a tip before giving up. */
    public static double checkTipTimeoutMs = 10000;
    /** {@code llrobot} is resent this often even when unchanged, so a Limelight reboot re-arms itself. */
    public static int resendInputsEveryNLoops = 25;

    private static final String DEFAULT_NAME = "limelight";

    // llrobot (hub → Limelight), 8 doubles.
    private static final int IN_TAG_0 = 0;
    private static final int IN_HOLD_SECONDS = 4;
    private static final int IN_ENABLED = 5;
    private static final int IN_MIN_TAG_AREA = 6;
    private static final int IN_REQUIRE_SEEN = 7;
    private static final int IN_LENGTH = 8;

    // llpython (Limelight → hub), 8 doubles.
    private static final int OUT_TIPPED = 0;
    private static final int OUT_VISIBLE_COUNT = 1;
    private static final int OUT_VISIBLE_MASK = 2;
    private static final int OUT_HIDDEN_SECONDS = 3;
    private static final int OUT_SEEN = 4;
    private static final int OUT_TAG_CHECKSUM = 5;
    private static final int OUT_DETECTED_COUNT = 6;
    private static final int OUT_FRAME_COUNTER = 7;
    private static final int OUT_LENGTH = 8;

    /**
     * The four AprilTags under each cell, as one set per cell.
     *
     * <p>Tag ids are a mirror of the game manual; the four sets are contiguous runs of four from 30.
     */
    public enum Cell {
        RED_1(30, 31, 32, 33),
        RED_2(34, 35, 36, 37),
        BLUE_1(38, 39, 40, 41),
        BLUE_2(42, 43, 44, 45);

        public final int[] tagIds;
        public final int checksum;

        Cell(int a, int b, int c, int d) {
            tagIds = new int[] {a, b, c, d};
            checksum = a + b + c + d;
        }
    }

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final Limelight3A limelight;
    private final String name;

    private Cell watchedCell;
    private double[] lastInputs;
    private int resendCounter;
    private int activePipeline = -1;

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
        if (limelight == null) return;
        selectPipeline();
        limelight.start();
    }

    @Override
    protected void read() {
        update();
    }

    /**
     * Pulls the latest verdict and pushes the current request. The framework calls this from
     * {@link #read()}; a bare {@code LinearOpMode} that skips the framework calls it itself, once
     * per loop, after {@link #init()}.
     */
    public void update() {
        if (limelight == null) return;
        selectPipeline();
        parse(limelight.getLatestResult());
        // llrobot is configuration for the Limelight's next frame rather than an actuator write, so
        // it is pumped from read(): the framework holds write() back until start(), and the tip
        // verdict has to be live on the DS during init.
        pushInputs();
    }

    @Override
    protected void write() {}

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    /** Points the script at {@code cell}'s four tags. The verdict stays false until it answers about them. */
    public void watch(Cell cell) {
        this.watchedCell = cell;
    }

    /** Stops watching; {@link #isTipped()} reads false and the Limelight stops matching tags. */
    public void stopWatching() {
        this.watchedCell = null;
    }

    public Cell getWatchedCell() {
        return watchedCell;
    }

    /**
     * Watches {@code cell} and returns whether it has tipped — false while any of its four tags is
     * in view, true once all four have been out of view for {@link #hiddenHoldSeconds}.
     */
    public boolean checkTip(Cell cell) {
        watch(cell);
        return isTipped();
    }

    /** Whether the watched cell has tipped. False whenever the answer isn't current and about that cell. */
    public boolean isTipped() {
        return fresh && tipped;
    }

    /** True once a current verdict about the watched cell is in hand — false means "don't know yet". */
    public boolean hasVerdict() {
        return fresh;
    }

    /** Whether any of the watched cell's tags has been seen since it was armed. */
    public boolean hasSeenCell() {
        return fresh && seenSinceArm;
    }

    /** How many of the watched cell's four tags are in the current frame. */
    public int getVisibleCount() {
        return fresh ? visibleCount : 0;
    }

    /** Bit {@code i} set ⇒ {@code watchedCell.tagIds[i]} is in the current frame. */
    public int getVisibleMask() {
        return fresh ? visibleMask : 0;
    }

    /** Seconds since any of the watched tags was last seen; 0 while one is visible. */
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
        Cell cell = watchedCell;

        fresh = cell != null
                && out != null
                && out.length >= OUT_LENGTH
                && stalenessMs <= maxStalenessMs
                && (int) Math.round(out[OUT_TAG_CHECKSUM]) == cell.checksum;
        if (!fresh) {
            tipped = false;
            seenSinceArm = false;
            visibleCount = 0;
            visibleMask = 0;
            hiddenSeconds = 0;
            detectedTagCount = 0;
            return;
        }

        tipped = out[OUT_TIPPED] != 0;
        visibleCount = (int) Math.round(out[OUT_VISIBLE_COUNT]);
        visibleMask = (int) Math.round(out[OUT_VISIBLE_MASK]);
        hiddenSeconds = out[OUT_HIDDEN_SECONDS];
        seenSinceArm = out[OUT_SEEN] != 0;
        detectedTagCount = (int) Math.round(out[OUT_DETECTED_COUNT]);
    }

    private void selectPipeline() {
        if (snapScriptPipeline == activePipeline) return;
        limelight.pipelineSwitch(snapScriptPipeline);
        activePipeline = snapScriptPipeline;
    }

    private void pushInputs() {
        double[] inputs = new double[IN_LENGTH];
        Cell cell = watchedCell;
        if (cell != null) {
            for (int i = 0; i < cell.tagIds.length; i++) inputs[IN_TAG_0 + i] = cell.tagIds[i];
        }
        inputs[IN_HOLD_SECONDS] = hiddenHoldSeconds;
        inputs[IN_ENABLED] = cell != null && !isInAny(VisionState.DISABLED) ? 1 : 0;
        inputs[IN_MIN_TAG_AREA] = minTagAreaPx;
        inputs[IN_REQUIRE_SEEN] = requireSeenBeforeTip ? 1 : 0;

        boolean changed = !Arrays.equals(inputs, lastInputs);
        if (!changed && ++resendCounter < Math.max(1, resendInputsEveryNLoops)) return;
        limelight.updatePythonInputs(inputs);
        lastInputs = inputs;
        resendCounter = 0;
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
        if (watchedCell == null) {
            logDashboard("Cell", "none watched");
        } else {
            logDashboard("Cell", "%s %s", watchedCell, Arrays.toString(watchedCell.tagIds));
        }
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
        if (!seenSinceArm) log("Tip", "cell never seen since arming");
    }
}
