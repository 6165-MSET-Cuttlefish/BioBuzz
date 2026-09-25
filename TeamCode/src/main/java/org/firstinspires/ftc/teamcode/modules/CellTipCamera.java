package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.modules.vision.CellTipPipeline;
import org.firstinspires.ftc.teamcode.modules.vision.WebcamSession;

/** HIVE-cell tip verdict from {@link CellTipPipeline} on the webcam; the alliance is fixed at init. */
@Config
public class CellTipCamera extends Module {

    public static long maxStalenessMs = 250;
    public static double checkTipTimeoutMs = 10000;

    private static final String[] CLUSTER_LABELS = {"SCORING", "AUDIENCE"};
    private static final int[][] RED_CLUSTERS = {{30, 31, 32, 33}, {34, 35, 36, 37}};
    private static final int[][] BLUE_CLUSTERS = {{42, 43, 44, 45}, {38, 39, 40, 41}};

    public enum VisionState implements State {
        ENABLED,
        DISABLED
    }

    private final HardwareMap hardwareMap;

    private AllianceColor alliance;
    private CellTipPipeline pipeline;
    private WebcamSession session;

    private boolean fresh;
    private boolean tipped;
    private double stalenessMs = Double.NaN;

    public CellTipCamera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    protected void initStates() {
        setStates(VisionState.ENABLED);
    }

    @Override
    public void init() {
        alliance = Context.allianceColor;
        pipeline = new CellTipPipeline(alliance.name(), CLUSTER_LABELS,
                alliance == AllianceColor.BLUE ? BLUE_CLUSTERS : RED_CLUSTERS);
        // Not in the constructor: open failures report through telemetry, which a Module only gets at init.
        session = new WebcamSession(hardwareMap, getTelemetry(), Camera.WEBCAM_NAME, pipeline);
    }

    @Override
    protected void read() {
        // In read(), not write(): exposure/gain tuning must apply during init, when writes are held back.
        session.update();
        CellTipPipeline.Verdict verdict = pipeline.latest();
        stalenessMs = verdict.timestampSeconds == 0
                ? Double.NaN
                : (System.nanoTime() * 1e-9 - verdict.timestampSeconds) * 1000;
        fresh = stalenessMs <= maxStalenessMs;
        tipped = fresh && verdict.tipped;
    }

    @Override
    protected void write() {
        pipeline.setEnabled(isInAny(VisionState.ENABLED));
    }

    @Override
    public void stop() {
        if (session != null) session.close();
        if (pipeline != null) pipeline.release();
    }

    /** Fixed at init from {@link Context#allianceColor}. */
    public AllianceColor getAlliance() {
        return alliance;
    }

    /** An empty frame also reads tipped, so a camera pointed away looks like a tipped cell. */
    public boolean isTipped() {
        return tipped;
    }

    public boolean isScorable() {
        return fresh && !tipped;
    }

    @Override
    protected void onTelemetry() {
        if (isInAny(VisionState.DISABLED)) {
            log("Cell", "OFF");
            return;
        }
        if (!fresh) {
            log("Cell", Double.isNaN(stalenessMs)
                    ? "NO VERDICT (no frames from " + Camera.WEBCAM_NAME + ")"
                    : String.format("NO VERDICT (staleness %.0fms)", stalenessMs));
            return;
        }
        log("Cell", tipped ? "TIPPED" : "SCORABLE");
    }
}
