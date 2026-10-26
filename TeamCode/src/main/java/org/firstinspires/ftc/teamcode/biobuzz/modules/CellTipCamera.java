package org.firstinspires.ftc.teamcode.biobuzz.modules;

import static org.firstinspires.ftc.teamcode.biobuzz.modules.vision.RobotStateHistory.nowSeconds;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.CellTipPipeline;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamControls;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession;

/** HIVE-cell tip verdict from {@link CellTipPipeline} on the webcam; the alliance is fixed at init. */
@Config("Cell Tip Camera")
public class CellTipCamera extends Module {

    public static final String WEBCAM_NAME = "aprilTagDetector";
    public static final String FAULT_KEY = "Webcam";

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

    public static CellTipPipeline newPipeline(AllianceColor alliance) {
        return new CellTipPipeline(alliance.name(), CLUSTER_LABELS,
                alliance == AllianceColor.BLUE ? BLUE_CLUSTERS : RED_CLUSTERS);
    }

    @Override
    protected void initStates() {
        setStates(VisionState.ENABLED);
    }

    @Override
    public void init() {
        alliance = Context.allianceColor;
        pipeline = newPipeline(alliance);
        session = new WebcamSession(hardwareMap, WEBCAM_NAME, pipeline);
    }

    @Override
    protected void read() {
        // In read(), not write(): exposure/gain tuning and the on/off state must apply during init, when writes are held back.
        pipeline.setEnabled(isInAny(VisionState.ENABLED));
        session.update();
        WebcamControls controls = session.controls();
        String rejected = controls == null ? null : controls.rejected();
        if (rejected == null) Faults.clear(FAULT_KEY);
        else Faults.raise(FAULT_KEY, WEBCAM_NAME + " rejected " + rejected);
        CellTipPipeline.Verdict verdict = pipeline.latest();
        stalenessMs = verdict.timestampSeconds == 0
                ? Double.NaN
                : (nowSeconds() - verdict.timestampSeconds) * 1000;
        fresh = stalenessMs >= 0 && stalenessMs <= maxStalenessMs;
        tipped = fresh && verdict.tipped;
    }

    @Override
    protected void write() {}

    @Override
    public void stop() {
        try {
            if (session != null) session.close();
        } finally {
            if (pipeline != null) pipeline.release();
        }
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
                    ? "NO VERDICT (no frames from " + WEBCAM_NAME + ")"
                    : String.format("NO VERDICT (staleness %.0fms)", stalenessMs));
            return;
        }
        log("Cell", tipped ? "TIPPED" : "SCORABLE");
    }
}
