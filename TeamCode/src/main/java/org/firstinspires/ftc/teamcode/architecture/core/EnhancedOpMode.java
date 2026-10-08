package org.firstinspires.ftc.teamcode.architecture.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.PoseRing;
import org.firstinspires.ftc.teamcode.architecture.telemetry.LoopProfiler;
import org.firstinspires.ftc.teamcode.architecture.hardware.BatteryVoltage;
import org.firstinspires.ftc.teamcode.architecture.input.InputClock;
import org.firstinspires.ftc.teamcode.architecture.telemetry.FieldMapRenderer;

import static org.firstinspires.ftc.teamcode.architecture.core.Robot.telemetryToggles;
import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.*;

/** Base OpMode with auto-discovered modules, voltage compensation, dual telemetry, and dashboard field rendering. Subclasses provide a {@link Robot} via {@link #createRobot()}. */
public abstract class EnhancedOpMode extends OpMode {
    private static final int FIELD_RENDER_INTERVAL = 10;
    // The voltage read is a bus call costing a few ms; battery voltage moves slowly.
    private static final int VOLTAGE_READ_LOOP_INTERVAL = 50;

    protected Robot robot;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime gameTimer = new ElapsedTime();
    private final double[] loopTimes = new double[20];
    private final LoopProfiler profiler = new LoopProfiler();
    private final PoseRing poseHistory = new PoseRing(30);
    private TelemetryPacket packet;
    private List<LynxModule> lynxHubs;
    private int loopIndex = 0;
    private long monotonicLoopCount = 0;
    private long profiledLoopCount = 0;
    private double profiledLoopSumMs = 0;
    private double profiledLoopMaxMs = 0;
    private int voltageLoopCounter = 0;

    private final List<Module> modules = new ArrayList<>();
    private long lastCurrentReadLoop = Long.MIN_VALUE;
    private int initializedModuleCount = 0;
    // Modules whose initStates() has been entered; only these can have commanded hardware.
    private int initStartedModuleCount = 0;
    private double cachedTotalCurrent = 0.0;
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private int loopDumpCounter = 0;
    private String cachedLoopDump = "";

    private FieldMapRenderer field;
    private String cachedFieldHtml = "";
    private double lastFieldRenderX = Double.NaN;
    private double lastFieldRenderY = Double.NaN;
    private double lastFieldRenderHeading = Double.NaN;
    private int loopsSinceFieldRender = Integer.MAX_VALUE;
    private AllianceColor cachedAllianceColor;
    private String cachedAllianceHtml;

    protected void initialize() {}
    protected void initializeLoop() {}
    protected void gameLoop() {}
    protected void onStart() {}
    protected void onEnd() {}
    protected void telemetry() {}

    protected boolean shouldWriteDuringInit() { return false; }
    protected boolean shouldReadDuringInit() { return true; }

    protected abstract Robot createRobot();

    /** Called at the top of every init_loop and loop, before module reads. */
    protected void onLoopStart() {}

    protected void dashboardOverlay(Canvas overlay) {}

    // Once a hook throws, the SDK calls no further hooks (stop() included), so each one runs the safe-state pass itself before rethrowing.
    @Override
    public final void init() {
        try {
            runInit();
        } catch (Throwable t) {
            reportFailure("init", t);
            safeStatePass(t);
            throw unchecked(t);
        }
    }

    private void runInit() {
        // Statics outlive an OpMode; Ivy's Scheduler even survives a Sloth reload.
        State.clearModuleBindings();
        Faults.reset();
        Scheduler.reset();
        FieldConfig.clear();

        configureBulkCaching();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        robot = createRobot();
        robot.telemetry.setEnabled(telemetryToggles.dsTelemetry, telemetryToggles.dashboardTelemetry);
        telemetry = robot.telemetry;
        packet = newPacket();
        robot.telemetry.setPacket(packet);

        autoDiscoverModules();
        initModules();
        initialize();
        // Second pass picks up modules created during initialize().
        autoDiscoverModules();
        initModules();

        loopTimer.reset();
        gameTimer.reset();

        field = new FieldMapRenderer(73, 74);
        field.drawFieldLayout();
        field.snapshot();
    }

    @Override
    public final void init_loop() {
        try {
            runInitLoop();
        } catch (Throwable t) {
            reportFailure("init_loop", t);
            safeStatePass(t);
            throw unchecked(t);
        }
    }

    private void runInitLoop() {
        runPipelineHead(shouldReadDuringInit());

        initializeLoop();
        profiler.mark("initializeLoop");

        // Init-scoped: start() calls Scheduler.reset(), so anything scheduled during init is dropped without its end() running.
        Scheduler.execute();
        profiler.mark("commands");

        // 500 ms grace before the first write so SDK mode/direction calls settle.
        if (shouldWriteDuringInit() && gameTimer.milliseconds() > 500) {
            writeModules();
        }
        profiler.mark("writeModules");

        updateTelemetry();
        profiler.mark("updateTelemetry");

        updateDashboard();
        profiler.mark("updateDashboard");

        recordLoopTime();
        loopTimer.reset();
    }

    @Override
    public final void start() {
        try {
            runStart();
        } catch (Throwable t) {
            reportFailure("start", t);
            safeStatePass(t);
            throw unchecked(t);
        }
    }

    private void runStart() {
        gameTimer.reset();
        clearBulkCaches();
        // Reset throttle counters so the first match loop samples fresh instead of inheriting the init loop's mid-cycle phase.
        voltageLoopCounter = 0;
        loopsSinceFieldRender = Integer.MAX_VALUE;
        lastCurrentReadLoop = Long.MIN_VALUE;
        // Drop init-phase timings so the first match loops don't average them into loop time.
        for (int i = 0; i < loopTimes.length; i++) loopTimes[i] = 0.0;
        loopIndex = 0;
        profiledLoopCount = 0;
        profiledLoopSumMs = 0;
        profiledLoopMaxMs = 0;
        profiler.reset();
        Scheduler.reset();
        scheduleStartupCommands();
        onStart();
        loopTimer.reset();
    }

    @Override
    public final void loop() {
        try {
            runLoop();
        } catch (Throwable t) {
            reportFailure("loop", t);
            safeStatePass(t);
            throw unchecked(t);
        }
    }

    private void runLoop() {
        runPipelineHead(true);

        gameLoop();
        profiler.mark("gameLoop");

        // Run between user code and writes so command-applied state lands in the same write pass.
        Scheduler.execute();
        profiler.mark("commands");

        writeModules();
        profiler.mark("writeModules");

        updateTelemetry();
        profiler.mark("updateTelemetry");

        updateDashboard();
        profiler.mark("updateDashboard");

        recordLoopTime();
        loopTimer.reset();
    }

    @Override
    public final void stop() {
        Throwable failure = safeStatePass(null);
        failure = attempt(failure, this::onEnd);
        if (failure != null) {
            reportFailure("stop", failure);
            throw unchecked(failure);
        }
    }

    /** Every step runs even if an earlier one throws; returns the first failure, later ones attached to it as suppressed. */
    private Throwable safeStatePass(Throwable failure) {
        // reset() drops every command without running its end() hook, so hardware safe-state must live in Module.stop().
        failure = attempt(failure, Scheduler::reset);
        if (robot != null) {
            // Follower.stop() only changes mode; Pedro writes its drive motors on the next update(), which never comes.
            failure = attempt(failure, () -> robot.follower.stop());
            failure = attempt(failure, () -> robot.follower.drivetrain.stop());
        }
        for (int i = 0; i < initStartedModuleCount; i++) {
            Module m = modules.get(i);
            failure = attempt(failure, m::stop);
        }
        return failure;
    }

    private void reportFailure(String hook, Throwable t) {
        if (t instanceof OpModeManagerImpl.ForceStopException) return;
        RobotLog.setGlobalErrorMsg(getClass().getSimpleName() + " " + hook + ": " + t);
    }

    private static RuntimeException unchecked(Throwable t) {
        return t instanceof RuntimeException ? (RuntimeException) t : new RuntimeException(t);
    }

    private static Throwable attempt(Throwable failure, Runnable step) {
        try {
            step.run();
        } catch (Throwable t) {
            if (failure == null) return t;
            if (t != failure) failure.addSuppressed(t);
        }
        return failure;
    }

    private void runPipelineHead(boolean read) {
        robot.telemetry.setEnabled(telemetryToggles.dsTelemetry, telemetryToggles.dashboardTelemetry);
        robot.telemetry.syncDsTransmissionInterval();
        robot.telemetry.beginLoop();
        // Here, before any other line, so faults lead the frame; ones raised later this loop show next loop.
        addFaultTelemetry();

        profiler.enabled = profilerEnabled;
        profiler.start();

        clearBulkCaches();
        profiler.mark("clearBulkCaches");

        InputClock.advance();

        updateVoltageThrottled();
        BatteryVoltage.update(voltage);
        profiler.mark("voltage");

        onLoopStart();
        profiler.mark("onLoopStart");

        if (read) readModules();
        profiler.mark("readModules");

        robot.follower.update();
        poseHistory.record(robot.follower.pose());
        profiler.mark("follower.update");
    }

    private void autoDiscoverModules() {
        // Identity-based so user classes that override equals/hashCode don't collide.
        Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        try {
            discover(this, getClass(), visited);
            // discover() stops above EnhancedOpMode, so seed the inherited robot field directly.
            discoverValue(robot, visited);
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Module auto-discovery failed", e);
        }
    }

    private void discover(Object obj, Class<?> clazz, Set<Object> visited)
            throws IllegalAccessException {
        if (obj == null || !visited.add(obj)) return;

        while (clazz != EnhancedOpMode.class && clazz != Object.class && clazz != null) {
            for (Field f : clazz.getDeclaredFields()) {
                f.setAccessible(true);
                discoverValue(f.get(obj), visited);
            }
            clazz = clazz.getSuperclass();
        }
    }

    private void discoverValue(Object val, Set<Object> visited) throws IllegalAccessException {
        if (val == null) return;

        if (val instanceof Module) {
            if (!modules.contains(val)) modules.add((Module) val);
        } else if (val instanceof Object[]) {
            for (Object e : (Object[]) val) discoverValue(e, visited);
        } else if (val instanceof Iterable<?>) {
            for (Object e : (Iterable<?>) val) discoverValue(e, visited);
        } else if (val instanceof Map<?, ?>) {
            for (Map.Entry<?, ?> e : ((Map<?, ?>) val).entrySet()) {
                discoverValue(e.getKey(), visited);
                discoverValue(e.getValue(), visited);
            }
        } else if (shouldRecurse(val.getClass())) {
            discover(val, val.getClass(), visited);
        }
    }

    // Whitelist, not blacklist: every Module lives in our package, and descending into library objects
    // (slothboard's DashboardCore, Pedro's Follower) walks lock-guarded lists without the lock.
    private boolean shouldRecurse(Class<?> c) {
        return c.getName().startsWith("org.firstinspires.ftc.teamcode.");
    }

    private void initModules() {
        // Idempotent via initializedModuleCount: this runs twice (before and after initialize()) and must not re-init a module.
        for (int i = initializedModuleCount; i < modules.size(); i++) {
            Module m = modules.get(i);
            initStartedModuleCount = i + 1;
            m.setTelemetry(robot.telemetry);
            m.initStates();
            m.init();
        }
        initializedModuleCount = modules.size();
    }

    private void readModules() {
        for (int i = 0; i < modules.size(); i++) {
            Module m = modules.get(i);
            m.refreshTunables();
            long t = profiler.enterSection();
            m.read();
            profiler.leaveSection(m.getReadSectionName(), t);
        }
    }

    private void writeModules() {
        for (int i = 0; i < modules.size(); i++) {
            Module m = modules.get(i);
            if (m.isWriteEnabled()) {
                long t = profiler.enterSection();
                m.write();
                profiler.leaveSection(m.getWriteSectionName(), t);
            }
        }
    }

    // Ivy cannot enumerate running commands: a startup command that must yield to one already holding
    // its module should require that module with BlockedBehavior/ConflictBehavior CANCEL.
    private void scheduleStartupCommands() {
        for (Module m : modules) {
            Command startup = m.getStartupCommand();
            if (startup != null) Scheduler.schedule(startup);
        }
    }

    private void configureBulkCaching() {
        lynxHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : lynxHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCaches() {
        for (int i = 0; i < lynxHubs.size(); i++) lynxHubs.get(i).clearBulkCache();
    }

    private void updateVoltageThrottled() {
        if (voltageLoopCounter == 0) voltage = voltageSensor.getVoltage();
        voltageLoopCounter = (voltageLoopCounter + 1) % VOLTAGE_READ_LOOP_INTERVAL;
    }

    private void recordLoopTime() {
        double ms = loopTimer.milliseconds();
        loopTimes[loopIndex] = ms;
        loopIndex = (loopIndex + 1) % loopTimes.length;
        monotonicLoopCount++;
        profiledLoopCount++;
        profiledLoopSumMs += ms;
        if (ms > profiledLoopMaxMs) profiledLoopMaxMs = ms;
    }

    private void updateTelemetry() {
        boolean dsFrame = robot.telemetry.isDSFrame();
        if (dsFrame || telemetryToggles.dashboardTelemetry) {
            Pose currentPose = robot.follower.pose();
            addStatusTelemetry(currentPose);

            robot.telemetry.addDashboardData("Game Time", "%.1fs", gameTimer.seconds());
            robot.telemetry.addData("Loop Time", "%.1fms (avg %.1fms)", lastLoopMs(), avgLoopMs());

            if (!modules.isEmpty()) {
                robot.telemetry.addSeparator();
                robot.telemetry.addGroupHeader("MODULES", COLOR_MODULE);
                for (int i = 0; i < modules.size(); i++) modules.get(i).telemetry();
            }

            if (telemetryToggles.loopProfile) {
                robot.telemetry.addSeparator();
                robot.telemetry.addGroupHeader("LOOP PROFILE (avg ms)", COLOR_BLUE);
                List<Map.Entry<String, Double>> snapshot = profiler.snapshotSortedDesc();
                for (int i = 0; i < snapshot.size(); i++) {
                    Map.Entry<String, Double> entry = snapshot.get(i);
                    robot.telemetry.addDashboardData(entry.getKey(), "%.2fms", entry.getValue());
                }
                // Rebuilt rarely: ~60 String.formats, and being cumulative it barely moves loop to loop.
                if ((loopDumpCounter++ % 25) == 0) {
                    double loopAvg = profiledLoopCount == 0 ? 0 : profiledLoopSumMs / profiledLoopCount;
                    cachedLoopDump = profiler.report(profiledLoopCount, loopAvg, profiledLoopMaxMs);
                }
                robot.telemetry.addDashboardData("LOOP_DUMP", cachedLoopDump);
            }

            if (dsFrame) renderFieldMap(currentPose);
        }

        telemetry();
        robot.telemetry.update();
    }

    private void renderFieldMap(Pose fieldPose) {
        double fx = fieldPose.x();
        double fy = fieldPose.y();
        double fh = fieldPose.heading();
        boolean poseChanged = Math.abs(fx - lastFieldRenderX) > 0.5
                || Math.abs(fy - lastFieldRenderY) > 0.5
                || Math.abs(fh - lastFieldRenderHeading) > Math.toRadians(2);
        if (poseChanged || loopsSinceFieldRender >= FIELD_RENDER_INTERVAL) {
            field.restore();
            field.drawRobot(fx, fy, fh,
                    Context.allianceColor.equals(AllianceColor.RED) ? COLOR_RED : COLOR_BLUE);
            cachedFieldHtml = htmlSize(FONT_SMALL, field.renderHtml());
            lastFieldRenderX = fx;
            lastFieldRenderY = fy;
            lastFieldRenderHeading = fh;
            loopsSinceFieldRender = 0;
        } else {
            loopsSinceFieldRender++;
        }
        robot.telemetry.addDSLine(cachedFieldHtml);
    }

    // No markTelemetryFrame(): a packet carrying only drawings must not blank the telemetry view.
    private TelemetryPacket newPacket() {
        TelemetryPacket p = new TelemetryPacket(false);
        p.setDisplayFormat(TelemetryPacket.DisplayFormat.HTML);
        p.setCaptionValueSeparator(": ");
        return p;
    }

    private void updateDashboard() {
        // Thread.sleep() on slothboard's sender thread throws on a negative interval and kills dashboard telemetry silently.
        if (dashboardTransmissionIntervalMs < 0) {
            throw new IllegalArgumentException("OptimizationToggles.dashboardTransmissionIntervalMs must be >= 0, was "
                    + dashboardTransmissionIntervalMs);
        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        // Re-applied every loop: slothboard resets it to its default at every OpMode init.
        dashboard.setTelemetryTransmissionInterval(dashboardTransmissionIntervalMs);

        Canvas overlay = packet.fieldOverlay();

        String image = FieldConfig.image();
        double inches = image != null ? FieldConfig.imageInches() : FieldConfig.fieldWidthInches;
        // The page frame runs down from its top-left corner; Pedro's origin is the field's bottom-left one.
        double top = 144 - inches;
        if (!dashboardSkipFieldImage && image != null) {
            overlay.setAlpha(FieldConfig.imageOpacity());
            overlay.drawImage(image, 0, top, inches, inches);
            overlay.setAlpha(1);
        }

        if (!dashboardSkipGrid) overlay.drawGrid(0, top, inches, inches, 7, 7);

        FieldVisualization.drawRobot(overlay, robot.follower.pose());

        if (!dashboardSkipPoseHistory) {
            FieldVisualization.drawPoseHistory(overlay, poseHistory);
        }

        dashboardOverlay(overlay);
        drawFaultOverlay(overlay);

        // Swap before queueing: slothboard serializes queued packets on its own thread while telemetry writes go on (SDK post-loop update() too).
        TelemetryPacket queued = packet;
        packet = newPacket();
        robot.telemetry.setPacket(packet);
        dashboard.sendTelemetryPacket(queued);
    }

    private void addFaultTelemetry() {
        List<Faults.Fault> faults = Faults.active();
        for (int i = 0; i < faults.size(); i++) {
            Faults.Fault f = faults.get(i);
            robot.telemetry.addFault(f.key, f.message);
        }
    }

    private static void drawFaultOverlay(Canvas overlay) {
        List<Faults.Fault> faults = Faults.active();
        if (faults.isEmpty()) return;
        overlay.setAlpha(1);
        overlay.setFill(COLOR_FAULT);
        for (int i = 0; i < faults.size(); i++) {
            // Page frame: 144 x 144 from the top-left corner, y down, untouched by setTranslation/setRotation.
            overlay.fillText("FAULT " + faults.get(i), 2, 7 + 6 * i, "bold 5px sans-serif", 0, true);
        }
    }

    private void addStatusTelemetry(Pose currentPose) {
        robot.telemetry.addGroupHeader("ROBOT STATUS");
        addAllianceTelemetry();
        robot.telemetry.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°",
                currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
        addVoltageCurrentTelemetry();
    }

    private void addAllianceTelemetry() {
        if (Context.allianceColor != cachedAllianceColor) {
            cachedAllianceColor = Context.allianceColor;
            String colorHex = Context.allianceColor == AllianceColor.RED ? COLOR_RED : COLOR_BLUE;
            cachedAllianceHtml = htmlColor(colorHex, htmlBold(String.valueOf(Context.allianceColor)));
        }
        robot.telemetry.addRawHtml("Alliance", cachedAllianceHtml);
    }

    private void addVoltageCurrentTelemetry() {
        if (telemetryToggles.voltage) robot.telemetry.addDashboardData("Voltage", "%.2fV", voltage);
        if (telemetryToggles.current && telemetryToggles.dashboardTelemetry) {
            robot.telemetry.addDashboardData("Current", "%.2fA", getTotalCurrent());
        }
    }

    private double lastLoopMs() {
        return loopTimes[(loopIndex + loopTimes.length - 1) % loopTimes.length];
    }

    private double avgLoopMs() {
        double sum = 0;
        int count = 0;
        for (double t : loopTimes) {
            if (t > 0) {
                sum += t;
                count++;
            }
        }
        return count > 0 ? sum / count : 0;
    }

    private double getTotalCurrent() {
        int every = Math.max(1, currentReadEveryNLoops);
        // Throttle on loop count; this is only called on render loops.
        if (lastCurrentReadLoop == Long.MIN_VALUE || monotonicLoopCount - lastCurrentReadLoop >= every) {
            double total = 0;
            for (LynxModule hub : lynxHubs) total += hub.getCurrent(CurrentUnit.AMPS);
            cachedTotalCurrent = total;
            lastCurrentReadLoop = monotonicLoopCount;
        }
        return cachedTotalCurrent;
    }

    public final LoopProfiler getProfiler() { return profiler; }

    protected final void raiseFault(String key, String message) { Faults.raise(key, message); }
    protected final void clearFault(String key) { Faults.clear(key); }
    public final boolean hasFault(String key) { return Faults.has(key); }
    public final List<Faults.Fault> activeFaults() { return Faults.active(); }
}
