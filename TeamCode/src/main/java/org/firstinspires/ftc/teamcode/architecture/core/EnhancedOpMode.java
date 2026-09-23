package org.firstinspires.ftc.teamcode.architecture.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.telemetry.LoopProfiler;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedCRServo;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.input.InputClock;
import org.firstinspires.ftc.teamcode.architecture.telemetry.FieldMapRenderer;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;
import org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter;

import static org.firstinspires.ftc.teamcode.architecture.core.Robot.telemetryToggles;
import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.*;

/** Base OpMode with auto-discovered modules, voltage compensation, dual telemetry, and dashboard field rendering. Subclasses provide a {@link Robot} via {@link #createRobot()}. */
public abstract class EnhancedOpMode extends OpMode {
    protected Robot robot;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime gameTimer = new ElapsedTime();
    private final ElapsedTime readWriteTimer = new ElapsedTime();
    private final double[] loopTimes = new double[20];
    private final LoopProfiler profiler = new LoopProfiler();
    private List<LynxModule> lynxHubs;
    private int loopIndex = 0;
    private long monotonicLoopCount = 0;
    private long profiledLoopCount = 0;
    private double profiledLoopSumMs = 0;
    private double profiledLoopMaxMs = 0;
    private int voltageLoopCounter = 0;
    private int dashboardLoopCounter = 0;
    private int telemetryLoopCounter = 0;

    private final List<Module> modules = new ArrayList<>();
    private List<Module> sortedTelemetryModules;
    private long lastCurrentReadLoop = Long.MIN_VALUE;
    private int initializedModuleCount = 0;
    private double cachedTotalCurrent = 0.0;
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private boolean running = false;
    private boolean stopRequested = false;
    private boolean isInit = false;
    private boolean telemetryRenderedThisLoop = false;
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
    /** Re-render the DS field map at most every N loops even when the pose hasn't moved. */
    protected int fieldRenderInterval = 10;

    /** Hold each loop to at least this many ms (0 = no holding). */
    protected int minLoopMs = 0;
    /** Loops between voltage reads. Bus call costs a few ms; voltage moves slowly. */
    protected int voltageReadLoopInterval = 50;
    protected boolean voltageCompensationEnabled = true;

    protected void initialize() {}
    protected void initializeLoop() {}
    protected void gameLoop() {}
    protected void onStart() {}
    protected void onEnd() {}
    protected void telemetry() {}

    protected boolean shouldWriteDuringInit() { return false; }
    protected boolean shouldReadDuringInit() { return true; }

    protected abstract Robot createRobot() throws InterruptedException;

    /** Called at the top of every init_loop and loop, before module reads. */
    protected void onLoopStart() {}

    /**
     * Draw game-specific field overlay. Called only on loops where the packet is actually SENT — every
     * dashboard frame is standalone, so anything drawn on a subset of sent frames flickers. Draw here
     * rather than into {@code robot.packet} from game code, which cannot know if the packet is sent.
     */
    protected void dashboardOverlay(Canvas overlay) {}

    @Override
    public final void init() {
        // Statics: drop stale State→Module bindings and commands from prior runs before modules instantiate.
        // Ivy's Scheduler is a static in a library class, so it survives Sloth hot-reloads and back-to-back OpModes.
        State.clearModuleBindings();
        Scheduler.reset();

        configureBulkCaching();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        try {
            robot = createRobot();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry = robot.telemetry;
        robot.packet = newPacket();

        autoDiscoverModules();
        initModules();
        initialize();
        // Second pass picks up modules created during initialize(); sort once all modules are known.
        autoDiscoverModules();
        initModules();
        if (telemetrySortModulesOnce) buildSortedTelemetryModules();

        loopTimer.reset();
        gameTimer.reset();
        isInit = true;

        field = new FieldMapRenderer(73, 74);
        field.drawFieldLayout();
        field.snapshot();
    }

    @Override
    public final void init_loop() {
        if (!isInit) return;
        if (stopRequested) {
            requestOpModeStop();
            return;
        }

        profiler.enabled = profilerEnabled;
        profiler.start();

        clearBulkCaches();
        profiler.mark("clearBulkCaches");

        InputClock.advance();

        if (voltageCompensationEnabled) {
            updateVoltageThrottled();
            EnhancedMotor.updateVoltage(voltage);
            EnhancedCRServo.updateVoltage(voltage);
        }
        profiler.mark("voltage");

        onLoopStart();
        profiler.mark("onLoopStart");

        if (shouldReadDuringInit()) readModules();
        profiler.mark("readModules");

        robot.follower.update();
        robot.poseHistory.record(robot.follower.pose());
        profiler.mark("follower.update");

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

        loopTimer.reset();
    }

    @Override
    public final void start() {
        running = true;
        gameTimer.reset();
        clearBulkCaches();
        // Reset throttle counters so the first match loop samples fresh instead of inheriting the init loop's mid-cycle phase.
        voltageLoopCounter = 0;
        telemetryLoopCounter = 0;
        dashboardLoopCounter = 0;
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
        if (stopRequested) {
            running = false;
            requestOpModeStop();
            return;
        }

        profiler.enabled = profilerEnabled;
        profiler.start();

        clearBulkCaches();
        profiler.mark("clearBulkCaches");

        InputClock.advance();

        if (voltageCompensationEnabled) {
            updateVoltageThrottled();
            EnhancedMotor.updateVoltage(voltage);
            EnhancedCRServo.updateVoltage(voltage);
        }
        profiler.mark("voltage");

        onLoopStart();
        profiler.mark("onLoopStart");

        readModules();
        profiler.mark("readModules");

        robot.follower.update();
        robot.poseHistory.record(robot.follower.pose());
        profiler.mark("follower.update");

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

        long remaining = minLoopMs - (long) loopTimer.milliseconds();
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        recordLoopTime();
        loopTimer.reset();
    }

    @Override
    public final void stop() {
        running = false;
        // reset() drops every command without running its end() hook, so hardware safe-state must live in Module.stop().
        Scheduler.reset();
        // Safe-state pass: every module must get its stop() even if an earlier one throws (the SDK does not auto-zero motors on stop); the first Throwable is rethrown afterwards so the failure still surfaces.
        Throwable first = null;
        for (int i = 0; i < modules.size(); i++) {
            try {
                modules.get(i).stop();
            } catch (Throwable t) {
                if (first == null) first = t;
            }
        }
        try {
            onEnd();
        } catch (Throwable t) {
            if (first == null) first = t;
        }
        if (first != null) {
            if (first instanceof RuntimeException) throw (RuntimeException) first;
            throw (Error) first;
        }
    }

    /** Manually register modules; reflection auto-discovery picks up the rest. */
    protected void register(Module... mods) {
        for (Module m : mods) {
            if (!modules.contains(m)) modules.add(m);
        }
    }

    private void autoDiscoverModules() {
        // Identity-based so user classes that override equals/hashCode don't collide.
        Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        try {
            discover(this, getClass(), visited);
            // discover() stops above EnhancedOpMode, so the inherited `robot` field is not a discovery root — seed from it directly (idempotent via the visited set).
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
            register((Module) val);
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
            m.setTelemetry(telemetry);
            m.initStates();
            m.init();
        }
        initializedModuleCount = modules.size();
    }

    private List<Module> telemetryOrderedModules() {
        if (telemetrySortModulesOnce) {
            if (sortedTelemetryModules == null) buildSortedTelemetryModules();
            return sortedTelemetryModules;
        }
        List<Module> ordered = new ArrayList<>(modules);
        Collections.sort(ordered, (a, b) -> Integer.compare(a.telemetryOrder(), b.telemetryOrder()));
        return ordered;
    }

    private void buildSortedTelemetryModules() {
        List<Module> ordered = new ArrayList<>(modules);
        Collections.sort(ordered, (a, b) -> Integer.compare(a.telemetryOrder(), b.telemetryOrder()));
        sortedTelemetryModules = ordered;
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

    protected void writeModules() {
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
        int interval = Math.max(1, voltageReadLoopInterval);
        if (voltageLoopCounter == 0) voltage = voltageSensor.getVoltage();
        voltageLoopCounter = (voltageLoopCounter + 1) % interval;
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
        int every = Math.max(1, telemetryEveryNLoops);
        if (every > 1 && (telemetryLoopCounter++ % every) != 0) {
            // Pending items carry over; calling update() here would cause partial-packet flicker.
            telemetryRenderedThisLoop = false;
            return;
        }
        telemetryRenderedThisLoop = true;

        DualTelemetry.enableDSTelemetry = telemetryToggles.dsTelemetry;
        DualTelemetry.enableDashboardTelemetry = telemetryToggles.dashboardTelemetry;

        // Dashboard data goes into the packet that also carries the field overlay, so exactly one
        // complete frame is sent per loop. Re-set every loop: updateDashboard swaps in a fresh packet.
        robot.telemetry.setPacket(robot.packet);

        Pose currentPose = robot.follower.pose();
        addStatusTelemetry(currentPose);

        if (telemetry instanceof DualTelemetry) {
            DualTelemetry et = (DualTelemetry) telemetry;

            et.addDashboardData("Game Time", "%.1fs", gameTimer.seconds());
            et.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());

            if (!modules.isEmpty()) {
                et.addSeparator();
                et.addGroupHeader("MODULES", HtmlFormatter.COLOR_MODULE);
                List<Module> ordered = telemetryOrderedModules();
                for (int i = 0; i < ordered.size(); i++) ordered.get(i).telemetry();
            }

            if (telemetryToggles.loopProfile) {
                et.addSeparator();
                et.addGroupHeader("LOOP PROFILE (avg ms)", HtmlFormatter.COLOR_BLUE);
                List<Map.Entry<String, Double>> snapshot = profiler.snapshotSortedDesc();
                for (int i = 0; i < snapshot.size(); i++) {
                    Map.Entry<String, Double> entry = snapshot.get(i);
                    et.addDashboardData(entry.getKey(), "%.2fms", entry.getValue());
                }
                // Single copy-pasteable dashboard value: whole-run per-section avg/peak/count. Rebuilt
                // rarely — it's ~60 String.formats and, being cumulative, barely moves loop to loop.
                if ((loopDumpCounter++ % 25) == 0) {
                    double loopAvg = profiledLoopCount == 0 ? 0 : profiledLoopSumMs / profiledLoopCount;
                    cachedLoopDump = profiler.report(profiledLoopCount, loopAvg, profiledLoopMaxMs);
                }
                et.addDashboardData("LOOP_DUMP", cachedLoopDump);
            }

            renderFieldMap(currentPose);
        } else {
            telemetry.addData("Game Time", "%.1fs", gameTimer.seconds());
            telemetry.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());
            List<Module> ordered = telemetryOrderedModules();
            for (int i = 0; i < ordered.size(); i++) ordered.get(i).telemetry();
        }

        telemetry();
        telemetry.update();
    }

    private void renderFieldMap(Pose fieldPose) {
        // Only sink is addDSLine (a no-op when DS telemetry is off) — skip the whole render then.
        if (!DualTelemetry.enableDSTelemetry) return;
        double fx = fieldPose.x();
        double fy = fieldPose.y();
        double fh = fieldPose.heading();
        boolean poseChanged = Math.abs(fx - lastFieldRenderX) > 0.5
                || Math.abs(fy - lastFieldRenderY) > 0.5
                || Math.abs(fh - lastFieldRenderHeading) > Math.toRadians(2);
        if (poseChanged || loopsSinceFieldRender >= fieldRenderInterval) {
            field.restore();
            field.drawRobot(fx, fy, fh,
                    Context.allianceColor.equals(AllianceColor.RED) ? COLOR_RED : COLOR_BLUE);
            cachedFieldHtml = HtmlFormatter.htmlSize(FONT_SMALL, field.renderHtml());
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

    protected void updateDashboard() {
        // Only send a frame carrying BOTH the routed telemetry data and the overlay. Sending one without
        // the other blanks that half for a frame, which reads as flicker; skipping the send entirely
        // just leaves the previous complete frame up.
        int every = Math.max(1, dashboardEveryNLoops);
        if (!telemetryRenderedThisLoop || (every > 1 && (dashboardLoopCounter++ % every) != 0)) {
            // Fresh packet discards draws accumulated this loop, which would otherwise pile up across skipped loops.
            robot.packet = newPacket();
            return;
        }

        Canvas overlay = robot.packet.fieldOverlay();

        if (!dashboardSkipFieldImage) {
            overlay.setAlpha(0.4);
            overlay.drawImage("/images/fieldcoordinates-pedro.png", 0, 0, 144, 144);
            overlay.setAlpha(1);
        }

        if (!dashboardSkipGrid) overlay.drawGrid(0, 0, 144, 144, 7, 7);

        FieldVisualization.drawRobot(overlay, robot.follower.pose());

        if (!dashboardSkipPoseHistory) {
            FieldVisualization.drawPoseHistory(overlay, robot.poseHistory);
        }

        dashboardOverlay(overlay);

        FtcDashboard.getInstance().sendTelemetryPacket(robot.packet);
        robot.packet = newPacket();
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
            String colorHex = Context.allianceColor == AllianceColor.RED
                    ? HtmlFormatter.COLOR_RED
                    : HtmlFormatter.COLOR_BLUE;
            cachedAllianceHtml = HtmlFormatter.htmlColor(colorHex,
                    HtmlFormatter.htmlBold(String.valueOf(Context.allianceColor)));
        }
        robot.telemetry.addRawHtml("Alliance", cachedAllianceHtml);
    }

    private void addVoltageCurrentTelemetry() {
        if (telemetryToggles.voltage) robot.telemetry.addDashboardData("Voltage", "%.2fV", voltage);
        if (telemetryToggles.current) robot.telemetry.addDashboardData("Current", "%.2fA", getTotalCurrent());
    }

    public final void requestStop() { stopRequested = true; }
    public final boolean isRunning() { return running; }
    public final double getVoltage() { return voltage; }
    public final ElapsedTime getGameTimer() { return gameTimer; }

    public final double avgLoopMs() {
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

    public final double getTotalCurrent() {
        int every = Math.max(1, currentReadEveryNLoops);
        // Throttle against the loop counter, not call count: this is only called on render loops with the current toggle on, so a per-call counter would read far less often than currentReadEveryNLoops promises.
        if (lastCurrentReadLoop == Long.MIN_VALUE || monotonicLoopCount - lastCurrentReadLoop >= every) {
            double total = 0;
            for (LynxModule hub : lynxHubs) total += hub.getCurrent(CurrentUnit.AMPS);
            cachedTotalCurrent = total;
            lastCurrentReadLoop = monotonicLoopCount;
        }
        return cachedTotalCurrent;
    }

    public final List<Module> getModules() {
        return Collections.unmodifiableList(modules);
    }

    public final LoopProfiler getProfiler() { return profiler; }
}
