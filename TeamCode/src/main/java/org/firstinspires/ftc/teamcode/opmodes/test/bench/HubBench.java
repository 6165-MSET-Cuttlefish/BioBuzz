package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.message.redux.ReceiveTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.robotcore.internal.usb.EthernetOverUsbSerialNumber;
import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.PoseRing;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;
import org.firstinspires.ftc.teamcode.architecture.telemetry.FieldMapRenderer;
import org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.lang.reflect.Field;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;

/**
 * Bare-Control-Hub costs, phase by phase: the SDK's own loop overhead, bus reads and writes on empty ports, the
 * framework's telemetry and dashboard packets, the route planner, and the Limelight poller against a fake
 * Limelight on loopback. Every phase is timed on this OpMode thread; the chunked ones stop each loop() after
 * {@link #CHUNK_NS} so the dashboard keeps hearing from it.
 */
@TeleOp(name = "Bench: Hub", group = "Test")
public class HubBench extends OpMode {
    private static final long CHUNK_NS = 30_000_000L;
    private static final int DS_HOLD_MS = Integer.MAX_VALUE;

    private abstract static class Phase {
        final String name;
        final Map<String, Object> result = new LinkedHashMap<>();
        private BenchIO.Gc gc;
        private BenchIO.Cpu cpu;
        long loops;

        Phase(String name) {
            this.name = name;
        }

        void begin() {}

        /** Once per loop(); true when finished. */
        abstract boolean step();

        void end() {}

        void release() {}

        final void start() {
            gc = BenchIO.Gc.now();
            cpu = BenchIO.Cpu.now();
            begin();
        }

        final void finish() {
            end();
            result.put("gc", BenchIO.Gc.now().since(gc, loops));
            result.put("cpu", BenchIO.Cpu.now().since(cpu));
        }
    }

    /** Runs {@code body} {@code total} times, a chunk per loop(), timing each call. */
    private abstract static class Repeat extends Phase {
        final int total;
        final Samples samples = new Samples();
        int done;

        Repeat(String name, int total) {
            super(name);
            this.total = total;
        }

        abstract void once(int i);

        @Override
        final boolean step() {
            loops++;
            long chunkStart = System.nanoTime();
            while (done < total && System.nanoTime() - chunkStart < CHUNK_NS) {
                long t0 = System.nanoTime();
                once(done);
                samples.addNanos(System.nanoTime() - t0);
                done++;
            }
            return done >= total;
        }

        @Override
        void end() {
            result.put("callMs", samples.summary());
        }
    }

    private BenchReport report;
    private final List<Phase> phases = new ArrayList<>();
    private int phaseIndex = -1;

    private LynxModule hub;
    private DcMotorEx[] motors;
    private DcMotorEx motor;
    private Servo servo;
    private AnalogInput analog;
    private VoltageSensor voltageSensor;
    private IMU imu;
    private EnhancedMotor enhancedMotor;
    private int sdkDefaultDsIntervalMs;

    @Override
    public void init() {
        report = new BenchReport("Hub", "hub");
        sdkDefaultDsIntervalMs = telemetry.getMsTransmissionInterval();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        if (hubs.size() != 1 || !hubs.get(0).isParent()) {
            throw new IllegalStateException("Bench: Hub expects the Control Hub alone in the config, found " + hubs.size()
                    + " REV hubs; " + BenchIO.RUNNER_HINT);
        }
        hub = hubs.get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        motors = new DcMotorEx[] {
                BenchIO.require(hardwareMap, DcMotorEx.class, "fl", BenchIO.RUNNER_HINT),
                BenchIO.require(hardwareMap, DcMotorEx.class, "fr", BenchIO.RUNNER_HINT),
                BenchIO.require(hardwareMap, DcMotorEx.class, "bl", BenchIO.RUNNER_HINT),
                BenchIO.require(hardwareMap, DcMotorEx.class, "br", BenchIO.RUNNER_HINT)};
        motor = motors[0];
        enhancedMotor = new EnhancedMotor(hardwareMap, "fr");
        servo = BenchIO.require(hardwareMap, Servo.class, "benchServo", BenchIO.RUNNER_HINT);
        analog = BenchIO.require(hardwareMap, AnalogInput.class, "floodgate", BenchIO.RUNNER_HINT);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        // As EnhancedOpMode does every loop; slothboard resets it at each init, before this runs.
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(OptimizationToggles.dashboardTransmissionIntervalMs);

        Map<String, Object> info = new LinkedHashMap<>();
        info.put("imuType", String.valueOf(hub.getImuType()));
        info.put("batteryVolts", voltageSensor.getVoltage());
        info.put("sdkDefaultDsMsTransmissionInterval", sdkDefaultDsIntervalMs);
        info.put("driverStationConnected", NetworkConnectionHandler.getInstance().isPeerConnected());
        info.put("slothboardIntervalAtInit", FtcDashboard.getInstance().getTelemetryTransmissionInterval());
        info.put("optimizationTogglesDashboardIntervalMs", OptimizationToggles.dashboardTransmissionIntervalMs);
        info.put("optimizationTogglesDsIntervalMs", OptimizationToggles.dsTransmissionIntervalMs);
        info.put("devices", BenchIO.deviceNames(hardwareMap));
        imu = BenchIO.require(hardwareMap, IMU.class, "imu", BenchIO.RUNNER_HINT);
        long t0 = System.nanoTime();
        boolean ok = imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        info.put("imuInitializeMs", (System.nanoTime() - t0) / 1e6);
        if (!ok) throw new IllegalStateException("the Control Hub's embedded IMU failed to initialize (type " + hub.getImuType() + ")");
        info.put("fakeLimelightPort", FakeLimelightServer.PORT);
        report.put("info", info);
        new FakeLimelightServer(0).close();
        buildPhases(BenchIO.param(report.params, "limelightPresent", 0) != 0,
                BenchIO.param(report.params, "plannerScale", 1));
    }

    @Override
    public void init_loop() {
        report.sendIfDue();
    }

    @Override
    public void start() {
        next();
    }

    @Override
    public void loop() {
        if (phaseIndex < phases.size()) {
            Phase p = phases.get(phaseIndex);
            if (p.step()) {
                p.finish();
                report.endPhase(p.name, p.result);
                next();
            }
        }
        report.sendIfDue();
    }

    private void next() {
        phaseIndex++;
        if (phaseIndex < phases.size()) {
            Phase p = phases.get(phaseIndex);
            report.phase(p.name + " (" + (phaseIndex + 1) + "/" + phases.size() + ")");
            p.start();
        } else {
            report.done(phases.size() + " phases");
        }
    }

    @Override
    public void stop() {
        try {
            for (DcMotorEx m : motors) m.setPower(0);
            telemetry.setMsTransmissionInterval(sdkDefaultDsIntervalMs);
        } finally {
            if (phaseIndex >= 0 && phaseIndex < phases.size()) phases.get(phaseIndex).release();
        }
    }

    private void buildPhases(boolean limelightPresent, double plannerScale) {
        phases.add(new Phase("sdkLoopEmpty") {
            final Samples periods = new Samples();
            long last;
            long startNs;

            @Override boolean step() {
                long now = System.nanoTime();
                if (last != 0) periods.addNanos(now - last);
                else startNs = now;
                last = now;
                loops++;
                return now - startNs > 5_000_000_000L;
            }

            @Override void end() {
                result.put("what", "period between loop() calls of an OpMode whose loop() does nothing: the SDK's 1 ms sleep, gamepad copy and telemetry.update()");
                result.put("loopPeriodMs", periods.summary());
                result.put("histogram", periods.histogram(0.25, 40));
            }
        });

        LinkedHashSet<Integer> dsIntervals = new LinkedHashSet<>();
        dsIntervals.add(OptimizationToggles.dsTransmissionIntervalMs);
        dsIntervals.add(100);
        dsIntervals.add(0);
        for (int interval : dsIntervals) addDsTransmission(interval);

        phases.add(new Repeat("bulkReadManual", 2000) {
            final Samples cachedPosition = new Samples();
            final Samples cachedVelocity = new Samples();
            final Samples cachedAnalog = new Samples();
            final Samples floodgateVolts = new Samples();

            @Override void once(int i) {
                hub.clearBulkCache();
                motor.getCurrentPosition();
                long t1 = System.nanoTime();
                motors[1].getCurrentPosition();
                long t2 = System.nanoTime();
                motor.getVelocity();
                long t3 = System.nanoTime();
                double volts = analog.getVoltage();
                long t4 = System.nanoTime();
                cachedPosition.addNanos(t2 - t1);
                cachedVelocity.addNanos(t3 - t2);
                cachedAnalog.addNanos(t4 - t3);
                floodgateVolts.add(volts);
            }

            @Override void end() {
                result.put("what", "MANUAL bulk caching: callMs is clearBulkCache() plus the first read (the bulk read) plus three cached reads");
                super.end();
                result.put("cachedPositionMs", cachedPosition.summary());
                result.put("cachedVelocityMs", cachedVelocity.summary());
                result.put("cachedAnalogMs", cachedAnalog.summary());
                result.put("floodgateVolts", floodgateVolts.summary());
            }
        });

        phases.add(new Repeat("readsBulkCachingOff", 1500) {
            final Samples position = new Samples();
            final Samples velocity = new Samples();
            final Samples analogRead = new Samples();
            final Samples floodgateVolts = new Samples();

            @Override void begin() {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            }

            @Override void once(int i) {
                long t0 = System.nanoTime();
                if (i % 3 == 0) {
                    motor.getCurrentPosition();
                    position.addNanos(System.nanoTime() - t0);
                } else if (i % 3 == 1) {
                    motor.getVelocity();
                    velocity.addNanos(System.nanoTime() - t0);
                } else {
                    double volts = analog.getVoltage();
                    analogRead.addNanos(System.nanoTime() - t0);
                    floodgateVolts.add(volts);
                }
            }

            @Override void end() {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                result.put("what", "bulk caching OFF: each read is its own hub command");
                result.put("getCurrentPositionMs", position.summary());
                result.put("getVelocityMs", velocity.summary());
                result.put("analogGetVoltageMs", analogRead.summary());
                result.put("floodgateVolts", floodgateVolts.summary());
            }
        });

        addCommand("voltageSensorGetVoltage", 300, "what EnhancedOpMode reads every 50 loops", i -> voltageSensor.getVoltage());
        addCommand("hubGetInputVoltage", 300, "what slothboard's GET_ROBOT_STATUS reads per hub on its socket thread",
                i -> hub.getInputVoltage(VoltageUnit.VOLTS));
        addCommand("hubGetCurrent", 300, "EnhancedOpMode's per-loop read with telemetryToggles.current",
                i -> hub.getCurrent(CurrentUnit.AMPS));
        addCommand("motorGetCurrent", 300, "one of Drivetrain.motorCurrentTelemetry's four reads", i -> motor.getCurrent(CurrentUnit.AMPS));
        addCommand("motorSetPowerChanging", 500, "a setPower that changes the value (a hub write)",
                i -> motor.setPower((i % 2 == 0) ? 0.1 : -0.1));
        addCommand("motorSetPowerSame", 500, "the same setPower value again on the raw SDK motor",
                i -> motor.setPower(0.1));
        addCommand("enhancedMotorSetPowerSame", 500, "the same value through EnhancedMotor's write cache",
                i -> enhancedMotor.setPower(0.1));
        addCommand("fourMotorWrite", 300, "four changing setPowers, one drivetrain write", i -> {
            double p = (i % 2 == 0) ? 0.1 : -0.1;
            for (DcMotorEx m : motors) m.setPower(p);
        });
        addCommand("servoSetPositionChanging", 500, "a changing servo setPosition on an empty port",
                i -> servo.setPosition((i % 2 == 0) ? 0.4 : 0.6));
        addCommand("servoSetPositionSame", 500, "the same servo position again", i -> servo.setPosition(0.4));
        addCommand("imuYawPitchRoll", 300, "embedded IMU read over I2C bus 0: the stand-in for a Pinpoint read",
                i -> imu.getRobotYawPitchRollAngles());
        addCommand("imuAngularVelocity", 300, "a second embedded IMU read", i -> imu.getRobotAngularVelocity(AngleUnit.RADIANS));

        addTelemetryBuild();
        addAllocatingLoop();
        addPlanner(plannerScale);
        addLimelightParse();
        addLimelightPostLoopback();
        addLimelightPollSim();
        addLimelightFailureModes(limelightPresent);
    }

    private interface Body {
        void run(int i);
    }

    private void addCommand(String name, int count, final String what, final Body body) {
        phases.add(new Repeat(name, count) {
            @Override void once(int i) {
                body.run(i);
            }

            @Override void end() {
                result.put("what", what);
                super.end();
            }
        });
    }

    private void addDsTransmission(final int intervalMs) {
        phases.add(new Phase("dsTransmission" + intervalMs + "ms") {
            final Samples updateSentMs = new Samples();
            final Samples updateHeldMs = new Samples();
            final Samples sendIntervals = new Samples();
            long lastSend;
            long startNs;
            int sends;

            @Override void begin() {
                startNs = System.nanoTime();
            }

            @Override boolean step() {
                loops++;
                for (int i = 0; i < 20; i++) telemetry.addData("line " + i, "%.3f", i * 1.5);
                telemetry.setMsTransmissionInterval(intervalMs);
                long t0 = System.nanoTime();
                boolean sent = telemetry.update();
                long t1 = System.nanoTime();
                telemetry.setMsTransmissionInterval(DS_HOLD_MS);
                if (sent) {
                    updateSentMs.addNanos(t1 - t0);
                    if (lastSend != 0) sendIntervals.addNanos(t0 - lastSend);
                    lastSend = t0;
                    sends++;
                } else {
                    updateHeldMs.addNanos(t1 - t0);
                }
                return t1 - startNs > 5_000_000_000L;
            }

            @Override void end() {
                telemetry.setMsTransmissionInterval(sdkDefaultDsIntervalMs);
                result.put("what", "20 SDK telemetry lines and telemetry.update() every loop at msTransmissionInterval "
                        + intervalMs + "; update() returns true only when it transmits; the SDK's own update() after "
                        + "loop() is held off, so only these sends happen");
                result.put("msTransmissionInterval", intervalMs);
                result.put("transmissions", sends);
                result.put("loops", loops);
                result.put("intervalBetweenTransmissionsMs", sendIntervals.summary());
                result.put("updateThatTransmittedMs", updateSentMs.summary());
                result.put("updateThatHeldMs", updateHeldMs.summary());
            }
        });
    }

    private final class Frame {
        final DualTelemetry dt = new DualTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final FieldMapRenderer field = new FieldMapRenderer(73, 74);
        final PoseRing poses = new PoseRing(30);
        final String loopDump;
        TelemetryPacket packet;

        Frame() {
            field.drawFieldLayout();
            field.snapshot();
            for (int i = 0; i < 30; i++) poses.record(new Pose(20 + i, 72 + i * 0.5, 0.1 * i));
            StringBuilder sb = new StringBuilder("loops=12345 loopAvg=6.21 loopMax=31.0 sumMarks=5.90 unprofiled=0.31ms ||");
            while (sb.length() < 700) sb.append(" read.Drivetrain=0.05/1.2/12345;");
            loopDump = sb.toString();
        }

        /** EnhancedOpMode.updateTelemetry's lines for three modules and the loop profile; returns the DS field-map ms. */
        double build() {
            dt.beginLoop();
            boolean dsFrame = dt.isDSFrame();
            packet = new TelemetryPacket(false);
            packet.setDisplayFormat(TelemetryPacket.DisplayFormat.HTML);
            dt.setPacket(packet);
            dt.addGroupHeader("ROBOT STATUS");
            dt.addRawHtml("Alliance", HtmlFormatter.htmlColor(HtmlFormatter.COLOR_RED, HtmlFormatter.htmlBold("RED")));
            dt.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°", 20.0, 72.0, 90.0);
            dt.addDashboardData("Voltage", "%.2fV", 12.5);
            dt.addDashboardData("Game Time", "%.1fs", 12.3);
            dt.addData("Loop Time", "%.1fms (avg %.1fms)", 6.2, 6.1);
            dt.addSeparator();
            dt.addGroupHeader("MODULES", HtmlFormatter.COLOR_MODULE);
            for (int m = 0; m < 3; m++) {
                dt.addModuleHeader("Module" + m, "IDLE | READY");
                for (int k = 0; k < 4; k++) dt.addDashboardData("Module" + m + " value" + k, "%.2f", k * 0.37);
            }
            dt.addSeparator();
            dt.addGroupHeader("LOOP PROFILE (avg ms)", HtmlFormatter.COLOR_BLUE);
            for (int s = 0; s < 15; s++) dt.addDashboardData("section" + s, "%.2fms", s * 0.11);
            dt.addDashboardData("LOOP_DUMP", loopDump);
            double fieldMs = 0;
            if (dsFrame) {
                long f0 = System.nanoTime();
                field.restore();
                field.drawRobot(20, 72, Math.PI / 2, HtmlFormatter.COLOR_RED);
                dt.addDSLine(HtmlFormatter.htmlSize(HtmlFormatter.FONT_SMALL, field.renderHtml()));
                fieldMs = (System.nanoTime() - f0) / 1e6;
            }
            return fieldMs;
        }

        void overlay() {
            Canvas overlay = packet.fieldOverlay();
            overlay.setAlpha(0.4);
            overlay.drawImage("/images/fieldcoordinates-pedro.png", 0, 0, 144, 144);
            overlay.setAlpha(1);
            overlay.drawGrid(0, 0, 144, 144, 7, 7);
            FieldVisualization.drawRobot(overlay, new Pose(20, 72, Math.PI / 2));
            FieldVisualization.drawPoseHistory(overlay, poses);
        }
    }

    private void addTelemetryBuild() {
        phases.add(new Phase("telemetryBuild") {
            Frame frame;
            final Samples dashBuild = new Samples();
            final Samples dashUpdate = new Samples();
            final Samples dsBuild = new Samples();
            final Samples dsUpdate = new Samples();
            final Samples fieldMap = new Samples();
            final Samples overlay = new Samples();
            final Samples send = new Samples();
            final Samples serialize = new Samples();
            final Samples packetChars = new Samples();
            int i;
            BenchIO.Gc gcDash;
            BenchIO.Gc gcDs;
            Map<String, Object> dashAlloc;

            @Override void begin() {
                frame = new Frame();
                frame.dt.setEnabled(false, true);
                gcDash = BenchIO.Gc.now();
            }

            @Override boolean step() {
                loops++;
                if (i > 300) telemetry.setMsTransmissionInterval(0);
                long chunkStart = System.nanoTime();
                while (System.nanoTime() - chunkStart < CHUNK_NS && i < 600) {
                    boolean ds = i >= 300;
                    if (i == 300) {
                        dashAlloc = BenchIO.Gc.now().since(gcDash, 300);
                        frame.dt.setEnabled(true, true);
                        telemetry.setMsTransmissionInterval(0);
                        gcDs = BenchIO.Gc.now();
                    }
                    long t0 = System.nanoTime();
                    double fieldMs = frame.build();
                    long t1 = System.nanoTime();
                    frame.dt.update();
                    long t2 = System.nanoTime();
                    frame.overlay();
                    long t3 = System.nanoTime();
                    if (i % 10 == 0) {
                        String json = DashboardCore.GSON.toJson(new ReceiveTelemetry(Collections.singletonList(frame.packet)));
                        serialize.addNanos(System.nanoTime() - t3);
                        packetChars.add(json.length());
                    }
                    long t4 = System.nanoTime();
                    FtcDashboard.getInstance().sendTelemetryPacket(frame.packet);
                    send.addNanos(System.nanoTime() - t4);
                    (ds ? dsBuild : dashBuild).addNanos(t1 - t0);
                    (ds ? dsUpdate : dashUpdate).addNanos(t2 - t1);
                    if (ds) fieldMap.add(fieldMs);
                    overlay.addNanos(t3 - t2);
                    i++;
                }
                if (i > 300) telemetry.setMsTransmissionInterval(DS_HOLD_MS);
                return i >= 600;
            }

            @Override void end() {
                telemetry.setMsTransmissionInterval(sdkDefaultDsIntervalMs);
                result.put("what", "the real DualTelemetry, FieldMapRenderer and packet overlay building an EnhancedOpMode-sized frame: "
                        + "300 dashboard-only frames, then 300 DS frames (DS interval 0) with the braille field map");
                result.put("dashboardOnlyBuildMs", dashBuild.summary());
                result.put("dashboardOnlyUpdateMs", dashUpdate.summary());
                result.put("dsFrameBuildMs", dsBuild.summary());
                result.put("dsFrameUpdateMs", dsUpdate.summary());
                result.put("fieldMapRenderMs", fieldMap.summary());
                result.put("overlayMs", overlay.summary());
                result.put("sendTelemetryPacketMs", send.summary());
                result.put("gsonSerializeOnePacketMs", serialize.summary());
                result.put("serializedPacketChars", packetChars.summary());
                result.put("allocDashboardOnlyFrames", dashAlloc);
                result.put("allocDsFrames", BenchIO.Gc.now().since(gcDs, 300));
            }
        });
    }

    private void addAllocatingLoop() {
        phases.add(new Phase("allocatingLoop") {
            Frame frame;
            final Samples periods = new Samples();
            long last;
            long startNs;

            @Override void begin() {
                frame = new Frame();
                frame.dt.syncDsTransmissionInterval();
            }

            @Override boolean step() {
                long now = System.nanoTime();
                if (last != 0) periods.addNanos(now - last);
                else startNs = now;
                last = now;
                loops++;
                frame.build();
                frame.overlay();
                frame.dt.update();
                FtcDashboard.getInstance().sendTelemetryPacket(frame.packet);
                return now - startNs > 10_000_000_000L;
            }

            @Override void end() {
                telemetry.setMsTransmissionInterval(sdkDefaultDsIntervalMs);
                double median = periods.percentile(50);
                result.put("what", "one framework-sized telemetry frame and packet per loop for 10 s (DS frames every "
                        + "OptimizationToggles.dsTransmissionIntervalMs = " + OptimizationToggles.dsTransmissionIntervalMs
                        + " ms): GC counts and the loop-period outliers they cause");
                result.put("loopPeriodMs", periods.summary());
                result.put("histogram", periods.histogram(0.5, 60));
                result.put("medianMs", median);
            }
        });
    }

    private void addPlanner(final double scale) {
        phases.add(new Phase("planner") {
            final PlannerWorkload workload = new PlannerWorkload();
            final List<PlannerWorkload.Layout> queue = new ArrayList<>();
            final Map<String, List<PlannerWorkload.Timing>> byBalls = new LinkedHashMap<>();
            final Map<String, List<PlannerWorkload.Timing>> byCase = new LinkedHashMap<>();
            final Samples chunkMs = new Samples();
            final Map<String, Object> cold = new LinkedHashMap<>();
            int next;

            @Override void begin() {
                // An auto plans once, usually soon after an app start or a Sloth load, before the JIT has seen the planner.
                queue.add(workload.random(4));
                int[] plansPerCount = {40, 60, 60, 60, 48, 40};
                for (int n = 0; n <= 5; n++) {
                    for (int k = 0; k < Math.max(1, (int) Math.round(plansPerCount[n] * scale)); k++) {
                        queue.add(workload.random(n));
                    }
                }
                for (PlannerWorkload.Layout l : workload.crafted()) {
                    for (int r = 0; r < 5; r++) queue.add(l);
                }
            }

            @Override boolean step() {
                loops++;
                long chunkStart = System.nanoTime();
                while (next < queue.size() && System.nanoTime() - chunkStart < CHUNK_NS) {
                    PlannerWorkload.Layout l = queue.get(next++);
                    if (next == 1) {
                        cold.put("firstPlanInProcess", !PlannerWorkload.plannedBefore());
                        PlannerWorkload.Timing t = workload.plan(l);
                        cold.put("balls", l.balls.length);
                        cold.put("totalMs", t.routeMs + t.buildMs);
                        continue;
                    }
                    PlannerWorkload.Timing t = workload.plan(l);
                    String key = l.label.startsWith("random") ? "n=" + l.balls.length : l.label;
                    Map<String, List<PlannerWorkload.Timing>> into = l.label.startsWith("random") ? byBalls : byCase;
                    if (!into.containsKey(key)) into.put(key, new ArrayList<PlannerWorkload.Timing>());
                    into.get(key).add(t);
                }
                chunkMs.addNanos(System.nanoTime() - chunkStart);
                report.status("%d/%d plans", next, queue.size());
                return next >= queue.size();
            }

            @Override void end() {
                result.put("what", "RouteOptimizer.findOptimalRoute + RoutePathBuilder.build as Vision Ball Collection calls them "
                        + "(RED own half, the HIVE rails, wall and rail gaps 1, an 18 x 18 in robot with its intake 9 in ahead, start = return), seeded random layouts plus crafted worst cases");
                Map<String, Object> balls = new LinkedHashMap<>();
                for (Map.Entry<String, List<PlannerWorkload.Timing>> e : byBalls.entrySet()) {
                    balls.put(e.getKey(), PlannerWorkload.summarize(e.getValue()));
                }
                Map<String, Object> cases = new LinkedHashMap<>();
                for (Map.Entry<String, List<PlannerWorkload.Timing>> e : byCase.entrySet()) {
                    cases.put(e.getKey(), PlannerWorkload.summarize(e.getValue()));
                }
                result.put("coldFirstPlan", cold);
                result.put("byBallCount", balls);
                result.put("byCase", cases);
                result.put("loopChunkMs", chunkMs.summary());
            }
        });
    }

    private void addLimelightParse() {
        final String[] names = {"small", "typical", "ballScriptMax", "large", "xlarge"};
        final String[] contents = {"ball script, 0 balls (the smallest reply a Limelight sends)", "ball script, 3 balls",
                "ball script, 10 balls (its llpython limit)", "8 AprilTags + 16 detector entries",
                "24 AprilTags + 64 detector entries"};
        final String[] jsons = {
                ParsedLLResult.synthetic(0, 123456.0),
                ParsedLLResult.synthetic(3, 123456.0),
                ParsedLLResult.synthetic(10, 123456.0),
                ParsedLLResult.synthetic(0, 8, 16, 123456.0),
                ParsedLLResult.synthetic(0, 24, 64, 123456.0)};
        final int perCase = 400;
        phases.add(new Phase("limelightParse") {
            final Samples[] jsonParse = new Samples[names.length];
            final Samples[] llParse = new Samples[names.length];
            final BenchIO.Gc[] gcStart = new BenchIO.Gc[names.length];
            final Object[] alloc = new Object[names.length];
            int i;

            @Override void begin() {
                for (int c = 0; c < names.length; c++) {
                    jsonParse[c] = new Samples();
                    llParse[c] = new Samples();
                }
            }

            @Override boolean step() {
                loops++;
                long chunkStart = System.nanoTime();
                while (i < perCase * names.length && System.nanoTime() - chunkStart < CHUNK_NS) {
                    int c = i / perCase;
                    if (i % perCase == 0) gcStart[c] = BenchIO.Gc.now();
                    try {
                        long t0 = System.nanoTime();
                        JSONObject o = new JSONObject(jsons[c]);
                        long t1 = System.nanoTime();
                        LLResult r = new ParsedLLResult(o);
                        r.getPythonOutput();
                        r.getPipelineIndex();
                        r.getTimestamp();
                        r.getCaptureLatency();
                        r.getTargetingLatency();
                        long t2 = System.nanoTime();
                        jsonParse[c].addNanos(t1 - t0);
                        llParse[c].addNanos(t2 - t1);
                    } catch (JSONException e) {
                        throw new IllegalStateException("synthetic Limelight JSON didn't parse: " + jsons[c], e);
                    }
                    i++;
                    if (i % perCase == 0) alloc[c] = BenchIO.Gc.now().since(gcStart[c], perCase);
                }
                return i >= perCase * names.length;
            }

            @Override void end() {
                result.put("what", "the poller's per-result work minus HTTP, on this thread: new JSONObject(body), then "
                        + "LLResult's constructor (which builds every Fiducial/Detector entry) and the getters "
                        + "LimelightBallSource reads; synthetic results from small to large. gc.bytesAllocatedPerLoop is "
                        + "bytes per parse, process-wide");
                for (int c = 0; c < names.length; c++) {
                    Map<String, Object> m = new LinkedHashMap<>();
                    m.put("contents", contents[c]);
                    m.put("jsonChars", jsons[c].length());
                    m.put("jsonObjectMs", jsonParse[c].summary());
                    m.put("llResultMs", llParse[c].summary());
                    m.put("gc", alloc[c]);
                    result.put(names[c], m);
                }
            }
        });
    }

    /** POSTs against the fake Limelight on loopback: the HttpURLConnection floor under the real Limelight's numbers. */
    private void addLimelightPostLoopback() {
        phases.add(new Phase("limelightPostLoopback") {
            FakeLimelightServer server;
            Limelight3A limelight;
            final Samples pipelineSwitch = new Samples();
            final Samples pythonInputs = new Samples();
            final Samples robotOrientation = new Samples();
            final double[] inputs = {1, 2, 3, 4, 5, 6, 7, 8};
            int i;

            @Override void begin() {
                server = new FakeLimelightServer(3);
                limelight = loopbackLimelight(FakeLimelightServer.PORT);
            }

            @Override boolean step() {
                loops++;
                long chunkStart = System.nanoTime();
                while (i < 300 && System.nanoTime() - chunkStart < CHUNK_NS) {
                    inputs[0] = i;
                    long t0 = System.nanoTime();
                    boolean ok;
                    Samples into;
                    String call;
                    switch (i % 3) {
                        case 0:
                            ok = limelight.pipelineSwitch(4);
                            into = pipelineSwitch;
                            call = "pipelineSwitch";
                            break;
                        case 1:
                            ok = limelight.updatePythonInputs(inputs);
                            into = pythonInputs;
                            call = "updatePythonInputs";
                            break;
                        default:
                            ok = limelight.updateRobotOrientation(i * 1.2 - 180);
                            into = robotOrientation;
                            call = "updateRobotOrientation";
                            break;
                    }
                    into.addNanos(System.nanoTime() - t0);
                    server.check();
                    if (!ok) throw new IllegalStateException(call + " returned false against the fake Limelight, which answers 200");
                    i++;
                }
                return i >= 300;
            }

            @Override void release() {
                if (limelight != null) limelight.stop();
                if (server != null) server.close();
            }

            @Override void end() {
                release();
                result.put("what", "Limelight3A's blocking POSTs (100 ms connect, 15 s read timeout, on the calling thread) "
                        + "against a fake Limelight on 127.0.0.1 with the poller off: HTTP setup and teardown on the hub, no USB "
                        + "network and no Limelight-side work; the fake's CPU counts here too");
                result.put("pipelineSwitchMs", pipelineSwitch.summary());
                result.put("updatePythonInputsMs", pythonInputs.summary());
                result.put("updateRobotOrientationMs", robotOrientation.summary());
                result.put("requestsServed", server.requests());
                result.put("limelightBaseUrl", baseUrl(limelight));
            }
        });
    }

    private static Limelight3A limelightAt(String address) {
        try {
            return new Limelight3A(EthernetOverUsbSerialNumber.fromIpAddress(address, "bench"), "bench-" + address,
                    InetAddress.getByName(address));
        } catch (UnknownHostException e) {
            throw new IllegalStateException("bad bench Limelight address " + address, e);
        }
    }

    private static Limelight3A loopbackLimelight(int port) {
        Limelight3A limelight = limelightAt("127.0.0.1");
        String url = "http://127.0.0.1:" + port;
        try {
            Field f = Limelight3A.class.getDeclaredField("baseUrl");
            f.setAccessible(true);
            f.set(limelight, url);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new IllegalStateException("Limelight3A.baseUrl changed in this SDK", e);
        }
        if (!url.equals(baseUrl(limelight))) throw new IllegalStateException("Limelight3A.baseUrl didn't take " + url);
        return limelight;
    }

    private static String baseUrl(Limelight3A limelight) {
        try {
            Field f = Limelight3A.class.getDeclaredField("baseUrl");
            f.setAccessible(true);
            return (String) f.get(limelight);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new IllegalStateException("Limelight3A.baseUrl changed in this SDK", e);
        }
    }

    private static int unusedLoopbackPort() {
        try (ServerSocket s = new ServerSocket(0, 1, InetAddress.getByName("127.0.0.1"))) {
            return s.getLocalPort();
        } catch (IOException e) {
            throw new IllegalStateException("can't find a free loopback port", e);
        }
    }

    private static String connectFailure(int port) {
        try (Socket s = new Socket()) {
            s.connect(new InetSocketAddress(InetAddress.getByName("127.0.0.1"), port), 100);
        } catch (IOException e) {
            return e.getClass().getSimpleName();
        }
        throw new IllegalStateException("127.0.0.1:" + port + " accepted a connection; the refused case needs nothing there");
    }

    /**
     * Starts the SDK poller at {@code hz}. Limelight3A ignores setPollRateHz while running and the device outlives the
     * OpMode, so it is stopped first, and the interval is checked because the SDK clamps to 1-250 Hz silently.
     */
    static void startPolling(Limelight3A limelight, int hz) {
        limelight.stop();
        limelight.setPollRateHz(hz);
        long interval = pollIntervalMs(limelight);
        if (interval != 1000 / hz) {
            throw new IllegalStateException("Limelight3A took " + hz + " Hz as a " + interval + " ms poll interval, not "
                    + 1000 / hz + " ms (the SDK clamps to 1-250 Hz)");
        }
        limelight.start();
        if (!limelight.isRunning()) throw new IllegalStateException("Limelight3A.start() didn't start the poller");
    }

    static long pollIntervalMs(Limelight3A limelight) {
        try {
            Field f = Limelight3A.class.getDeclaredField("pollIntervalMs");
            f.setAccessible(true);
            return f.getLong(limelight);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new IllegalStateException("Limelight3A.pollIntervalMs changed in this SDK", e);
        }
    }

    /** Fixed CPU work standing in for a loop body: its duration grows only when other threads take the core. */
    private static double workload() {
        double acc = 0;
        for (int i = 0; i < 20000; i++) acc += Math.sin(i * 0.001) * Math.cos(i * 0.002);
        return acc;
    }

    private void addLimelightPollSim() {
        final int[] rates = {0, 100, 200, 250};
        for (final int rate : rates) {
            phases.add(new Phase("limelightPoll" + (rate == 0 ? "Off" : rate + "Hz")) {
                FakeLimelightServer server;
                Limelight3A limelight;
                final Samples periods = new Samples();
                final Samples work = new Samples();
                final Samples staleness = new Samples();
                double lastTs = Double.NaN;
                int newResults;
                long last;
                long startNs;
                double serverCpu0;
                long requests0;
                double sink;

                @Override void begin() {
                    server = new FakeLimelightServer(3);
                    limelight = loopbackLimelight(FakeLimelightServer.PORT);
                    if (rate > 0) startPolling(limelight, rate);
                    serverCpu0 = server.cpuMs();
                    requests0 = server.requests();
                }

                @Override boolean step() {
                    long now = System.nanoTime();
                    if (last != 0) periods.addNanos(now - last);
                    else startNs = now;
                    last = now;
                    loops++;
                    long w0 = System.nanoTime();
                    sink += workload();
                    work.addNanos(System.nanoTime() - w0);
                    LLResult r = limelight.getLatestResult();
                    if (r.getTimestamp() != lastTs) {
                        lastTs = r.getTimestamp();
                        newResults++;
                        staleness.add(r.getStaleness());
                    }
                    server.check();
                    return now - startNs > 6_000_000_000L;
                }

                @Override void release() {
                    if (limelight != null) limelight.stop();
                    if (server != null) server.close();
                }

                @Override void end() {
                    double seconds = (last - startNs) / 1e9;
                    release();
                    result.put("what", "the SDK's Limelight3A poller at " + (rate == 0 ? "off" : rate + " Hz")
                            + " against a fake Limelight on 127.0.0.1 while loop() runs a fixed workload; "
                            + "the fake's own CPU is reported separately and inflates the load");
                    result.put("requestedHz", rate);
                    result.put("limelightBaseUrl", baseUrl(limelight));
                    result.put("pollIntervalMs", pollIntervalMs(limelight));
                    result.put("requestsServedPerSecond", (server.requests() - requests0) / seconds);
                    result.put("fakeServerCpuMsPerSecond", (server.cpuMs() - serverCpu0) / seconds);
                    result.put("newResultsSeenPerSecond", newResults / seconds);
                    result.put("stalenessAtReadMs", staleness.summary());
                    result.put("loopPeriodMs", periods.summary());
                    result.put("workloadMs", work.summary());
                    result.put("checksum", sink);
                }
            });
        }
    }

    private void addLimelightFailureModes(final boolean limelightPresent) {
        phases.add(new Phase("limelightFailureModes") {
            final Map<String, Object> cases = new LinkedHashMap<>();
            int stage;

            @Override boolean step() {
                loops++;
                switch (stage++) {
                    case 0: {
                        int unused = unusedLoopbackPort();
                        Map<String, Object> refused = timeSwitches(loopbackLimelight(unused), 10,
                                "nothing listening on 127.0.0.1:" + unused + " (connection refused)");
                        refused.put("refusedTargetProbe", connectFailure(unused));
                        cases.put("refusedPipelineSwitch", refused);
                        return false;
                    }
                    case 1:
                        if (limelightPresent) {
                            cases.put("unpluggedPipelineSwitch", "skipped: a Limelight is plugged in, and this would switch its pipeline");
                        } else {
                            cases.put("unpluggedPipelineSwitch", timeSwitches(limelightAt("172.29.0.1"), 3,
                                    "the Limelight's own address with no Limelight plugged in"));
                        }
                        return false;
                    case 2: {
                        FakeLimelightServer s = new FakeLimelightServer(3);
                        try {
                            s.setMode(FakeLimelightServer.Mode.ERROR_500);
                            cases.put("http500PipelineSwitch", timeSwitches(loopbackLimelight(FakeLimelightServer.PORT), 10,
                                    "a Limelight that answers every request with HTTP 500"));
                            s.check();
                        } finally {
                            s.close();
                        }
                        return false;
                    }
                    case 3: {
                        FakeLimelightServer s = new FakeLimelightServer(3);
                        try {
                            s.setMode(FakeLimelightServer.Mode.HANG);
                            Limelight3A ll = loopbackLimelight(FakeLimelightServer.PORT);
                            Map<String, Object> hang = new LinkedHashMap<>();
                            hang.put("pipelineSwitch", timeSwitches(ll, 1,
                                    "a Limelight that accepts the connection and never answers (booting or wedged)"));
                            Samples status = new Samples();
                            for (int i = 0; i < 3; i++) {
                                long t0 = System.nanoTime();
                                ll.getStatus();
                                status.addNanos(System.nanoTime() - t0);
                            }
                            hang.put("getStatusMs", status.summary());
                            cases.put("hangingLimelight", hang);
                            s.check();
                        } finally {
                            s.close();
                        }
                        return true;
                    }
                    default:
                        throw new IllegalStateException("limelightFailureModes has no stage " + stage);
                }
            }

            @Override void end() {
                result.put("what", "how long Limelight3A.pipelineSwitch (a blocking POST on the calling thread) takes when it fails");
                result.putAll(cases);
            }
        });
    }

    private static Map<String, Object> timeSwitches(Limelight3A limelight, int count, String what) {
        Samples ms = new Samples();
        int accepted = 0;
        for (int i = 0; i < count; i++) {
            long t0 = System.nanoTime();
            if (limelight.pipelineSwitch(4)) accepted++;
            ms.addNanos(System.nanoTime() - t0);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", what);
        m.put("pipelineSwitchMs", ms.summary());
        m.put("returnedTrue", accepted);
        return m;
    }
}
