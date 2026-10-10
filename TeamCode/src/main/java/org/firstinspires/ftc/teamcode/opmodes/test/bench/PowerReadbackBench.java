package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Power Readback", group = "Test")
public class PowerReadbackBench extends OpMode {
    static final double SELF_CHECK_POWER = 0.2;
    private static final double RUN_S = 3;
    private static final double MOVED = 0.01;
    private static final int MAX_CHANGES = 2_000;
    private static final long PACKET_INTERVAL_NS = 50_000_000L;

    private static final class Window {
        long loops;
        double maxHub;
        double maxSdk;
        double firstMovedMs = Double.NaN;

        void add(double ms, double hubPower, double sdkPower) {
            loops++;
            maxHub = Math.max(maxHub, hubPower);
            maxSdk = Math.max(maxSdk, sdkPower);
            if (hubPower >= MOVED && Double.isNaN(firstMovedMs)) firstMovedMs = ms;
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("loops", loops);
            m.put("maxHubPower", maxHub);
            m.put("maxSdkPower", maxSdk);
            m.put("firstMovedMs", firstMovedMs);
            return m;
        }
    }

    private BenchReport report;
    private LynxModule hub;
    private DcMotorEx fl;
    private int port;
    private boolean selfCheckOk;
    private final Window initWindow = new Window();
    private final Window runWindow = new Window();
    private final List<double[]> changes = new ArrayList<>();
    private long startNs;
    private long runStartNs;
    private long loops;
    private double lastHub = Double.NaN;
    private double lastSdk = Double.NaN;
    private long lastPacketNs;

    @Override
    public void init() {
        report = new BenchReport("Power Readback", "power_readback");
        hub = HubRegisters.controlHub(hardwareMap, "Bench: Power Readback");
        fl = BenchIO.require(hardwareMap, DcMotorEx.class, "fl", BenchIO.RUNNER_HINT);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        port = fl.getPortNumber();
        fl.setPower(SELF_CHECK_POWER);
        double atPower = Math.abs(HubRegisters.motorPower(hub, port));
        fl.setPower(0);
        double atZero = Math.abs(HubRegisters.motorPower(hub, port));
        selfCheckOk = Math.abs(atPower - SELF_CHECK_POWER) < MOVED && atZero < MOVED;
        Map<String, Object> selfCheck = new LinkedHashMap<>();
        selfCheck.put("ok", selfCheckOk);
        selfCheck.put("hubPowerAfterSettingIt", atPower);
        selfCheck.put("hubPowerAfterZero", atZero);
        report.put("selfCheck", selfCheck);
        report.put("changeColumns", new String[] {"ms", "running", "hubPower", "sdkPower"});
        report.put("changes", changes);
        report.status(selfCheckOk ? "self-check OK" : "self-check FAILED");
        startNs = System.nanoTime();
    }

    @Override
    public void init_loop() {
        sample(initWindow, false);
    }

    @Override
    public void start() {
        runStartNs = System.nanoTime();
        report.phase("run");
    }

    @Override
    public void loop() {
        sample(runWindow, true);
        if (!report.isDone() && (System.nanoTime() - runStartNs) / 1e9 >= RUN_S) {
            report.put("init", initWindow.toMap());
            report.put("run", runWindow.toMap());
            report.done(String.format("fl's hub power peaked at %.3f in INIT and %.3f running", initWindow.maxHub,
                    runWindow.maxHub));
        }
    }

    private void sample(Window window, boolean running) {
        loops++;
        long now = System.nanoTime();
        double ms = (now - startNs) / 1e6;
        double hubPower = Math.abs(HubRegisters.motorPower(hub, port));
        double sdkPower = Math.abs(fl.getPower());
        window.add(ms, hubPower, sdkPower);
        if ((hubPower != lastHub || sdkPower != lastSdk) && changes.size() < MAX_CHANGES) {
            changes.add(new double[] {ms, running ? 1 : 0, hubPower, sdkPower});
        }
        lastHub = hubPower;
        lastSdk = sdkPower;
        if (now - lastPacketNs < PACKET_INTERVAL_NS) return;
        lastPacketNs = now;
        TelemetryPacket p = new TelemetryPacket(false);
        report.addTo(p);
        p.put("Bench loop", loops);
        p.put("Bench self-check", selfCheckOk ? "OK" : "FAILED");
        p.put("fl hub power", String.format("%.3f", hubPower));
        p.put("fl sdk power", String.format("%.3f", sdkPower));
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    @Override
    public void stop() {
        fl.setPower(0);
    }
}
