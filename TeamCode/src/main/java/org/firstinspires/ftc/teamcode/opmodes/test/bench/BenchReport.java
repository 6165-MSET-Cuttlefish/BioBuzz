package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;

/** run_bench.py watches these keys by name; the file is rewritten every phase so a crash keeps what finished. */
final class BenchReport {
    static final String KEY_PHASE = "Bench phase";
    static final String KEY_STATUS = "Bench status";
    static final String KEY_DONE = "BENCH DONE";
    static final String KEY_LOOP = "Bench loop";

    private static final long PACKET_INTERVAL_NS = 100_000_000L;
    private static final Consumer<TelemetryPacket> NO_EXTRA = packet -> {};

    final String fileName;
    final Map<String, Object> root = new LinkedHashMap<>();
    final Map<String, Object> phases = new LinkedHashMap<>();
    final Map<String, Object> params;
    private String phase = "init";
    private String status = "";
    private String doneLine;
    private long lastPacketNs;

    BenchReport(String benchName, String fileName) {
        this.fileName = fileName;
        params = BenchIO.params();
        root.put("bench", benchName);
        root.put("complete", false);
        root.put("device", BenchIO.deviceInfo());
        root.put("params", params);
        root.put("phases", phases);
        BenchIO.log("%s starting, results in %s/%s.json", benchName, BenchIO.dir(), fileName);
    }

    void phase(String name) {
        phase = name;
        status = "";
        BenchIO.log("phase %s", name);
        sendNow(NO_EXTRA);
    }

    String phase() {
        return phase;
    }

    void status(String format, Object... args) {
        status = String.format(format, args);
    }

    void put(String key, Object value) {
        root.put(key, value);
    }

    void endPhase(String name, Map<String, Object> result) {
        phases.put(name, result);
        save();
    }

    void save() {
        BenchIO.write(fileName, root);
    }

    void done(String summary) {
        root.put("complete", true);
        root.put("summary", summary);
        save();
        doneLine = summary;
        phase = "done";
        BenchIO.log("BENCH DONE %s: %s", fileName, summary);
        sendNow(NO_EXTRA);
    }

    boolean isDone() {
        return doneLine != null;
    }

    /** For benches outside EnhancedOpMode: sends the keys straight to slothboard, at most every 100 ms. */
    void sendIfDue() {
        sendIfDue(PACKET_INTERVAL_NS, NO_EXTRA);
    }

    void sendIfDue(long intervalNs, Consumer<TelemetryPacket> extra) {
        if (System.nanoTime() - lastPacketNs >= intervalNs) sendNow(extra);
    }

    /** Keeps sending through a LinearOpMode's INIT; true if it then started rather than stopped. */
    boolean holdUntilStart(LinearOpMode opMode) {
        while (opMode.opModeInInit()) {
            sendIfDue();
            opMode.sleep(20);
        }
        return opMode.opModeIsActive();
    }

    void holdUntilStop(LinearOpMode opMode) {
        while (opMode.opModeIsActive()) {
            sendIfDue();
            opMode.sleep(20);
        }
    }

    private void sendNow(Consumer<TelemetryPacket> extra) {
        lastPacketNs = System.nanoTime();
        TelemetryPacket p = new TelemetryPacket(false);
        addTo(p);
        extra.accept(p);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    void addTo(TelemetryPacket packet) {
        packet.put(KEY_PHASE, phase);
        packet.put(KEY_STATUS, status);
        if (doneLine != null) packet.put(KEY_DONE, doneLine);
    }

    /** For benches inside EnhancedOpMode, whose per-loop packet carries these keys through its telemetry. */
    void addTo(Telemetry telemetry) {
        telemetry.addData(KEY_PHASE, phase);
        telemetry.addData(KEY_STATUS, status);
        if (doneLine != null) telemetry.addData(KEY_DONE, doneLine);
    }
}
