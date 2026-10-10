package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * One bench's results file plus the dashboard keys run_bench.py watches: "Bench phase", "Bench status" and, once
 * the results file is final, "BENCH DONE". The file is rewritten at every phase end, so a crash keeps what finished.
 */
final class BenchReport {
    static final String KEY_PHASE = "Bench phase";
    static final String KEY_STATUS = "Bench status";
    static final String KEY_DONE = "BENCH DONE";

    private static final long PACKET_INTERVAL_NS = 100_000_000L;

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
        sendNow();
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
        sendNow();
    }

    boolean isDone() {
        return doneLine != null;
    }

    /** For benches outside EnhancedOpMode: sends the keys straight to slothboard, at most every 100 ms. */
    void sendIfDue() {
        if (System.nanoTime() - lastPacketNs >= PACKET_INTERVAL_NS) sendNow();
    }

    private void sendNow() {
        lastPacketNs = System.nanoTime();
        TelemetryPacket p = new TelemetryPacket(false);
        addTo(p);
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
