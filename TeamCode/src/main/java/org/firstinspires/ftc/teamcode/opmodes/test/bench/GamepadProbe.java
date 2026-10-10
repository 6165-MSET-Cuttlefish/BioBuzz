package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.architecture.input.EdgeBooleanSupplier;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

final class GamepadProbe {
    static final int DONE_PHASE = 99;
    private static final float STRONG = 0.9f;
    private static final float DELIVERY_VALUE = -0.5f;
    private static final int MAX_CHANGES = 6_000;
    private static final long PACKET_INTERVAL_NS = 50_000_000L;

    private static final class Phase {
        final int code;
        final boolean running;
        final double startMs;
        long loops;
        long deliveries;
        long deliveryValueLoops;
        long lbRises;
        long lbFalls;
        long ltRises;
        long strongNeg;
        long strongPos;
        long zeroBetweenStrong;
        private long zeroPending;
        private long fallPending;
        private boolean strongSeen;
        private long lastStrongDeliveryNs;
        private long restAfterNs;
        float minLy = Float.POSITIVE_INFINITY;
        float maxLy = Float.NEGATIVE_INFINITY;

        Phase(int code, boolean running, double startMs) {
            this.code = code;
            this.running = running;
            this.startMs = startMs;
        }

        void add(long now, float ly, boolean delivery, boolean lbRose, boolean lbFell, boolean ltRose) {
            loops++;
            minLy = Math.min(minLy, ly);
            maxLy = Math.max(maxLy, ly);
            if (delivery) deliveries++;
            if (ly == DELIVERY_VALUE) deliveryValueLoops++;
            if (lbRose) lbRises++;
            if (ltRose) ltRises++;
            if (lbFell) fallPending++;
            if (Math.abs(ly) >= STRONG) {
                if (ly < 0) strongNeg++;
                else strongPos++;
                zeroBetweenStrong += zeroPending;
                lbFalls += fallPending;
                zeroPending = 0;
                fallPending = 0;
                strongSeen = true;
                if (delivery) {
                    lastStrongDeliveryNs = now;
                    restAfterNs = 0;
                }
            } else if (ly == 0) {
                if (strongSeen) zeroPending++;
                if (lastStrongDeliveryNs != 0 && restAfterNs == 0) restAfterNs = now;
            }
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("code", code);
            m.put("running", running);
            m.put("startMs", startMs);
            m.put("loops", loops);
            m.put("deliveries", deliveries);
            m.put("deliveryValueLoops", deliveryValueLoops);
            m.put("lbRises", lbRises);
            m.put("lbFalls", lbFalls);
            m.put("ltRises", ltRises);
            m.put("strongNegLoops", strongNeg);
            m.put("strongPosLoops", strongPos);
            m.put("zeroStickLoops", zeroBetweenStrong);
            m.put("minLy", loops == 0 ? null : (double) minLy);
            m.put("maxLy", loops == 0 ? null : (double) maxLy);
            m.put("restAfterLastDeliveryMs", restAfterNs == 0 ? null : (restAfterNs - lastStrongDeliveryNs) / 1e6);
            return m;
        }
    }

    private final BenchReport report;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final EdgeBooleanSupplier headingLockToggle;
    private final EdgeBooleanSupplier slowToggle;
    private final Map<Integer, Phase> phases = new LinkedHashMap<>();
    private final List<double[]> changes = new ArrayList<>();
    private final long startNs = System.nanoTime();
    private Phase current;
    private int phaseCode;
    private int lastMarker;
    private int lastSeq;
    private long loops;
    private long lbEdges;
    private boolean headingLock;
    private boolean slowMode;
    private double[] lastChange;
    private long lastPacketNs;

    GamepadProbe(BenchReport report, Gamepad gamepad1, Gamepad gamepad2, String variant) {
        this.report = report;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        headingLockToggle = new EdgeBooleanSupplier(() -> gamepad1.left_bumper);
        slowToggle = new EdgeBooleanSupplier(() -> gamepad1.left_trigger > 0.1);
        report.put("variant", variant);
        report.save();
    }

    void sample(boolean running) {
        loops++;
        long now = System.nanoTime();
        float ly = gamepad1.left_stick_y;
        int marker = Math.round(gamepad2.right_trigger * 100);
        int seq = Math.round(gamepad2.left_trigger * 1000);
        boolean delivery = marker != 0 && (marker != lastMarker || seq != lastSeq);
        if (marker != 0) {
            lastMarker = marker;
            lastSeq = seq;
            if (marker != phaseCode) begin(marker, now, running);
        }
        boolean lbRose = headingLockToggle.wasJustPressed();
        boolean lbFell = headingLockToggle.wasJustReleased();
        boolean ltRose = slowToggle.wasJustPressed();
        if (lbRose) {
            headingLock = !headingLock;
            lbEdges++;
        }
        if (ltRose) slowMode = !slowMode;
        if (current != null) current.add(now, ly, delivery, lbRose, lbFell, ltRose);
        logChange(now, ly, marker, seq);
        if (phaseCode == DONE_PHASE && !report.isDone()) finish();
    }

    private void begin(int code, long now, boolean running) {
        if (current != null) report.endPhase("p" + current.code, current.toMap());
        phaseCode = code;
        current = phases.get(code);
        if (current == null) {
            current = new Phase(code, running, (now - startNs) / 1e6);
            phases.put(code, current);
        }
        report.phase("p" + code);
    }

    private void logChange(long now, float ly, int marker, int seq) {
        double[] row = {(now - startNs) / 1e6, loops, ly, gamepad1.left_stick_x, gamepad1.left_bumper ? 1 : 0,
                gamepad1.left_trigger, marker, seq};
        if (lastChange != null) {
            boolean same = true;
            for (int i = 2; i < row.length && same; i++) same = row[i] == lastChange[i];
            if (same) return;
        }
        lastChange = row;
        if (changes.size() < MAX_CHANGES) changes.add(row);
    }

    private void finish() {
        report.put("loops", loops);
        report.put("headingLockToggles", lbEdges);
        report.put("headingLock", headingLock);
        report.put("slowMode", slowMode);
        report.put("changeColumns", new String[] {"ms", "loop", "ly", "lx", "lb", "lt", "phase", "seq"});
        report.put("changes", changes);
        report.done(phases.size() + " phases in " + loops + " loops");
    }

    void put(BiConsumer<String, Object> out) {
        long zeroStick = 0;
        for (Phase p : phases.values()) zeroStick += p.zeroBetweenStrong;
        out.accept("gp ly", String.format("%.3f", gamepad1.left_stick_y));
        out.accept("gp lb", gamepad1.left_bumper);
        out.accept("gp phase", phaseCode);
        out.accept("lb edges", lbEdges);
        out.accept("zero-stick loops", zeroStick);
        out.accept("Bench loop", loops);
    }

    void sendIfDue() {
        long now = System.nanoTime();
        if (now - lastPacketNs < PACKET_INTERVAL_NS) return;
        lastPacketNs = now;
        TelemetryPacket p = new TelemetryPacket(false);
        report.addTo(p);
        put(p::put);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
