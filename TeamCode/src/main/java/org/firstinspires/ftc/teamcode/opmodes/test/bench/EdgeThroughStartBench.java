package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.input.EdgeBooleanSupplier;

import java.util.LinkedHashMap;
import java.util.Map;

@TeleOp(name = "Bench: Edge Through Start", group = "Test")
public class EdgeThroughStartBench extends EnhancedOpMode {
    static final int INIT_IDLE = 10;
    static final int INIT_HOLD = 11;
    static final int RUN_HOLD = 12;
    static final int RUN_RELEASE = 13;
    static final int RUN_PRESS = 14;
    static final int RUN_RELEASE_AGAIN = 15;

    public static class EdgeRobot extends Robot {
        public EdgeRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {}
    }

    private static final class Counts {
        final int code;
        final boolean running;
        long loops;
        long lbLoops;
        long lbRises;
        long lbFalls;
        long spentRises;
        long controlRises;

        Counts(int code, boolean running) {
            this.code = code;
            this.running = running;
        }

        void add(boolean lb, boolean lbRose, boolean lbFell, boolean spentRose, boolean controlRose) {
            loops++;
            if (lb) lbLoops++;
            if (lbRose) lbRises++;
            if (lbFell) lbFalls++;
            if (spentRose) spentRises++;
            if (controlRose) controlRises++;
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("code", code);
            m.put("running", running);
            m.put("loops", loops);
            m.put("lbLoops", lbLoops);
            m.put("lbRises", lbRises);
            m.put("lbFalls", lbFalls);
            m.put("spentRises", spentRises);
            m.put("controlRises", controlRises);
            return m;
        }

        @Override
        public String toString() {
            return key(code, running) + ": " + loops + " loops, " + lbLoops + " with LB, LB rose " + lbRises + " fell "
                    + lbFalls + ", invalidated supplier " + spentRises + " edges, control " + controlRises;
        }
    }

    private BenchReport report;
    private EdgeBooleanSupplier spent;
    private EdgeBooleanSupplier control;
    private final Map<String, Counts> counts = new LinkedHashMap<>();
    private Counts current;
    private int phaseCode;
    private boolean lb;
    private boolean lbAtConstruction;
    private Map<String, Object> firstRunLoop;
    private long loops;
    private int failedChecks;

    @Override
    protected Robot createRobot() {
        return new EdgeRobot(this);
    }

    @Override
    protected void initialize() {
        report = new BenchReport("Edge Through Start", "edge_start");
        spent = new EdgeBooleanSupplier(() -> gamepad1.left_bumper);
        control = new EdgeBooleanSupplier(() -> gamepad1.left_bumper);
        lbAtConstruction = gamepad1.left_bumper;
        lb = lbAtConstruction;
        report.put("lbHeldAtConstruction", lbAtConstruction);
        report.save();
    }

    // Only the BioBuzzTele-style supplier is refreshed in INIT; the control's first read is after START.
    @Override
    protected void initializeLoop() {
        spent.invalidate();
        sample(false);
    }

    @Override
    protected void gameLoop() {
        sample(true);
    }

    private void sample(boolean running) {
        loops++;
        int marker = Math.round(gamepad2.right_trigger * 100);
        if (marker != 0 && (current == null || marker != phaseCode || running != current.running)) begin(marker, running);
        boolean was = lb;
        lb = gamepad1.left_bumper;
        boolean spentRose = spent.wasJustPressed();
        boolean controlRose = running && control.wasJustPressed();
        if (running && firstRunLoop == null) {
            firstRunLoop = new LinkedHashMap<>();
            firstRunLoop.put("phase", phaseCode);
            firstRunLoop.put("lb", lb);
            firstRunLoop.put("spentRose", spentRose);
            firstRunLoop.put("controlRose", controlRose);
            report.put("firstRunLoop", firstRunLoop);
            report.save();
        }
        if (current != null) current.add(lb, lb && !was, !lb && was, spentRose, controlRose);
        if (phaseCode == GamepadProbe.DONE_PHASE && !report.isDone()) finish();
    }

    private static String key(int code, boolean running) {
        return "p" + code + (running ? "run" : "init");
    }

    private void begin(int code, boolean running) {
        if (current != null) report.endPhase(key(current.code, current.running), current.toMap());
        phaseCode = code;
        String key = key(code, running);
        current = counts.get(key);
        if (current == null) {
            current = new Counts(code, running);
            counts.put(key, current);
        }
        report.phase(key);
    }

    private Counts counts(int code, boolean running) {
        Counts c = counts.get(key(code, running));
        return c == null ? new Counts(code, running) : c;
    }

    private void finish() {
        report.endPhase(key(current.code, current.running), current.toMap());
        Counts holdInit = counts(INIT_HOLD, false);
        Counts holdAtStart = counts(INIT_HOLD, true);
        Counts hold = counts(RUN_HOLD, true);
        Counts release = counts(RUN_RELEASE, true);
        Counts press = counts(RUN_PRESS, true);
        Counts releaseAgain = counts(RUN_RELEASE_AGAIN, true);
        boolean firstLb = firstRunLoop != null && (Boolean) firstRunLoop.get("lb");
        boolean spentAtStart = firstRunLoop != null && (Boolean) firstRunLoop.get("spentRose");
        boolean controlAtStart = firstRunLoop != null && (Boolean) firstRunLoop.get("controlRose");
        Map<String, Object> checks = new LinkedHashMap<>();
        check(checks, "setup: LB released when the suppliers were built", !lbAtConstruction,
                "LB " + (lbAtConstruction ? "held" : "released") + " in initialize()");
        check(checks, "setup: the LB press reached INIT", holdInit.lbLoops > 0, holdInit.toString());
        check(checks, "setup: LB held unbroken from INIT through START and the hold", firstLb
                        && holdInit.lbFalls + holdAtStart.lbFalls + hold.lbFalls == 0
                        && holdAtStart.lbRises + hold.lbRises == 0,
                "first running loop LB " + firstLb + "; " + holdInit + "; " + holdAtStart + "; " + hold);
        boolean setupOk = failedChecks == 0;
        check(checks, "invalidated: no edge on the first loop after START", firstRunLoop != null && !spentAtStart,
                "first running loop " + firstRunLoop);
        check(checks, "invalidated: no edge through START and the hold", holdAtStart.spentRises + hold.spentRises == 0,
                holdAtStart + "; " + hold);
        check(checks, "invalidated: exactly one edge for the later press",
                release.spentRises == 0 && press.spentRises == 1 && releaseAgain.spentRises == 0,
                release + "; " + press + "; " + releaseAgain);
        check(checks, "control: exactly one edge for the later press", press.controlRises == 1, press.toString());
        report.put("checks", checks);
        report.put("controlEdgeAtStart", controlAtStart);
        report.put("verdict", !setupOk ? "inconclusive" : failedChecks == 0 ? "pass" : "fail");
        report.put("pass", setupOk && failedChecks == 0);
        report.put("loops", loops);
        report.done(!setupOk ? "setup failed: the LB hold broke or never reached INIT, so this says nothing about the fix"
                : failedChecks == 0 ? "invalidated supplier as expected" : failedChecks + " check(s) failed");
    }

    private void check(Map<String, Object> checks, String name, boolean pass, String detail) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("pass", pass);
        m.put("detail", detail);
        checks.put(name, m);
        if (!pass) failedChecks++;
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        telemetry.addData("gp phase", phaseCode);
        telemetry.addData("gp lb", lb);
        telemetry.addData(BenchReport.KEY_LOOP, loops);
    }
}
