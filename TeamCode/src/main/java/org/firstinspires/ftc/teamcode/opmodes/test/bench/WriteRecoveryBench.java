package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Write Recovery", group = "Test")
public class WriteRecoveryBench extends OpMode {
    static final double POWER = 0.2;
    static final double POSITION = 0.4;
    private static final double NUDGE = 0.002;
    private static final double SETTLE_S = 1;
    private static final double[] PHASES_MS = {0, 40, 80, 120, 160, 200, 235};
    private static final double RECOVER_MS = 1000;
    private static final double HOLD_MS = 300;
    private static final double POWER_TOLERANCE = 0.01;
    private static final double PULSE_TOLERANCE_US = 5;
    private static final int MAX_ROWS = 2_000;

    private enum Step { SETTLE, WAIT_PHASE, RECOVER, HOLD }

    private final class Channel {
        final String name;
        final boolean wrapped;
        final boolean motor;
        final int port;
        final double tolerance;
        final PwmControl.PwmRange range;
        double target;
        double value;
        boolean enabled;
        Boolean settled;
        double recoveredMs;
        long recoveredLoops;
        long loopsAfter;
        long matchedAfter;
        double worstMs = Double.NaN;
        double worstPhaseMs = Double.NaN;
        long worstLoops;
        int never;

        Channel(String name, boolean wrapped, boolean motor, int port, double tolerance, PwmControl.PwmRange range) {
            this.name = name;
            this.wrapped = wrapped;
            this.motor = motor;
            this.port = port;
            this.tolerance = tolerance;
            this.range = range;
        }

        void command(double power, double position) {
            target = motor ? power : pulse(range, position);
        }

        void read() {
            if (motor) {
                value = Math.abs(HubRegisters.motorPower(hub, port));
                enabled = HubRegisters.motorEnabled(hub, port);
            } else {
                value = HubRegisters.servoPulseWidth(hub, port);
                enabled = HubRegisters.servoEnabled(hub, port);
            }
        }

        boolean atTarget() {
            return enabled && Math.abs(value - target) <= tolerance;
        }

        void failedSafe() {
            recoveredMs = Double.NaN;
            recoveredLoops = 0;
            loopsAfter = 0;
            matchedAfter = 0;
        }

        boolean afterFailSafe(double ms, long loops) {
            loopsAfter++;
            if (!atTarget()) return !Double.isNaN(recoveredMs);
            matchedAfter++;
            if (Double.isNaN(recoveredMs)) {
                recoveredMs = ms;
                recoveredLoops = loops;
            }
            return true;
        }

        Map<String, Object> endTrial(double phaseMs) {
            if (Double.isNaN(recoveredMs)) {
                never++;
            } else if (Double.isNaN(worstMs) || recoveredMs > worstMs) {
                worstMs = recoveredMs;
                worstPhaseMs = phaseMs;
                worstLoops = recoveredLoops;
            }
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("recoveredMs", recoveredMs);
            m.put("recoveredLoops", Double.isNaN(recoveredMs) ? null : recoveredLoops);
            m.put("loopsAfterFailSafe", loopsAfter);
            m.put("loopsAtTargetAfterFailSafe", matchedAfter);
            return m;
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("wrapped", wrapped);
            m.put("kind", motor ? "motor" : "servo");
            m.put("port", port);
            m.put("settledBeforeFailSafe", settled);
            m.put("worstRecoveredMs", worstMs);
            m.put("worstPhaseMs", worstPhaseMs);
            m.put("worstRecoveredLoops", Double.isNaN(worstMs) ? null : worstLoops);
            m.put("neverRecovered", never);
            m.put("lastValue", value);
            m.put("lastEnabled", enabled);
            return m;
        }
    }

    private BenchReport report;
    private LynxModule hub;
    private EnhancedMotor wrappedMotor;
    private DcMotorEx rawMotor;
    private ServoImplEx rawServo;
    private EnhancedServo wrappedServo;
    private final List<Channel> channels = new ArrayList<>();
    private final List<double[]> rows = new ArrayList<>();
    private final List<Map<String, Object>> trials = new ArrayList<>();
    private final Samples loopPeriods = new Samples();
    private Step step = Step.SETTLE;
    private int trial = -1;
    private double power = POWER;
    private double position = POSITION;
    private long startNs;
    private long writtenNs;
    private long failSafeNs;
    private long holdUntilNs;
    private long loopsSinceFailSafe;
    private double phaseMs;

    @Override
    public void init() {
        report = new BenchReport("Write Recovery", "write_recovery");
        PhotonSetup.fromParams(report);
        hub = HubRegisters.controlHub(hardwareMap, "Bench: Write Recovery");
        BenchIO.require(hardwareMap, DcMotorEx.class, "fl", BenchIO.RUNNER_HINT);
        BenchIO.require(hardwareMap, ServoImplEx.class, "benchServo2", BenchIO.RUNNER_HINT);
        wrappedMotor = new EnhancedMotor(hardwareMap, "fl");
        rawMotor = BenchIO.require(hardwareMap, DcMotorEx.class, "fr", BenchIO.RUNNER_HINT);
        rawServo = BenchIO.require(hardwareMap, ServoImplEx.class, "benchServo", BenchIO.RUNNER_HINT);
        wrappedServo = new EnhancedServo(hardwareMap, "benchServo2");
        for (DcMotorEx m : new DcMotorEx[] {wrappedMotor, rawMotor}) {
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        rawServo.setDirection(Servo.Direction.FORWARD);
        wrappedServo.setDirection(Servo.Direction.FORWARD);
        channels.add(new Channel("fl", true, true, wrappedMotor.getPortNumber(), POWER_TOLERANCE, null));
        channels.add(new Channel("fr", false, true, rawMotor.getPortNumber(), POWER_TOLERANCE, null));
        channels.add(new Channel("benchServo", false, false, rawServo.getPortNumber(), PULSE_TOLERANCE_US,
                rawServo.getPwmRange()));
        channels.add(new Channel("benchServo2", true, false, wrappedServo.getPortNumber(), PULSE_TOLERANCE_US,
                wrappedServo.getPwmRange()));
        for (Channel c : channels) c.command(power, position);

        Map<String, Object> info = new LinkedHashMap<>();
        info.put("batteryVolts", hardwareMap.voltageSensor.iterator().next().getVoltage());
        info.put("power", POWER);
        info.put("servoPosition", POSITION);
        info.put("nudge", NUDGE);
        info.put("settleSeconds", SETTLE_S);
        info.put("phasesMs", PHASES_MS);
        info.put("recoverMs", RECOVER_MS);
        info.put("holdMs", HOLD_MS);
        info.put("resendMs", resendMs());
        report.put("info", info);
        report.put("rowColumns", new String[] {"ms", "failSafe", "fl", "flEnabled", "fr", "frEnabled", "benchServo",
                "benchServoEnabled", "benchServo2", "benchServo2Enabled"});
        report.put("rows", rows);
        report.put("trials", trials);
    }

    private static double resendMs() {
        try {
            Field f = Class.forName("org.firstinspires.ftc.teamcode.architecture.hardware.WriteCache")
                    .getDeclaredField("RESEND_NANOS");
            f.setAccessible(true);
            return f.getLong(null) / 1e6;
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("can't read WriteCache.RESEND_NANOS", e);
        }
    }

    private static double pulse(PwmControl.PwmRange range, double position) {
        return range.usPulseLower + position * (range.usPulseUpper - range.usPulseLower);
    }

    @Override
    public void init_loop() {
        report.sendIfDue();
    }

    @Override
    public void start() {
        startNs = System.nanoTime();
        report.phase("settle");
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        loopPeriods.tick(now);
        if (report.isDone()) {
            report.sendIfDue();
            return;
        }
        boolean nudged = false;
        if (step == Step.SETTLE && (now - startNs) / 1e9 >= SETTLE_S) {
            for (Channel c : channels) c.settled = c.atTarget();
            nudged = nextTrial();
        } else if (step == Step.HOLD && now >= holdUntilNs) {
            endTrial();
            if (!nextTrial()) {
                finish();
                report.sendIfDue();
                return;
            }
            nudged = true;
        }
        wrappedMotor.setPower(power);
        rawMotor.setPower(power);
        rawServo.setPosition(position);
        wrappedServo.setPosition(position);
        long afterWrites = System.nanoTime();
        if (nudged) writtenNs = afterWrites;
        boolean failedSafeNow = false;
        if (step == Step.WAIT_PHASE && (afterWrites - writtenNs) / 1e6 >= PHASES_MS[trial]) {
            phaseMs = (afterWrites - writtenNs) / 1e6;
            HubRegisters.failSafe(hub);
            failSafeNs = System.nanoTime();
            loopsSinceFailSafe = 0;
            for (Channel c : channels) c.failedSafe();
            step = Step.RECOVER;
            failedSafeNow = true;
        }
        for (Channel c : channels) c.read();
        long readNs = System.nanoTime();
        record(readNs);
        if ((step == Step.RECOVER || step == Step.HOLD) && !failedSafeNow) {
            loopsSinceFailSafe++;
            double ms = (readNs - failSafeNs) / 1e6;
            boolean all = true;
            for (Channel c : channels) all &= c.afterFailSafe(ms, loopsSinceFailSafe);
            if (step == Step.RECOVER && (all || ms >= RECOVER_MS)) {
                step = Step.HOLD;
                holdUntilNs = readNs + (long) (HOLD_MS * 1e6);
            }
        }
        report.status("failSafe %d of %d: %s", trial + 1, PHASES_MS.length, step);
        report.sendIfDue();
    }

    private boolean nextTrial() {
        if (trial + 1 >= PHASES_MS.length) return false;
        trial++;
        double nudge = trial % 2 == 0 ? NUDGE : 0;
        power = POWER + nudge;
        position = POSITION + nudge;
        for (Channel c : channels) c.command(power, position);
        step = Step.WAIT_PHASE;
        report.phase("failSafe " + (trial + 1));
        return true;
    }

    private void endTrial() {
        Map<String, Object> t = new LinkedHashMap<>();
        t.put("trial", trial + 1);
        t.put("targetPhaseMs", PHASES_MS[trial]);
        t.put("phaseMs", phaseMs);
        t.put("power", power);
        t.put("position", position);
        Map<String, Object> byChannel = new LinkedHashMap<>();
        for (Channel c : channels) byChannel.put(c.name, c.endTrial(phaseMs));
        t.put("channels", byChannel);
        trials.add(t);
    }

    private void record(long now) {
        if (rows.size() >= MAX_ROWS) return;
        double[] row = new double[2 + 2 * channels.size()];
        row[0] = (now - startNs) / 1e6;
        row[1] = trial + 1;
        for (int i = 0; i < channels.size(); i++) {
            row[2 + 2 * i] = channels.get(i).value;
            row[3 + 2 * i] = channels.get(i).enabled ? 1 : 0;
        }
        rows.add(row);
    }

    private void finish() {
        wrappedMotor.stop();
        rawMotor.setPower(0);
        Map<String, Object> result = new LinkedHashMap<>();
        StringBuilder summary = new StringBuilder();
        for (Channel c : channels) {
            result.put(c.name, c.toMap());
            String worst = Double.isNaN(c.worstMs) ? "never back" : String.format("worst %.0f ms", c.worstMs);
            summary.append(c.name).append(' ').append(worst).append("; ");
        }
        report.put("channels", result);
        report.put("loopPeriodMs", loopPeriods.summary());
        report.done("recovery after " + trials.size() + " failSafes: " + summary);
    }

    @Override
    public void stop() {
        wrappedMotor.stop();
        rawMotor.setPower(0);
    }
}
