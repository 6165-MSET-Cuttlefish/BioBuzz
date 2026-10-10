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

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Bench: Write Recovery", group = "Test")
public class WriteRecoveryBench extends OpMode {
    static final double POWER = 0.2;
    static final double POSITION = 0.4;
    private static final double SETTLE_S = 1;
    private static final double RECOVER_S = 2;
    private static final double POWER_TOLERANCE = 0.01;
    private static final double PULSE_TOLERANCE_US = 5;
    private static final int MAX_ROWS = 1_000;

    private final class Channel {
        final String name;
        final boolean wrapped;
        final boolean motor;
        final int port;
        final double target;
        final double tolerance;
        double value;
        boolean enabled;
        Boolean settled;
        double recoveredMs = Double.NaN;
        long loopsAfter;
        long matchedAfter;

        Channel(String name, boolean wrapped, boolean motor, int port, double target, double tolerance) {
            this.name = name;
            this.wrapped = wrapped;
            this.motor = motor;
            this.port = port;
            this.target = target;
            this.tolerance = tolerance;
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

        void afterFailSafe(double ms) {
            loopsAfter++;
            if (!atTarget()) return;
            matchedAfter++;
            if (Double.isNaN(recoveredMs)) recoveredMs = ms;
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("wrapped", wrapped);
            m.put("kind", motor ? "motor" : "servo");
            m.put("port", port);
            m.put("target", target);
            m.put("settledBeforeFailSafe", settled);
            m.put("recoveredMs", recoveredMs);
            m.put("loopsAfterFailSafe", loopsAfter);
            m.put("loopsAtTargetAfterFailSafe", matchedAfter);
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
    private final Samples loopPeriods = new Samples();
    private long startNs;
    private long failSafeNs;
    private long lastLoopNs;

    @Override
    public void init() {
        report = new BenchReport("Write Recovery", "write_recovery");
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
        channels.add(new Channel("fl", true, true, wrappedMotor.getPortNumber(), POWER, POWER_TOLERANCE));
        channels.add(new Channel("fr", false, true, rawMotor.getPortNumber(), POWER, POWER_TOLERANCE));
        channels.add(new Channel("benchServo", false, false, rawServo.getPortNumber(), pulse(rawServo.getPwmRange()),
                PULSE_TOLERANCE_US));
        channels.add(new Channel("benchServo2", true, false, wrappedServo.getPortNumber(),
                pulse(wrappedServo.getPwmRange()), PULSE_TOLERANCE_US));

        Map<String, Object> info = new LinkedHashMap<>();
        info.put("batteryVolts", hardwareMap.voltageSensor.iterator().next().getVoltage());
        info.put("power", POWER);
        info.put("servoPosition", POSITION);
        info.put("settleSeconds", SETTLE_S);
        info.put("recoverSeconds", RECOVER_S);
        report.put("info", info);
        report.put("rowColumns", new String[] {"ms", "fl", "flEnabled", "fr", "frEnabled", "benchServo",
                "benchServoEnabled", "benchServo2", "benchServo2Enabled"});
        report.put("rows", rows);
    }

    private static double pulse(PwmControl.PwmRange range) {
        return range.usPulseLower + POSITION * (range.usPulseUpper - range.usPulseLower);
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
        if (lastLoopNs != 0) loopPeriods.addNanos(now - lastLoopNs);
        lastLoopNs = now;
        if (report.isDone()) {
            report.sendIfDue();
            return;
        }
        wrappedMotor.setPower(POWER);
        rawMotor.setPower(POWER);
        rawServo.setPosition(POSITION);
        wrappedServo.setPosition(POSITION);
        for (Channel c : channels) c.read();
        record(now);
        if (failSafeNs == 0) {
            if ((now - startNs) / 1e9 >= SETTLE_S) {
                for (Channel c : channels) c.settled = c.atTarget();
                HubRegisters.failSafe(hub);
                failSafeNs = System.nanoTime();
                report.phase("recover");
            }
        } else {
            double ms = (now - failSafeNs) / 1e6;
            for (Channel c : channels) c.afterFailSafe(ms);
            report.status("%.0f ms after failSafe", ms);
            if (ms >= RECOVER_S * 1000) finish();
        }
        report.sendIfDue();
    }

    private void record(long now) {
        if (rows.size() >= MAX_ROWS) return;
        double[] row = new double[1 + 2 * channels.size()];
        row[0] = (now - startNs) / 1e6;
        for (int i = 0; i < channels.size(); i++) {
            row[1 + 2 * i] = channels.get(i).value;
            row[2 + 2 * i] = channels.get(i).enabled ? 1 : 0;
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
            String recovered = Double.isNaN(c.recoveredMs) ? "never" : String.format("%.0f ms", c.recoveredMs);
            summary.append(c.name).append(' ').append(recovered).append("; ");
        }
        report.put("channels", result);
        report.put("loopPeriodMs", loopPeriods.summary());
        report.done("recovery after failSafe: " + summary);
    }

    @Override
    public void stop() {
        wrappedMotor.stop();
        rawMotor.setPower(0);
    }
}
