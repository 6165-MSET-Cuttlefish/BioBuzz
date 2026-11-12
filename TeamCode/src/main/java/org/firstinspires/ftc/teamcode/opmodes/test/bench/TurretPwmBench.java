package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.modules.BettaTurret;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "Bench: Turret PWM", group = "Test")
public class TurretPwmBench extends EnhancedOpMode {
    static final String LEFT = "TurretL";
    static final String RIGHT = "TurretR";
    private static final String HINT = "run it through scripts/bench/run_bench.py, which writes and activates the turret "
            + "bench config";
    private static final double STEP_S = 0.6;
    private static final double PULSE_TOLERANCE_US = 5;

    private enum Step { CENTER, STOP_WRITES_OFF, RESUME, STOP_WRITES_ON, IDLE_STOP, RIGHT_FORWARD }

    public static class TurretRobot extends Robot {
        public BettaTurret turret;

        public TurretRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {
            BenchIO.require(opMode.hardwareMap, ServoImplEx.class, LEFT, HINT);
            BenchIO.require(opMode.hardwareMap, ServoImplEx.class, RIGHT, HINT);
            turret = new BettaTurret(opMode.hardwareMap);
        }
    }

    private static final class Reading {
        final boolean left;
        final boolean right;
        final int pulseLeft;
        final int pulseRight;
        final boolean sdkLeft;
        final boolean sdkRight;

        Reading(boolean left, boolean right, int pulseLeft, int pulseRight, boolean sdkLeft, boolean sdkRight) {
            this.left = left;
            this.right = right;
            this.pulseLeft = pulseLeft;
            this.pulseRight = pulseRight;
            this.sdkLeft = sdkLeft;
            this.sdkRight = sdkRight;
        }

        boolean bothOff() {
            return !left && !right;
        }

        boolean bothOn() {
            return left && right;
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("leftEnabled", left);
            m.put("rightEnabled", right);
            m.put("leftPulseUs", pulseLeft);
            m.put("rightPulseUs", pulseRight);
            m.put("sdkLeftEnabled", sdkLeft);
            m.put("sdkRightEnabled", sdkRight);
            return m;
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "L %s %d us, R %s %d us", left ? "on" : "off", pulseLeft,
                    right ? "on" : "off", pulseRight);
        }
    }

    private static final class Window {
        final String name;
        Reading entry;
        Reading first;
        Reading last;
        long loops;
        long loopsLeftOn;
        long loopsRightOn;
        long loopsSdkDisagrees;

        Window(String name) {
            this.name = name;
        }

        void add(Reading r) {
            if (first == null) first = r;
            last = r;
            loops++;
            if (r.left) loopsLeftOn++;
            if (r.right) loopsRightOn++;
            if (r.left != r.sdkLeft || r.right != r.sdkRight) loopsSdkDisagrees++;
        }

        boolean neverOn() {
            return loops > 0 && loopsLeftOn == 0 && loopsRightOn == 0;
        }

        Map<String, Object> toMap() {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("loops", loops);
            m.put("loopsLeftEnabled", loopsLeftOn);
            m.put("loopsRightEnabled", loopsRightOn);
            m.put("loopsSdkCacheDisagrees", loopsSdkDisagrees);
            m.put("afterEntryAction", entry == null ? null : entry.toMap());
            m.put("firstLoop", first == null ? null : first.toMap());
            m.put("lastLoop", last == null ? null : last.toMap());
            return m;
        }

        String describe() {
            return "after entry " + entry + "; first loop " + first + "; last loop " + last + "; " + loops + " loops, "
                    + loopsLeftOn + " with L on, " + loopsRightOn + " with R on";
        }
    }

    private TurretRobot bench;
    private BenchReport report;
    private LynxModule hub;
    private ServoImplEx left;
    private ServoImplEx right;
    private final Window initWindow = new Window("init");
    private final Map<Step, Window> windows = new EnumMap<>(Step.class);
    private Window current;
    private int stepIndex = -1;
    private long stepStartNs;
    private int failedChecks;

    @Override
    protected Robot createRobot() {
        bench = new TurretRobot(this);
        return bench;
    }

    @Override
    protected void initialize() {
        report = new BenchReport("Turret PWM", "turret_pwm");
        hub = HubRegisters.controlHub(hardwareMap, "Bench: Turret PWM");
        left = hardwareMap.get(ServoImplEx.class, LEFT);
        right = hardwareMap.get(ServoImplEx.class, RIGHT);
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("leftPort", left.getPortNumber());
        info.put("rightPort", right.getPortNumber());
        info.put("leftDirection", left.getDirection().name());
        info.put("rightDirection", right.getDirection().name());
        info.put("leftPwmRange", range(left.getPwmRange()));
        info.put("rightPwmRange", range(right.getPwmRange()));
        info.put("stepSeconds", STEP_S);
        report.put("info", info);
        report.save();
    }

    @Override
    protected void initializeLoop() {
        initWindow.add(read());
    }

    @Override
    protected void onStart() {
        report.put("init", initWindow.toMap());
        report.save();
    }

    @Override
    protected void gameLoop() {
        if (report.isDone()) return;
        long now = System.nanoTime();
        if (current != null && (now - stepStartNs) / 1e9 < STEP_S) {
            current.add(read());
            return;
        }
        if (current != null) report.endPhase(current.name, current.toMap());
        if (++stepIndex == Step.values().length) {
            finish();
            return;
        }
        Step step = Step.values()[stepIndex];
        current = new Window(step.name());
        windows.put(step, current);
        report.phase(step.name());
        enter(step);
        // Read before this loop's writeModules(), so it shows what the entry action alone did.
        current.entry = read();
        stepStartNs = now;
    }

    private void enter(Step step) {
        switch (step) {
            case CENTER:
                BettaTurret.TurretState.CENTER.activate();
                break;
            case STOP_WRITES_OFF:
                bench.turret.stop();
                bench.turret.setWriteEnabled(false);
                break;
            case RESUME:
                bench.turret.setWriteEnabled(true);
                break;
            case STOP_WRITES_ON:
                bench.turret.stop();
                break;
            case IDLE_STOP:
                BettaTurret.TurretState.IDLE.activate();
                bench.turret.stop();
                break;
            case RIGHT_FORWARD:
                BettaTurret.RightState.FORWARD.activate();
                break;
        }
    }

    private Reading read() {
        int l = left.getPortNumber();
        int r = right.getPortNumber();
        return new Reading(HubRegisters.servoEnabled(hub, l), HubRegisters.servoEnabled(hub, r),
                HubRegisters.servoPulseWidth(hub, l), HubRegisters.servoPulseWidth(hub, r), left.isPwmEnabled(),
                right.isPwmEnabled());
    }

    private void finish() {
        double centerLeft = pulse(left, BettaTurret.TurretState.CENTER.getValue());
        double centerRight = pulse(right, BettaTurret.TurretState.CENTER.getValue());
        double forwardRight = pulse(right, BettaTurret.RightState.FORWARD.getValue());
        Map<String, Object> checks = new LinkedHashMap<>();
        check(checks, "init: PWM off on both ports before any command", true, initWindow.neverOn(),
                initWindow.describe());
        Window w = windows.get(Step.CENTER);
        check(checks, "CENTER: PWM on at the center pulse on both ports", false,
                w.last != null && w.last.bothOn() && near(w.last.pulseLeft, centerLeft)
                        && near(w.last.pulseRight, centerRight),
                w.describe() + String.format(Locale.US, "; expected %.0f / %.0f us", centerLeft, centerRight));
        w = windows.get(Step.STOP_WRITES_OFF);
        check(checks, "stop() then writes off: PWM off right after stop() and for the whole step", false,
                w.entry.bothOff() && w.neverOn(), w.describe());
        w = windows.get(Step.RESUME);
        check(checks, "writes back on after a step off: PWM on again at the same position by the next loop", false,
                w.first != null && w.first.bothOn() && near(w.last.pulseLeft, centerLeft)
                        && near(w.last.pulseRight, centerRight), w.describe());
        w = windows.get(Step.STOP_WRITES_ON);
        check(checks, "stop() with writes on: PWM off right after stop(), on again the next loop", false,
                w.entry.bothOff() && w.first != null && w.first.bothOn(), w.describe());
        w = windows.get(Step.IDLE_STOP);
        check(checks, "IDLE then stop(): PWM stays off", false, w.entry.bothOff() && w.neverOn(), w.describe());
        w = windows.get(Step.RIGHT_FORWARD);
        check(checks, "RightState.FORWARD: TurretR on at the forward pulse, TurretL still off", false,
                w.last != null && w.last.right && !w.last.left && near(w.last.pulseRight, forwardRight),
                w.describe() + String.format(Locale.US, "; expected R %.0f us", forwardRight));
        report.put("expectedPulseUs", new double[] {centerLeft, centerRight, forwardRight});
        report.put("checks", checks);
        report.put("pass", failedChecks == 0);
        report.done(failedChecks == 0 ? "every PWM step as expected" : failedChecks + " PWM step(s) not as expected");
    }

    private void check(Map<String, Object> checks, String name, boolean informational, boolean pass, String detail) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("pass", pass);
        m.put("informational", informational);
        m.put("detail", detail);
        checks.put(name, m);
        if (!pass && !informational) failedChecks++;
    }

    private static boolean near(int hubPulse, double expected) {
        return Math.abs(hubPulse - expected) <= PULSE_TOLERANCE_US;
    }

    private static double pulse(ServoImplEx servo, double position) {
        double p = servo.getDirection() == Servo.Direction.REVERSE ? 1 - position : position;
        PwmControl.PwmRange r = servo.getPwmRange();
        return r.usPulseLower + p * (r.usPulseUpper - r.usPulseLower);
    }

    private static double[] range(PwmControl.PwmRange r) {
        return new double[] {r.usPulseLower, r.usPulseUpper, r.usFrame};
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
    }
}
