package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.biobuzz.modules.TransferCounter;
import org.firstinspires.ftc.teamcode.biobuzz.modules.TransferTiming;
import org.firstinspires.ftc.teamcode.opmodes.tele.BioBuzzBasicTele;

import java.util.Locale;

@TeleOp(name = "Transfer Thread Count Test", group = "Test")
@Config("Transfer Thread Count Test")
public class TransferThreadCountTest extends OpMode {
    public static String upstreamSensorName = "frontRangefinder";
    public static String downstreamSensorName = "backRangefinder";
    public static String colorSensorName = "colorRangefinder";
    public static String presenceSensorName = "backRangefinder";
    public static boolean usePresence = false;
    public static String motor1Name = "transfer";
    public static String servoName = "melonbotics";
    public static String intakeName = "intake";
    public static double maxRangeMm = 1300;
    public static double diffThresholdMm = 25;
    public static double hysteresisMm = 10;
    public static boolean oneSensor = false;
    public static double oneSensorThresholdMm = 100;
    public static double redHueLowDeg = 10;
    public static double redHueHighDeg = 50;
    public static double blueHueLowDeg = 170;
    public static double blueHueHighDeg = 210;
    public static double pollenHueLowDeg = 70;
    public static double pollenHueHighDeg = 89;
    public static double noSignalBelowDeg = 2;
    public static boolean runThread = true;
    public static double pollMs = 10;
    public static double loopMs = 20;
    public static int stopAtCount = 2;
    public static double motor1Power = 0;
    public static double intakePower = 0;
    public static boolean reverseIntake = true;
    public static boolean resetCount = false;
    public static boolean crashInLoop = false;
    public static boolean crashInStop = false;

    static final String NECTAR_RED = "nectar red";
    static final String NECTAR_BLUE = "nectar blue";
    static final String POLLEN = "pollen";
    static final String NONE = "none";
    static final String NO_SIGNAL = "no signal";

    private final ElapsedTime runtime = new ElapsedTime();

    AnalogInput upstream;
    AnalogInput downstream;
    AnalogInput colorSensor;
    AnalogInput presence;
    DcMotorEx motor1;
    DcMotorEx intake;
    DcMotorEx fl, bl, fr, br;
    CRServo servo;
    TransferCounter counter;
    final TransferTiming loopTiming = new TransferTiming();

    int loopCount;
    int lastPosition = 0;
    boolean lastResetCount;
    int talliedCount;
    int nectarCount;
    int pollenCount;
    int otherCount;
    String lastBallColour = NONE;

    @Override
    public void init() {
        upstream = hardwareMap.analogInput.get(upstreamSensorName);
        downstream = hardwareMap.analogInput.get(downstreamSensorName);
        colorSensor = hardwareMap.analogInput.get(colorSensorName);
        presence = hardwareMap.analogInput.get(presenceSensorName);
        motor1 = hardwareMap.get(DcMotorEx.class, motor1Name);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotorEx.class, intakeName);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo = hardwareMap.get(CRServo.class, servoName);
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");
        for (DcMotorEx motor : new DcMotorEx[] {fl, bl, fr, br}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (runThread) {
            counter = new TransferCounter(hardwareMap, upstreamSensorName, downstreamSensorName, maxRangeMm, pollMs);
            counter.setUpstreamOnly(oneSensor);
            counter.setThreshold(threshold(), hysteresisMm);
        }
        lastResetCount = resetCount;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        if (counter != null) counter.start();
    }

    @Override
    public void loop() {
        if (crashInLoop) throw new IllegalStateException("crashInLoop is on: deliberate crash in loop() to test the TransferCounter thread");
        runtime.reset();
        lastPosition = motor1.getCurrentPosition();
        double threshold = threshold();
        if (counter != null) {
            counter.setUpstreamOnly(oneSensor);
            counter.setThreshold(threshold, hysteresisMm);
        }

        double mmA = upstream.getVoltage() / 3.3 * maxRangeMm;
        double mmB = downstream.getVoltage() / 3.3 * maxRangeMm;
        double diff = mmA - mmB;
        if (loopTiming.update(System.nanoTime(), oneSensor ? mmA : diff, oneSensor, threshold, hysteresisMm)) loopCount++;
        double colorVolts = colorSensor.getVoltage();
        double hueDeg = colorVolts / 3.3 * 360;
        boolean present = presence.getVoltage() > 1.65;
        String colour = usePresence && !present ? NONE : ballColour(hueDeg);

        if (resetCount != lastResetCount) {
            loopCount = 0;
            loopTiming.reset();
            if (counter != null) counter.resetCount();
            talliedCount = 0;
            nectarCount = 0;
            pollenCount = 0;
            otherCount = 0;
            lastBallColour = NONE;
        }
        lastResetCount = resetCount;
        int count = counter != null ? counter.count() : loopCount;
        for (; talliedCount < count; talliedCount++) tally(colour);
        boolean stopped = stopAtCount > 0 && count >= stopAtCount;

        motor1.setPower(stopped ? 0 : motor1Power);
        servo.setPower(stopped ? 0 : motor1Power);
        intake.setDirection(direction(reverseIntake));
        intake.setPower(stopped ? 0 : intakePower);

        BioBuzzBasicTele.Drive drive = BioBuzzBasicTele.drive;
        double[] wheels = BioBuzzBasicTele.mecanum(
                BioBuzzBasicTele.deadband(-gamepad1.left_stick_y, drive.stickDeadband),
                BioBuzzBasicTele.deadband(gamepad1.left_stick_x, drive.stickDeadband),
                BioBuzzBasicTele.deadband(gamepad1.right_stick_x, drive.stickDeadband),
                Range.clip(gamepad1.right_bumper ? drive.slowPower : drive.maxPower, 0, 1));
        fl.setDirection(direction(drive.reverseFl));
        bl.setDirection(direction(drive.reverseBl));
        fr.setDirection(direction(drive.reverseFr));
        br.setDirection(direction(drive.reverseBr));
        fl.setPower(wheels[0]);
        bl.setPower(wheels[1]);
        fr.setPower(wheels[2]);
        br.setPower(wheels[3]);
        double workMs = runtime.milliseconds();

        while (runtime.milliseconds() < loopMs) {

        }

        telemetry.addData("Motor", stopped ? "STOPPED at count " + stopAtCount + ": flip resetCount to run again" : "running");
        telemetry.addData("Count (thread)", counter != null ? count : "off");
        telemetry.addData("Count (loop)", loopCount);
        telemetry.addData("Upstream (mm)", mmA);
        telemetry.addData("Downstream (mm)", mmB);
        telemetry.addData("A - B (mm)", diff);
        telemetry.addData("Diff threshold (mm)", diffThresholdMm);
        telemetry.addData("Hue (deg)", hueDeg);
        telemetry.addData("Colour sensor distance (mm), pin 0 in DISTANCE mode", colorVolts / 3.3 * 100);
        telemetry.addData("Ball present (1 yes, 0 no)", present ? 1 : 0);
        telemetry.addData("Ball colour", colour);
        telemetry.addData("Nectar count", nectarCount);
        telemetry.addData("Pollen count", pollenCount);
        telemetry.addData("Other count", otherCount);
        telemetry.addData("Last ball colour", lastBallColour);
        telemetry.addData("Counting on", oneSensor
                ? "Upstream alone, each time it rises above " + oneSensorThresholdMm + " mm (a ball leaving)"
                : "A - B rising above the diff threshold");
        if (counter != null) {
            TransferCounter.Stats s = counter.stats();
            if (s != null) {
                telemetry.addData("A - B thread (mm)", s.mmA - s.mmB);
                telemetry.addData("poll Hz", s.polls / s.seconds);
                telemetry.addData("new A values/s", s.newValues / s.seconds);
                telemetry.addData(String.format("late polls (>%.1f ms)", counter.latePollMs()), s.latePolls);
                telemetry.addData("max poll gap (ms)", s.maxGapMs);
                telemetry.addData("bulk read mean (ms)", s.meanCommandMs);
            }
            addTiming(counter.timing(), "the thread, at each " + ms(counter.pollMs()) + " ms poll");
        } else {
            addTiming(loopTiming.snapshot(), "the loop, once per loop");
        }
        telemetry.addData("Motor Current (A)", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake Current (A)", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("loop work (ms)", workMs);
        telemetry.addData("loop time (ms)", runtime.milliseconds());
        telemetry.addData("motor RPM", 60000 * (motor1.getCurrentPosition()-lastPosition) / runtime.milliseconds() / 145.1);
    }

    private void addTiming(TransferTiming.Snapshot t, String source) {
        telemetry.addData("Ball timing from", source + (t == null ? ": no poll since INIT or resetCount yet" : ""));
        if (t == null) return;
        telemetry.addData("Ball-to-ball period, last (ms)", ms(t.lastPeriodMs));
        telemetry.addData("Ball-to-ball period, shortest (ms)", ms(t.shortestPeriodMs));
        telemetry.addData("Ball-to-ball period, mean (ms)", ms(t.meanPeriodMs));
        telemetry.addData("Ball-to-ball periods timed since reset", t.periods);
        StringBuilder recent = new StringBuilder();
        for (double p : t.recentPeriodsMs()) recent.append(recent.length() == 0 ? "" : ", ").append(ms(p));
        telemetry.addData("Last " + TransferTiming.RECENT + " periods, oldest first (ms)",
                recent.length() == 0 ? "none yet" : recent);
        String armed = t.upstreamOnly ? "Ball in front of the beam" : "From re-arm until a ball leaves A";
        String rearm = t.upstreamOnly ? "Gap between balls at the beam"
                : "Ball's back edge from A to B, or until the next ball reaches A";
        telemetry.addData(armed + ", last (ms)", ms(t.lastArmedMs));
        telemetry.addData(armed + ", shortest (ms)", ms(t.shortestArmedMs));
        telemetry.addData(rearm + ", last (ms)", ms(t.lastRearmMs));
        telemetry.addData(rearm + ", shortest (ms)", ms(t.shortestRearmMs));
        telemetry.addData("Ball timing accuracy", "about one rangefinder update (20 ms at a 20 ms timing budget, "
                + "10 ms at 10 ms) or one " + (counter == null ? "loop" : "poll") + ", whichever is longer");
    }

    static String ms(double value) {
        return Double.isNaN(value) ? "none yet" : String.format(Locale.US, "%.1f", value);
    }

    private static DcMotorSimple.Direction direction(boolean reverse) {
        return reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static double threshold() {
        return oneSensor ? oneSensorThresholdMm : diffThresholdMm;
    }

    static boolean hueInRange(double hueDeg, double lowDeg, double highDeg) {
        return lowDeg <= highDeg ? hueDeg >= lowDeg && hueDeg <= highDeg : hueDeg >= lowDeg || hueDeg <= highDeg;
    }

    static String ballColour(double hueDeg) {
        // 0 V is also what an unplugged input or the connector's other channel reads, and it falls in the red range.
        if (hueDeg < noSignalBelowDeg) return NO_SIGNAL;
        if (hueInRange(hueDeg, redHueLowDeg, redHueHighDeg)) return NECTAR_RED;
        if (hueInRange(hueDeg, blueHueLowDeg, blueHueHighDeg)) return NECTAR_BLUE;
        if (hueInRange(hueDeg, pollenHueLowDeg, pollenHueHighDeg)) return POLLEN;
        return NONE;
    }

    private void tally(String colour) {
        lastBallColour = colour;
        if (colour.equals(NECTAR_RED) || colour.equals(NECTAR_BLUE)) nectarCount++;
        else if (colour.equals(POLLEN)) pollenCount++;
        else otherCount++;
    }

    @Override
    public void stop() {
        motor1.setPower(0);
        servo.setPower(0);
        intake.setPower(0);
        for (DcMotorEx motor : new DcMotorEx[] {fl, bl, fr, br}) motor.setPower(0);
        if (crashInStop) throw new IllegalStateException("crashInStop is on: deliberate crash in stop() before stopping the TransferCounter thread");
        if (counter != null) counter.stop();
    }
}
