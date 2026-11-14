package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayDeque;
import java.util.List;

/**
 * Drive, intake and shoot from gamepad 1 with no odometry, cameras or turret control. Every number is a dashboard
 * setting; the gamepad map is printed on the Driver Station.
 */
@TeleOp(name = "BioBuzz Basic Tele", group = "A")
@Config("BioBuzz Basic Tele")
public class BioBuzzBasicTele extends OpMode {
    public static class Drive {
        public double maxPower = 0.6;
        public double slowPower = 0.3;
        public double stickDeadband = 0.05;
        public boolean reverseFl = true;
        public boolean reverseBl = true;
        public boolean reverseFr = false;
        public boolean reverseBr = false;
    }

    public static class Flywheel {
        public double targetRpm = 0;
        public double power = 0.2;
        public double kV = 0;
        public double kP = 0;
        public double hoodPosition = 0.5;
        public boolean reverseFlywheel = false;
        public boolean reverseEncoder = false;
    }

    public static class Intake {
        public double power = 1.0;
        public double outtakePower = 1.0;
        public boolean reverse = false;
        public double triggerThreshold = 0.1;
    }

    public static class Transfer {
        public double feedPower = 0.8;
        public double intakePower = 0;
        public boolean reverse = false;
        public double servoFeedPower = 1;
        public boolean reverseServo = false;
        public double blockerOpenMs = 150;
    }

    public static class Blocker {
        public double openPosition = 0.6;
        public double closedPosition = 0.53;
    }

    public static Drive drive = new Drive();
    public static Flywheel pollen = new Flywheel();
    public static Flywheel nectar = new Flywheel();
    public static Intake intake = new Intake();
    public static Transfer transfer = new Transfer();
    public static Blocker blocker = new Blocker();

    public static double ticksPerRev = 8192;
    public static double rpmWindowMs = 50;
    public static double rpmStep = 100;
    public static double powerStep = 0.05;

    private final ArrayDeque<double[]> pollenHistory = new ArrayDeque<>();
    private final ArrayDeque<double[]> nectarHistory = new ArrayDeque<>();
    private List<LynxModule> hubs;
    private DcMotorEx fl, bl, fr, br;
    private DcMotorEx pollenFlywheel, nectarFlywheel;
    private DcMotorEx intakeMotor, transferMotor;
    private ServoImplEx pollenHood, nectarHood, blockerServo;
    private CRServoImplEx transferServo;
    private VoltageSensor battery;
    private boolean pollenOn, nectarOn;
    private boolean lastA, lastX, lastY, lastUp, lastDown;
    private long lastLoopNs;
    private long flywheelsOnNs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        if (hubs.size() != 2) {
            throw new IllegalStateException("Expected the Control Hub and the Expansion Hub, found " + hubs.size()
                    + " hub(s). Check the Expansion Hub's RS485 cable and power, and that biobuzz is the active config.");
        }
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");
        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "PollenShooter");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "NectarShooter");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        pollenHood = hardwareMap.get(ServoImplEx.class, "PollenHood");
        nectarHood = hardwareMap.get(ServoImplEx.class, "NectarHood");
        blockerServo = hardwareMap.get(ServoImplEx.class, "hardstop");
        transferServo = hardwareMap.get(CRServoImplEx.class, "transferServo");
        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        for (DcMotorEx motor : new DcMotorEx[] {fl, bl, fr, br}) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        for (DcMotorEx motor : new DcMotorEx[] {pollenFlywheel, nectarFlywheel, intakeMotor, transferMotor}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Nothing moves until START. The turret servos are never powered.");
        addGamepadMap();
    }

    @Override
    public void start() {
        lastLoopNs = System.nanoTime();
        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
    }

    @Override
    public void loop() {
        long loopStartNs = System.nanoTime();
        for (LynxModule hub : hubs) hub.clearBulkCache();

        if (gamepad1.a && !lastA) {
            boolean on = !(pollenOn || nectarOn);
            pollenOn = on;
            nectarOn = on;
        }
        if (gamepad1.x && !lastX) pollenOn = !pollenOn;
        if (gamepad1.y && !lastY) nectarOn = !nectarOn;
        if (gamepad1.b) {
            pollenOn = false;
            nectarOn = false;
        }
        boolean flywheelsOn = pollenOn || nectarOn;
        if (!flywheelsOn) flywheelsOnNs = 0;
        else if (flywheelsOnNs == 0) flywheelsOnNs = loopStartNs;
        int step = (gamepad1.dpad_up && !lastUp ? 1 : 0) - (gamepad1.dpad_down && !lastDown ? 1 : 0);
        if (step != 0) {
            adjust(pollen, step);
            adjust(nectar, step);
        }
        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        double limit = Range.clip(gamepad1.left_bumper ? drive.slowPower : drive.maxPower, 0, 1);
        double[] wheels = mecanum(
                deadband(-gamepad1.left_stick_y, drive.stickDeadband),
                deadband(gamepad1.left_stick_x, drive.stickDeadband),
                deadband(gamepad1.right_stick_x, drive.stickDeadband),
                limit);
        fl.setDirection(direction(drive.reverseFl));
        bl.setDirection(direction(drive.reverseBl));
        fr.setDirection(direction(drive.reverseFr));
        br.setDirection(direction(drive.reverseBr));
        fl.setPower(wheels[0]);
        bl.setPower(wheels[1]);
        fr.setPower(wheels[2]);
        br.setPower(wheels[3]);

        double pollenRpm = rpm(pollenHistory, pollenFlywheel, pollen.reverseEncoder);
        double nectarRpm = rpm(nectarHistory, nectarFlywheel, nectar.reverseEncoder);
        double pollenOut = pollenOn ? flywheelPower(pollen, pollenRpm) : 0;
        double nectarOut = nectarOn ? flywheelPower(nectar, nectarRpm) : 0;
        pollenFlywheel.setDirection(direction(pollen.reverseFlywheel));
        nectarFlywheel.setDirection(direction(nectar.reverseFlywheel));
        pollenFlywheel.setPower(pollenOut);
        nectarFlywheel.setPower(nectarOut);
        pollenHood.setPosition(Range.clip(pollen.hoodPosition, 0, 1));
        nectarHood.setPosition(Range.clip(nectar.hoodPosition, 0, 1));

        boolean blockerOpen = flywheelsOn;
        boolean feeding = gamepad1.right_bumper && flywheelsOn
                && (loopStartNs - flywheelsOnNs) / 1e6 >= transfer.blockerOpenMs;
        double intakeOutput = intakePower(gamepad1.right_trigger, gamepad1.left_trigger);
        double transferOutput = feeding ? transfer.feedPower : Math.signum(intakeOutput) * transfer.intakePower;
        intakeMotor.setDirection(direction(intake.reverse));
        transferMotor.setDirection(direction(transfer.reverse));
        intakeMotor.setPower(intakeOutput);
        transferMotor.setPower(Range.clip(transferOutput, -1, 1));
        transferServo.setDirection(direction(transfer.reverseServo));
        transferServo.setPower(feeding ? Range.clip(transfer.servoFeedPower, -1, 1) : 0);
        blockerServo.setPosition(Range.clip(blockerOpen ? blocker.openPosition : blocker.closedPosition, 0, 1));

        addGamepadMap();
        telemetry.addData("Pollen flywheel", "%s, %.0f RPM, power %.2f, %s", pollenOn ? "ON" : "off", pollenRpm, pollenOut, setpoint(pollen));
        telemetry.addData("Nectar flywheel", "%s, %.0f RPM, power %.2f, %s", nectarOn ? "ON" : "off", nectarRpm, nectarOut, setpoint(nectar));
        telemetry.addData("Feed", feeding ? "FEEDING" : gamepad1.right_bumper ? "RB ignored: turn a flywheel on first (A)" : "off");
        telemetry.addData("Blocker", blockerOpen ? "open (a flywheel is on)" : "closed");
        telemetry.addData("Intake", "%s (%.2f), %.1f A", intakeOutput > 0 ? "intake" : intakeOutput < 0 ? "outtake" : "off",
                intakeOutput, intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Drive limit", "%.2f", limit);
        telemetry.addData("Battery (V)", "%.2f", battery.getVoltage());
        telemetry.addData("Loop (ms)", "%.1f", (loopStartNs - lastLoopNs) / 1e6);
        lastLoopNs = loopStartNs;
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : new DcMotorEx[] {fl, bl, fr, br, pollenFlywheel, nectarFlywheel, intakeMotor, transferMotor}) {
            motor.setPower(0);
        }
        transferServo.setPower(0);
    }

    /** Wheel powers in the order front-left, back-left, front-right, back-right, the largest held to {@code limit}. */
    static double[] mecanum(double forward, double strafe, double turn, double limit) {
        double scale = limit / Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        return new double[] {
                (forward + strafe + turn) * scale,
                (forward - strafe + turn) * scale,
                (forward - strafe - turn) * scale,
                (forward + strafe - turn) * scale,
        };
    }

    static double deadband(double value, double band) {
        return Math.abs(value) < band ? 0 : value;
    }

    static double intakePower(double inTrigger, double outTrigger) {
        double threshold = Math.max(intake.triggerThreshold, 0.05);
        if (inTrigger > threshold) return Range.clip(intake.power, 0, 1);
        if (outTrigger > threshold) return -Range.clip(intake.outtakePower, 0, 1);
        return 0;
    }

    static double flywheelPower(Flywheel flywheel, double rpm) {
        if (flywheel.targetRpm <= 0) return Range.clip(flywheel.power, 0, 1);
        return Range.clip(flywheel.kV * flywheel.targetRpm + flywheel.kP * (flywheel.targetRpm - rpm), 0, 1);
    }

    private static void adjust(Flywheel flywheel, int step) {
        double rpm = Math.abs(rpmStep);
        if (flywheel.targetRpm > 0) flywheel.targetRpm = Math.max(rpm, flywheel.targetRpm + step * rpm);
        else flywheel.power = Range.clip(flywheel.power + step * powerStep, 0, 1);
    }

    private static String setpoint(Flywheel flywheel) {
        return flywheel.targetRpm > 0
                ? String.format("target %.0f RPM", flywheel.targetRpm)
                : String.format("open loop at %.2f", flywheel.power);
    }

    private void addGamepadMap() {
        telemetry.addLine("Gamepad 1: left stick drive, right stick turn, hold LB slow");
        telemetry.addLine("RT intake in, LT intake out, hold RB feed (only with a flywheel on)");
        telemetry.addLine("A both flywheels (blocker opens), X pollen, Y nectar, B flywheels off");
        telemetry.addLine("D-pad up/down: flywheel power or target RPM");
    }

    private static DcMotorSimple.Direction direction(boolean reverse) {
        return reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static double rpm(ArrayDeque<double[]> history, DcMotorEx motor, boolean reverse) {
        long before = System.nanoTime();
        int ticks = motor.getCurrentPosition();
        double seconds = (before + (System.nanoTime() - before) / 2) / 1e9;
        if (reverse) ticks = -ticks;

        history.addLast(new double[] {seconds, ticks});
        while (history.size() > 2 && seconds - history.peekFirst()[0] > rpmWindowMs / 1000) history.removeFirst();
        if (history.size() < 2) return 0;
        double[] oldest = history.peekFirst();
        return (ticks - oldest[1]) / (seconds - oldest[0]) / ticksPerRev * 60;
    }
}
