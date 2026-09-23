package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.decode.DecodeOpMode;
import org.firstinspires.ftc.teamcode.decode.modules.Endgame;
import org.firstinspires.ftc.teamcode.decode.modules.Magazine;
import org.firstinspires.ftc.teamcode.decode.modules.Shooter;
import org.firstinspires.ftc.teamcode.decode.modules.Turret;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.*;

/** Sequential bring-up check for the DECODE robot; gamepad1 A advances to the next stage. */
@TeleOp(name = "Decode System Check", group = "Test")
public class DecodeSystemCheck extends DecodeOpMode {

    private static final double CURRENT_OK_AMPS = 0.05;
    private static final int ENCODER_UPDATE_TICKS = 10;
    private static final double DRIVETRAIN_TEST_POWER = 0.3;
    private static final double ODO_HEADING_DELTA_RADIANS = Math.toRadians(2.0);
    private static final double INITIAL_ENCODER_VOLTAGE_DELTA = 0.05;

    private enum Stage {
        SENSORS,
        HEADLIGHTS,
        DRIVETRAIN_FORWARD,
        TURRET_SWEEP,
        FLYWHEEL_HOOD,
        INTAKE,
        VERTICAL,
        HORIZONTAL_CLOSED,
        HORIZONTAL_OPEN,
        DONE
    }

    private Stage stage = Stage.SENSORS;
    private final ElapsedTime stageTimer = new ElapsedTime();
    private boolean stageEntered = false;

    private NormalizedColorSensor[] colorSensors;
    private String[] colorNames;

    private Limelight3A limelight;
    // Read raw: the follower's pose keeps integrating from a cached source when the Pinpoint is unplugged.
    private GoBildaPinpointDriver pinpoint;
    private int odoXBaselineTicks, odoYBaselineTicks;
    private double odoHeadingBaseline;

    private double leftInitialVoltageBaseline, rightInitialVoltageBaseline;

    private DcMotorEx leftFlywheel;
    private int flBaselineTicks, frBaselineTicks, blBaselineTicks, brBaselineTicks;
    private int leftShooterBaselineTicks;
    private boolean turretSweepReversed = false;
    private boolean hoodReset = false;

    private final Map<String, Boolean> checks = new LinkedHashMap<>();
    private final Set<String> visualChecks = new HashSet<>();

    @Override
    protected boolean shouldReadDuringInit() {
        return true;
    }

    @Override
    protected void initialize() {
        colorNames = new String[] {
                "frontLeftColor", "frontRightColor", "middleFrontColor",
                "middleBackColor", "backRightColor", "backLeftColor"
        };
        colorSensors = new NormalizedColorSensor[colorNames.length];
        for (int i = 0; i < colorNames.length; i++) {
            colorSensors[i] = hardwareMap.get(NormalizedColorSensor.class, colorNames[i]);
        }
        updateColorChecks();

        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");

        resetEncoder(robot.drivetrain.getFl());
        resetEncoder(robot.drivetrain.getFr());
        resetEncoder(robot.drivetrain.getBl());
        resetEncoder(robot.drivetrain.getBr());

        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");
        if (pinpoint != null) {
            pinpoint.update();
            odoXBaselineTicks = pinpoint.getEncoderX();
            odoYBaselineTicks = pinpoint.getEncoderY();
            odoHeadingBaseline = pinpoint.getHeading(AngleUnit.RADIANS);
        }

        // Baseline before start() moves the initial servos, so any later drift proves the encoder reads them.
        leftInitialVoltageBaseline = robot.endgame.leftInitialEncoder.getVoltage();
        rightInitialVoltageBaseline = robot.endgame.rightInitialEncoder.getVoltage();

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        checks.put("Odometry X updates",       false);
        checks.put("Odometry Y updates",       false);
        checks.put("Odometry heading updates", false);
        checks.put("Left initial encoder",     false);
        checks.put("Right initial encoder",    false);
        checks.put("Headlight strobe",         false);
        checks.put("FL encoder updates",       false);
        checks.put("FR encoder updates",       false);
        checks.put("BL encoder updates",       false);
        checks.put("BR encoder updates",       false);
        checks.put("PTO disengaged",           false);
        checks.put("Drivetrain FL current",    false);
        checks.put("Drivetrain FR current",    false);
        checks.put("Drivetrain BL current",    false);
        checks.put("Drivetrain BR current",    false);
        checks.put("Turret servo moved",       false);
        checks.put("Flywheel position updates",false);
        checks.put("Left flywheel current",    false);
        checks.put("Right flywheel current",   false);
        checks.put("Hood TOP applied",         false);
        checks.put("Hood RESET applied",       false);
        checks.put("Intake current",           false);
        checks.put("Vertical current",         false);
        checks.put("Horizontal Front OPEN",    false);
        checks.put("Horizontal Back OPEN",     false);
        checks.put("Horizontal Front CLOSED",  false);
        checks.put("Horizontal Back CLOSED",   false);
        checks.put("Limelight connected",      false);
        checks.put("Limelight capturing",      false);

        visualChecks.add("Headlight strobe");
        visualChecks.add("PTO disengaged");
        visualChecks.add("Turret servo moved");
        visualChecks.add("Hood TOP applied");
        visualChecks.add("Hood RESET applied");
        visualChecks.add("Horizontal Front OPEN");
        visualChecks.add("Horizontal Back OPEN");
        visualChecks.add("Horizontal Front CLOSED");
        visualChecks.add("Horizontal Back CLOSED");
    }

    @Override
    protected void initializeLoop() {
        updateColorChecks();
        updateVisionChecks();
        updateInitialEncoderChecks();
        updateOdometryChecks();
        robot.magazine.updateMagazineColorState();
    }

    @Override
    protected void onStart() {
        stage = Stage.SENSORS;
        stageEntered = false;
        stageTimer.reset();
    }

    @Override
    protected void gameLoop() {
        if (gamepad1.aWasPressed() && stage != Stage.DONE) {
            advanceStage();
        }

        if (!stageEntered) {
            stopAll();
            onStageEnter();
            stageEntered = true;
            stageTimer.reset();
        }

        updateColorChecks();
        updateVisionChecks();
        updateInitialEncoderChecks();
        updateOdometryChecks();

        switch (stage) {
            case SENSORS:           runSensorsStage();          break;
            case HEADLIGHTS:        runHeadlightsStage();       break;
            case DRIVETRAIN_FORWARD:runDrivetrainForwardStage();break;
            case TURRET_SWEEP:      runTurretSweepStage();      break;
            case FLYWHEEL_HOOD:     runFlywheelHoodStage();     break;
            case INTAKE:            runIntakeStage();           break;
            case VERTICAL:          runVerticalStage();         break;
            case HORIZONTAL_OPEN:   runHorizontalOpenStage();   break;
            case HORIZONTAL_CLOSED: runHorizontalClosedStage(); break;
            case DONE:              stopAll();                  break;
        }

        robot.magazine.updateMagazineColorState();
    }

    @Override
    protected void onEnd() {
        stopAll();
    }

    private void advanceStage() {
        Stage[] all = Stage.values();
        int next = Math.min(stage.ordinal() + 1, all.length - 1);
        stage = all[next];
        stageEntered = false;
    }

    private Stage nextStage() {
        Stage[] all = Stage.values();
        return all[Math.min(stage.ordinal() + 1, all.length - 1)];
    }

    private void onStageEnter() {
        switch (stage) {
            case HEADLIGHTS:
                Magazine.HeadlightFrontState.STROBE.activate();
                break;
            case DRIVETRAIN_FORWARD:
                Endgame.LeftPtoState.UP.activate();
                Endgame.RightPtoState.UP.activate();
                // BRAKE so the wheels stop with the timer instead of coasting.
                setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                flBaselineTicks = robot.drivetrain.getFl().getCurrentPosition();
                frBaselineTicks = robot.drivetrain.getFr().getCurrentPosition();
                blBaselineTicks = robot.drivetrain.getBl().getCurrentPosition();
                brBaselineTicks = robot.drivetrain.getBr().getCurrentPosition();
                break;
            case TURRET_SWEEP:
                turretSweepReversed = false;
                Turret.TurretState.LEFT.activate();
                break;
            case FLYWHEEL_HOOD:
                leftShooterBaselineTicks = leftFlywheel.getCurrentPosition();
                hoodReset = false;
                Shooter.FlywheelState.IDLE.activate();
                Shooter.HoodState.TOP.activate();
                break;
            case INTAKE:
                Magazine.IntakeState.FORWARD.activate();
                break;
            case VERTICAL:
                Magazine.VerticalState.ON.activate();
                break;
            case HORIZONTAL_OPEN:
                Magazine.HorizontalFrontState.OPEN.activate();
                Magazine.HorizontalBackState.OPEN.activate();
                break;
            case HORIZONTAL_CLOSED:
                Magazine.HorizontalFrontState.STORED.activate();
                Magazine.HorizontalBackState.STORED.activate();
                break;
            default: break;
        }
    }

    private void stopAll() {
        robot.drivetrain.setRawTargets(0, 0, 0, 0);
        // Restore the SDK default so OpModes that expect coast aren't surprised after a check run.
        setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Magazine.IntakeState.OFF.activate();
        Magazine.VerticalState.OFF.activate();
        Magazine.HeadlightFrontState.OFF.activate();
        Shooter.FlywheelState.OFF.activate();
        Shooter.HoodState.BOTTOM.activate();
        Turret.TurretState.CENTER.activate();
    }

    private void setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        robot.drivetrain.getFl().setZeroPowerBehavior(behavior);
        robot.drivetrain.getFr().setZeroPowerBehavior(behavior);
        robot.drivetrain.getBl().setZeroPowerBehavior(behavior);
        robot.drivetrain.getBr().setZeroPowerBehavior(behavior);
    }

    private void runSensorsStage() {
    }

    private void runHeadlightsStage() {
        Magazine.HeadlightFrontState.STROBE.activate();
    }

    private void runDrivetrainForwardStage() {
        if (stageTimer.seconds() < 1.0) {
            robot.drivetrain.setRawTargets(
                    DRIVETRAIN_TEST_POWER, DRIVETRAIN_TEST_POWER,
                    DRIVETRAIN_TEST_POWER, DRIVETRAIN_TEST_POWER);
            updateIfTrue("Drivetrain FL current", robot.drivetrain.getFl().getCurrent(CurrentUnit.AMPS) > CURRENT_OK_AMPS);
            updateIfTrue("Drivetrain FR current", robot.drivetrain.getFr().getCurrent(CurrentUnit.AMPS) > CURRENT_OK_AMPS);
            updateIfTrue("Drivetrain BL current", robot.drivetrain.getBl().getCurrent(CurrentUnit.AMPS) > CURRENT_OK_AMPS);
            updateIfTrue("Drivetrain BR current", robot.drivetrain.getBr().getCurrent(CurrentUnit.AMPS) > CURRENT_OK_AMPS);
            updateIfTrue("FL encoder updates",
                    Math.abs(robot.drivetrain.getFl().getCurrentPosition() - flBaselineTicks) > ENCODER_UPDATE_TICKS);
            updateIfTrue("FR encoder updates",
                    Math.abs(robot.drivetrain.getFr().getCurrentPosition() - frBaselineTicks) > ENCODER_UPDATE_TICKS);
            updateIfTrue("BL encoder updates",
                    Math.abs(robot.drivetrain.getBl().getCurrentPosition() - blBaselineTicks) > ENCODER_UPDATE_TICKS);
            updateIfTrue("BR encoder updates",
                    Math.abs(robot.drivetrain.getBr().getCurrentPosition() - brBaselineTicks) > ENCODER_UPDATE_TICKS);
        } else {
            robot.drivetrain.setRawTargets(0, 0, 0, 0);
        }
    }

    private void runTurretSweepStage() {
        if (stageTimer.seconds() < 2.0) {
            Turret.TurretState.LEFT.activate();
        } else if (!turretSweepReversed) {
            Turret.TurretState.RIGHT.activate();
            turretSweepReversed = true;
        }
    }

    private void runFlywheelHoodStage() {
        Shooter.FlywheelState.IDLE.activate();
        if (!hoodReset && stageTimer.seconds() > 1.0) {
            Shooter.HoodState.RESET.activate();
            hoodReset = true;
        }
        updateIfTrue("Flywheel position updates",
                Math.abs(leftFlywheel.getCurrentPosition() - leftShooterBaselineTicks) > ENCODER_UPDATE_TICKS);
        updateIfTrue("Left flywheel current", robot.shooter.getLeftShooterCurrent() > CURRENT_OK_AMPS);
        updateIfTrue("Right flywheel current", robot.shooter.getRightShooterCurrent() > CURRENT_OK_AMPS);
    }

    private void runIntakeStage() {
        if (stageTimer.seconds() < 1.0) {
            Magazine.IntakeState.FORWARD.activate();
            updateIfTrue("Intake current", robot.magazine.getIntakeCurrent() > CURRENT_OK_AMPS);
        } else {
            Magazine.IntakeState.OFF.activate();
        }
    }

    private void runVerticalStage() {
        if (stageTimer.seconds() < 1.0) {
            Magazine.VerticalState.ON.activate();
            updateIfTrue("Vertical current", robot.magazine.getVerticalCurrent() > CURRENT_OK_AMPS);
        } else {
            Magazine.VerticalState.OFF.activate();
        }
    }

    private void runHorizontalOpenStage() {
        Magazine.HorizontalFrontState.OPEN.activate();
        Magazine.HorizontalBackState.OPEN.activate();
    }

    private void runHorizontalClosedStage() {
        Magazine.HorizontalFrontState.STORED.activate();
        Magazine.HorizontalBackState.STORED.activate();
    }

    private void resetEncoder(EnhancedMotor m) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Latches: once a check passes it stays passed, so a transient blip is enough.
    private void updateIfTrue(String key, boolean result) {
        Boolean prev = checks.get(key);
        if (prev == null || !prev) {
            checks.put(key, result);
        }
    }

    private void updateColorChecks() {
        for (int i = 0; i < colorSensors.length; i++) {
            checks.put("Color: " + colorNames[i], isColorSensorAlive(colorSensors[i]));
        }
    }

    private void updateVisionChecks() {
        boolean llConnected = false;
        boolean llCapturing = false;
        if (limelight != null) {
            LLStatus status = limelight.getStatus();
            llConnected = status != null;
            if (llConnected) {
                LLResult result = limelight.getLatestResult();
                llCapturing = status.getFps() > 0 && result != null;
            }
        }
        updateIfTrue("Limelight connected", llConnected);
        updateIfTrue("Limelight capturing", llCapturing);
    }

    // Counts only while READY: an unplugged Pinpoint holds frozen ticks and a non-READY status.
    private void updateOdometryChecks() {
        if (pinpoint == null) return;
        pinpoint.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) return;
        updateIfTrue("Odometry X updates",
                Math.abs(pinpoint.getEncoderX() - odoXBaselineTicks) > ENCODER_UPDATE_TICKS);
        updateIfTrue("Odometry Y updates",
                Math.abs(pinpoint.getEncoderY() - odoYBaselineTicks) > ENCODER_UPDATE_TICKS);
        double dh = pinpoint.getHeading(AngleUnit.RADIANS) - odoHeadingBaseline;
        while (dh > Math.PI) dh -= 2 * Math.PI;
        while (dh < -Math.PI) dh += 2 * Math.PI;
        updateIfTrue("Odometry heading updates", Math.abs(dh) > ODO_HEADING_DELTA_RADIANS);
    }

    private void updateInitialEncoderChecks() {
        updateIfTrue("Left initial encoder",
                Math.abs(robot.endgame.leftInitialEncoder.getVoltage() - leftInitialVoltageBaseline)
                        > INITIAL_ENCODER_VOLTAGE_DELTA);
        updateIfTrue("Right initial encoder",
                Math.abs(robot.endgame.rightInitialEncoder.getVoltage() - rightInitialVoltageBaseline)
                        > INITIAL_ENCODER_VOLTAGE_DELTA);
    }

    private boolean isColorSensorAlive(NormalizedColorSensor s) {
        NormalizedRGBA rgba = s.getNormalizedColors();
        return !Float.isNaN(rgba.red) && !Float.isNaN(rgba.green)
                && !Float.isNaN(rgba.blue) && !Float.isNaN(rgba.alpha);
    }

    @Override
    protected void telemetry() {
        Pose pose = robot.follower.pose();
        robot.telemetry.addSeparator();
        robot.telemetry.addGroupHeader("SYSTEM CHECK", COLOR_YELLOW);
        addDSLarge("Stage", stage);
        Stage next = nextStage();
        if (next == Stage.DRIVETRAIN_FORWARD) {
            addDSHtml("WARNING", htmlBold(htmlColorSize(COLOR_RED, FONT_XXLARGE,
                    "⚠ ROBOT WILL DRIVE FORWARD ⚠")));
        }
        addDSLarge("Next", next == stage ? "—" : next);
        robot.telemetry.addData("Advance", "Press gamepad1.A");
        robot.telemetry.addData("Stage Time", "%.2fs", stageTimer.seconds());
        robot.telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        robot.telemetry.addData("Magazine Pattern", robot.magazine.getColorPattern());

        robot.telemetry.addSeparator();
        robot.telemetry.addGroupHeader("CHECKS", COLOR_MODULE);
        for (Map.Entry<String, Boolean> e : checks.entrySet()) {
            String mark;
            String dashStatus;
            if (visualChecks.contains(e.getKey())) {
                mark = htmlColor(COLOR_YELLOW, "VISUAL CHECK");
                dashStatus = "VISUAL";
            } else if (e.getValue()) {
                mark = htmlColor(COLOR_GREEN, "✓");
                dashStatus = "OK";
            } else {
                mark = htmlColor(COLOR_RED, "✗");
                dashStatus = "FAIL";
            }
            addDSHtml(e.getKey(), mark);
            robot.telemetry.addDashboardData(e.getKey(), dashStatus);
        }
    }

    private void addDSLarge(String caption, Object value) {
        robot.telemetry.addDSLine(htmlSize(FONT_SMALL, htmlBold(htmlEscape(caption))) + ": "
                + htmlColorSize(COLOR_VALUE, FONT_XLARGE, htmlEscape(String.valueOf(value))));
    }

    private void addDSHtml(String caption, String html) {
        robot.telemetry.addDSLine(htmlSize(FONT_NORMAL, htmlBold(htmlEscape(caption))) + ": " + html);
    }
}
