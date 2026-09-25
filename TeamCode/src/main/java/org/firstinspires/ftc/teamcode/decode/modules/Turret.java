package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization.toField;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetY;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldY;
import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.turretTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

@Config("Decode Turret")
public class Turret extends Module {

    public double turretManualOffset = 0;
    public static double autoTurretOffset = 0;

    public static boolean isCloseTele = true;
    public static double teleTurretOffsetClose = 2;
    public static double teleTurretOffsetFar = -4;

    public static boolean isAuto = false;

    public static double turretX = -3.875;
    public static double turretY = -1.6;

    private static final int DECODE_PIPELINE = 0;

    public static double TENSION_OFFSET = 0.0;
    public static int RED_TAG_ID = 24;
    public static int BLUE_TAG_ID = 20;

    public static double LIMELIGHT_X = 3.25;
    public static double LIMELIGHT_Y = 2.688;

    public static long LIMELIGHT_MAX_STALENESS_MS = 200;

    public static double RELOCALIZATION_FIELD_CENTER_OFFSET_IN = 72;

    public static boolean drawMT1 = false;
    public static boolean drawMT2 = true;
    public static boolean logLimelightPoses = false;

    public static int limelightHeadingEveryNLoops = 3;

    public static double AHEAD_GAIN = 0;
    public static double MIN_DELTA_FOR_AHEAD = 0;

    public static double frontServoOffset = 0.008;
    public static double backServoOffset = 0.00;

    private String relocalizationStatus = "NO_ATTEMPT";

    private int relocalizationDesiredTagId = -1;
    private int relocalizationSeenTagCount = 0;
    private int relocalizationMatchedTagId = -1;
    private double relocalizationTxDeg = Double.NaN;
    private double relocalizationTyDeg = Double.NaN;
    private double relocalizationOdometryBeforeX = Double.NaN;
    private double relocalizationOdometryBeforeY = Double.NaN;
    private double relocalizationOdometryBeforeHeadingDeg = Double.NaN;

    private final Limelight3A limelight;

    private final EnhancedServo turretServoFront;
    private final EnhancedServo turretServoBack;

    private Follower follower;
    private OrientationSender orientationSender;
    private final boolean pipelineSwitchAccepted;
    private boolean wrongPipeline = false;
    private int lastPipelineIndex = DECODE_PIPELINE;

    private double targetAngle = 0;
    private double rawTargetAngle = 0;
    private double previousTargetAngle = 0.0;

    private double aheadTargetAngle = 0.0;

    private double targetServoPosition = 0.5;
    private double lastTargetServoPosition = 0.5;

    private double deltaRawTarget = 0;
    private double previousRawTargetAngle = 0.0;


    public double flightTime = 1;

    private int limelightHeadingLoopCounter = 0;

    private Pose limelightMT1Pose;
    private Pose limelightMT2Pose;
    private double mt2RawX = Double.NaN;
    private double mt2RawY = Double.NaN;

    public enum TurretState implements State {
        CENTER(.5),
        RIGHT(0.995),
        LEFT(0.005),
        AUTOAIM(-1),
        HOLD(-1);

        TurretState(double value) {
            setValue(value);
        }
    }

    public Turret(HardwareMap hardwareMap) {
        turretServoFront = new EnhancedServo(hardwareMap, "turretFront").withCachingTolerance(0.001);
        turretServoBack = new EnhancedServo(hardwareMap, "turretBack").withCachingTolerance(0.001);

        turretServoFront.setPwmRange(new PwmControl.PwmRange(525, 2475));
        turretServoBack.setPwmRange(new PwmControl.PwmRange(525, 2475));

        turretServoFront.setDirection(Servo.Direction.FORWARD);
        turretServoBack.setDirection(Servo.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pipelineSwitchAccepted = limelight.pipelineSwitch(DECODE_PIPELINE);
        limelight.start();
        orientationSender = new OrientationSender(limelight);
    }

    public Turret withFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    private Follower requireFollower() {
        if (follower == null) {
            throw new IllegalStateException("Turret needs a Follower; pass one with withFollower()");
        }
        return follower;
    }

    @Override
    protected void initStates() {
        setStates(TurretState.AUTOAIM);
    }

    @Override
    protected void read() {
        turretManualOffset = isAuto ? autoTurretOffset : (isCloseTele ? teleTurretOffsetClose : teleTurretOffsetFar);

        updateTargetPosition();

        // A write toggle calls stop() mid-OpMode, which shuts the Limelight down; bring it back.
        if (orientationSender == null) {
            limelight.start();
            orientationSender = new OrientationSender(limelight);
        }

        if (limelightHeadingLoopCounter++ % Math.max(1, limelightHeadingEveryNLoops) == 0) {
            // Pedro 0 deg points along +x (audience's right); the Limelight's 0 deg points away from the audience.
            double robotHeadingDeg = Math.toDegrees(requireFollower().pose().heading());
            orientationSender.send(robotHeadingDeg + 90.0);
        }

        updateLimelightPoses();
    }

    @Override
    protected void write() {
        double frontPos = Math.max(0.0, Math.min(1.0, targetServoPosition - TENSION_OFFSET + frontServoOffset));
        double backPos = Math.max(0.0, Math.min(1.0, targetServoPosition + TENSION_OFFSET + backServoOffset));

        turretServoFront.setPosition(frontPos);
        turretServoBack.setPosition(backPos);
    }

    @Override
    public void stop() {
        turretServoFront.setPwmDisable();
        turretServoBack.setPwmDisable();
        if (orientationSender != null) {
            orientationSender.shutdown();
            orientationSender = null;
        }
        limelight.stop();
    }

    @Override
    protected void onTelemetry() {
        if (turretTelemetry.position) {
            logDashboard("Turret State", getState(TurretState.class));
            logDashboard("Raw Target Angle (deg)", "%.1f", rawTargetAngle);
            logDashboard("Delta Raw Target (deg)", "%.1f", deltaRawTarget);
            log("Target Angle (deg)", "%.1f", targetAngle);
            logDashboard("Ahead Target Angle (deg)", "%.1f", aheadTargetAngle);
            log("Turret Offset (deg)", "%.1f", turretManualOffset);
            log("Relocalization status", "%s", relocalizationStatus);
            logDashboard("Reloc desiredTagId", "%d", relocalizationDesiredTagId);
            logDashboard("Reloc seenTagCount", "%d", relocalizationSeenTagCount);
            logDashboard("Reloc matchedTagId", "%d", relocalizationMatchedTagId);
            logDashboard("Reloc tx/ty (deg)", "%.2f / %.2f", relocalizationTxDeg, relocalizationTyDeg);
            logDashboard("Reloc odo before x/y/h", "%.2f / %.2f / %.2f",
                    relocalizationOdometryBeforeX,
                    relocalizationOdometryBeforeY,
                    relocalizationOdometryBeforeHeadingDeg);
        }
        logDashboard("Target Servo Position", "%.3f", targetServoPosition);
        if (turretTelemetry.servos) {
            logDashboard("Front Servo Position", "%.3f", turretServoFront.getPosition());
            logDashboard("Back Servo Position", "%.3f", turretServoBack.getPosition());
        }
        if (wrongPipeline) {
            log("Limelight", "WRONG PIPELINE %d, needs %d; re-INIT", lastPipelineIndex, DECODE_PIPELINE);
        }
        if (!pipelineSwitchAccepted) logDashboard("Limelight pipeline switch", "FAILED");
        if (logLimelightPoses && limelightMT2Pose != null) {
            Pose dash = toField(limelightMT2Pose);
            logDashboard("RAW robot pose LL", "x %.2f y %.2f", mt2RawX, mt2RawY);
            logDashboard("Robot Pose LL", "x %.2f y %.2f h %.3f",
                    limelightMT2Pose.x(), limelightMT2Pose.y(), limelightMT2Pose.heading());
            logDashboard("toField robot pose LL", "x %.2f y %.2f h %.1f",
                    dash.x(), dash.y(), Math.toDegrees(dash.heading()));
        }
    }

    private void updateTargetPosition() {
        if (getState(TurretState.class).equals(TurretState.AUTOAIM)) {
            double robotHeading = Math.toDegrees(requireFollower().pose().heading());

            double absoluteAngle =
                    Math.toDegrees(Math.atan2(targetY - turretFieldY, targetX - turretFieldX));

            rawTargetAngle = normalizeAngle(absoluteAngle - robotHeading);

            deltaRawTarget = calculateShortestError(rawTargetAngle, previousRawTargetAngle);
            previousRawTargetAngle = rawTargetAngle;

            targetAngle = rawTargetAngle + turretManualOffset;

            double angleDelta = calculateShortestError(targetAngle, previousTargetAngle);
            aheadTargetAngle = (Math.abs(angleDelta) > MIN_DELTA_FOR_AHEAD)
                    ? targetAngle + Math.signum(angleDelta) * AHEAD_GAIN
                    : targetAngle;

            previousTargetAngle = targetAngle;
            targetServoPosition = angleToServoPosition(aheadTargetAngle);

        } else if (getState(TurretState.class).equals(TurretState.HOLD)) {
            targetServoPosition = lastTargetServoPosition;
        } else {
            targetServoPosition = getState(TurretState.class).getValue();
        }

        double minServo = Math.min(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        double maxServo = Math.max(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        if (targetServoPosition > maxServo) {
            targetServoPosition = maxServo;
        } else if (targetServoPosition < minServo) {
            targetServoPosition = minServo;
        }

        lastTargetServoPosition = targetServoPosition;
    }

    /** Robot-center MegaTag2 pose, or null when rejected; the reason is in {@link #getRelocalizationStatus()}. */
    public Pose getRelocalizedRobotPoseFromLimelight() {
        clearRelocalizationDebugData();

        Pose odometryPoseBefore = requireFollower().pose();
        relocalizationOdometryBeforeX = odometryPoseBefore.x();
        relocalizationOdometryBeforeY = odometryPoseBefore.y();
        relocalizationOdometryBeforeHeadingDeg = Math.toDegrees(odometryPoseBefore.heading());

        LLResult result = limelight.getLatestResult();
        String rejectReason = rejectReason(result);
        if (rejectReason != null) {
            relocalizationStatus = rejectReason;
            return null;
        }

        int desiredTagId = Context.allianceColor == AllianceColor.RED ? RED_TAG_ID : BLUE_TAG_ID;
        relocalizationDesiredTagId = desiredTagId;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        relocalizationSeenTagCount = fiducials.size();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == desiredTagId) {
                relocalizationMatchedTagId = fr.getFiducialId();
                relocalizationTxDeg = fr.getTargetXDegrees();
                relocalizationTyDeg = fr.getTargetYDegrees();
                break;
            }
        }

        if (relocalizationMatchedTagId != desiredTagId) {
            relocalizationStatus = "MT2_ALLIANCE_TAG_NOT_VISIBLE";
            return null;
        }

        Pose3D cameraPoseMT2 = result.getBotpose_MT2();
        if (cameraPoseMT2 == null) {
            relocalizationStatus = "MT2_NO_POSE";
            return null;
        }

        double rawX = cameraPoseMT2.getPosition().toUnit(DistanceUnit.INCH).x;
        double rawY = cameraPoseMT2.getPosition().toUnit(DistanceUnit.INCH).y;
        if (!Double.isFinite(rawX) || !Double.isFinite(rawY)) {
            relocalizationStatus = "MT2_NONFINITE";
            return null;
        }

        return mt2ToPedro(cameraPoseMT2, rawX, rawY);
    }

    public String getRelocalizationStatus() {
        return relocalizationStatus;
    }

    /** Latest fresh MegaTag1 robot pose in Pedro coordinates, or null. */
    public Pose getMT1Pose() {
        return limelightMT1Pose;
    }

    /** Latest fresh MegaTag2 robot pose in Pedro coordinates, or null. */
    public Pose getMT2Pose() {
        return limelightMT2Pose;
    }

    private void updateLimelightPoses() {
        limelightMT1Pose = null;
        limelightMT2Pose = null;
        mt2RawX = Double.NaN;
        mt2RawY = Double.NaN;

        LLResult result = limelight.getLatestResult();
        String rejectReason = rejectReason(result);
        wrongPipeline = "WRONG_PIPELINE".equals(rejectReason);
        if (rejectReason != null) return;

        double robotHeadingRad = requireFollower().pose().heading();
        double camOffsetFieldX = LIMELIGHT_X * Math.cos(robotHeadingRad) - LIMELIGHT_Y * Math.sin(robotHeadingRad);
        double camOffsetFieldY = LIMELIGHT_X * Math.sin(robotHeadingRad) + LIMELIGHT_Y * Math.cos(robotHeadingRad);

        Pose3D mt1Pose = result.getBotpose();
        if (mt1Pose != null) {
            double mt1RawX = mt1Pose.getPosition().toUnit(DistanceUnit.INCH).x;
            double mt1RawY = mt1Pose.getPosition().toUnit(DistanceUnit.INCH).y;
            if (Double.isFinite(mt1RawX) && Double.isFinite(mt1RawY)) {
                limelightMT1Pose = new Pose(
                        mt1RawY + RELOCALIZATION_FIELD_CENTER_OFFSET_IN - camOffsetFieldX,
                        -mt1RawX + RELOCALIZATION_FIELD_CENTER_OFFSET_IN - camOffsetFieldY,
                        Math.toRadians(mt1Pose.getOrientation().getYaw(AngleUnit.DEGREES)) - Math.PI / 2);
            }
        }

        // Decode subtracts the camera offset from MT1 but not MT2; kept as-is until checked on the robot.
        Pose3D mt2Pose = result.getBotpose_MT2();
        if (mt2Pose != null) {
            double rawX = mt2Pose.getPosition().toUnit(DistanceUnit.INCH).x;
            double rawY = mt2Pose.getPosition().toUnit(DistanceUnit.INCH).y;
            mt2RawX = rawX;
            mt2RawY = rawY;
            if (Double.isFinite(rawX) && Double.isFinite(rawY)) {
                limelightMT2Pose = mt2ToPedro(mt2Pose, rawX, rawY);
            }
        }
    }

    /** Why {@code result} can't be used, as a relocalization status, or null if it is fresh, valid and from pipeline 0. */
    private String rejectReason(LLResult result) {
        if (result == null) return "NO_VALID_LL_RESULT";
        if (result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) return "STALE_RESULT";
        // The Limelight stays on whatever pipeline it was last switched to, e.g. a BioBuzz SnapScript.
        lastPipelineIndex = result.getPipelineIndex();
        if (lastPipelineIndex != DECODE_PIPELINE) return "WRONG_PIPELINE";
        if (!result.isValid()) return "NO_VALID_LL_RESULT";
        return null;
    }

    // Inverse of the +90 deg heading sent in read(): FTC center-origin inches to Pedro corner-origin.
    private static Pose mt2ToPedro(Pose3D mt2Pose, double rawX, double rawY) {
        return new Pose(
                rawY + RELOCALIZATION_FIELD_CENTER_OFFSET_IN,
                -rawX + RELOCALIZATION_FIELD_CENTER_OFFSET_IN,
                Math.toRadians(mt2Pose.getOrientation().getYaw(AngleUnit.DEGREES)) - Math.PI / 2);
    }

    private double angleToServoPosition(double angleDeg) {
        double angle = normalizeAngle(angleDeg);
        double scale = (TurretState.LEFT.getValue() - TurretState.RIGHT.getValue()) / 180.0;
        return (angle > 180)
                ? TurretState.CENTER.getValue() + scale * (angle - 360)
                : TurretState.CENTER.getValue() + scale * angle;
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double calculateShortestError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private void clearRelocalizationDebugData() {
        relocalizationDesiredTagId = -1;
        relocalizationSeenTagCount = 0;
        relocalizationMatchedTagId = -1;
        relocalizationTxDeg = Double.NaN;
        relocalizationTyDeg = Double.NaN;
        relocalizationOdometryBeforeX = Double.NaN;
        relocalizationOdometryBeforeY = Double.NaN;
        relocalizationOdometryBeforeHeadingDeg = Double.NaN;
    }

    /** updateRobotOrientation is a blocking HTTP POST with a 15 s read timeout, so it runs off the loop thread. */
    private static final class OrientationSender {
        private final Limelight3A limelight;
        private final AtomicReference<Double> pendingYawDeg = new AtomicReference<>();
        private final ExecutorService executor = Executors.newSingleThreadExecutor(runnable -> {
            Thread thread = new Thread(runnable, "Limelight orientation");
            thread.setDaemon(true);
            return thread;
        });

        OrientationSender(Limelight3A limelight) {
            this.limelight = limelight;
        }

        // Only an empty-to-full swap queues a post, so at most one waits behind the one in flight, and it sends the newest yaw.
        void send(double yawDeg) {
            if (pendingYawDeg.getAndSet(yawDeg) == null) {
                executor.execute(() -> limelight.updateRobotOrientation(pendingYawDeg.getAndSet(null)));
            }
        }

        void shutdown() {
            executor.shutdownNow();
        }
    }
}
