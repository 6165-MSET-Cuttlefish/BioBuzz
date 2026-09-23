package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization.toField;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetY;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldY;
import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.turretTelemetry;
import static org.firstinspires.ftc.teamcode.decode.modules.MagazineState.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.decode.modules.MagazineState.ArtifactColor.PURPLE;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.decode.DecodeContext;

@Config("Decode Turret")
public class Turret extends Module {

    public double turretManualOffset = 0;
    public double leftSideOffset = 0;
    public static double autoTurretOffset = 0;

    public static boolean isCloseTele = true;
    public static double teleTurretOffsetClose = 2;
    public static double teleTurretOffsetFar = -4;

    public double turretAprilTagOffset = 0;
    public static boolean isAuto = false;

    public static double turretX = -3.875;
    public static double turretY = -1.6;

    public static double TENSION_OFFSET = 0.0;
    public static boolean useLimelight = true;
    public static int RED_TAG_ID = 24;
    public static int BLUE_TAG_ID = 20;

    public static double LIMELIGHT_X = 3.25;
    public static double LIMELIGHT_Y = 2.688;

    public static long LIMELIGHT_MAX_STALENESS_MS = 200;

    public static double RELOCALIZATION_FIELD_CENTER_OFFSET_IN = 72;

    public static boolean drawMT1 = false;
    public static boolean drawMT2 = true;
    public static boolean logLimelightPoses = false;

    // updateRobotOrientation is a blocking HTTP POST to the Limelight, so don't send it every loop.
    public static int limelightHeadingEveryNLoops = 3;

    public static double maxTurretAngle = 90.0;

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
    private Endgame endgame;

    public double targetAngle = 0;
    public double rawTargetAngle = 0;
    private double previousTargetAngle = 0.0;

    public double aheadTargetAngle = 0.0;

    public double targetServoPosition = 0.5;
    public double lastTargetServoPosition = 0.5;

    public double deltaRawTarget = 0;
    private double previousRawTargetAngle = 0.0;

    private boolean withinRange = true;

    public double flightTime = 1;

    public boolean detectingObelisk = false;
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
        HOLD(-1),
        OFF(-1),
        MANUAL(-1);

        TurretState(double value) {
            setValue(value);
        }
    }

    public Turret(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(turretTelemetry.TOGGLE);

        turretServoFront = new EnhancedServo(hardwareMap, "turretFront").withCachingTolerance(0.001);
        turretServoBack = new EnhancedServo(hardwareMap, "turretBack").withCachingTolerance(0.001);

        turretServoFront.setPwmRange(new PwmControl.PwmRange(525, 2475));
        turretServoBack.setPwmRange(new PwmControl.PwmRange(525, 2475));

        turretServoFront.setDirection(Servo.Direction.FORWARD);
        turretServoBack.setDirection(Servo.Direction.FORWARD);

        // Pipeline 0 is the DECODE AprilTag/MT2 pipeline; BioBuzz's cell-tip SnapScripts are 1 and 2.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Turret withFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    public Turret withEndgame(Endgame endgame) {
        this.endgame = endgame;
        return this;
    }

    private Follower requireFollower() {
        if (follower == null) {
            throw new IllegalStateException("Turret needs a Follower; pass one with withFollower()");
        }
        return follower;
    }

    private Endgame requireEndgame() {
        if (endgame == null) {
            throw new IllegalStateException("Turret needs the Endgame module; pass it with withEndgame()");
        }
        return endgame;
    }

    @Override
    protected void initStates() {
        setStates(TurretState.AUTOAIM);
    }

    @Override
    protected void read() {
        turretManualOffset = isAuto ? autoTurretOffset : (isCloseTele ? teleTurretOffsetClose : teleTurretOffsetFar);

        updateTargetPosition();

        if (limelightHeadingLoopCounter++ % Math.max(1, limelightHeadingEveryNLoops) == 0) {
            // Pedro 0 deg points along +x (audience's right); the Limelight's 0 deg points away from the audience.
            double robotHeadingDeg = Math.toDegrees(requireFollower().pose().heading());
            limelight.updateRobotOrientation(robotHeadingDeg + 90.0);
        }

        updateLimelightPoses();
    }

    /** Sets {@link #turretAprilTagOffset} so the turret aims at {@code targetPose}, using the Limelight tx to {@code targetApriltagPose}. */
    public void snapshotAprilTagOffset(Pose targetPose, Pose targetApriltagPose) {
        if (!useLimelight) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;
        if (result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) return;

        int desiredTagId = Context.allianceColor == AllianceColor.RED ? RED_TAG_ID : BLUE_TAG_ID;
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            if (fr.getFiducialId() == desiredTagId) {
                double txRad = Math.toRadians(fr.getTargetXDegrees());

                Pose robotPose = requireFollower().pose();
                double heading = robotPose.heading();
                double camX = robotPose.x()
                        + LIMELIGHT_X * Math.cos(heading) - LIMELIGHT_Y * Math.sin(heading);
                double camY = robotPose.y()
                        + LIMELIGHT_X * Math.sin(heading) + LIMELIGHT_Y * Math.cos(heading);

                double tagAbsRad = Math.atan2(
                        targetApriltagPose.y() - camY,
                        targetApriltagPose.x() - camX);
                double tpAbsRad = Math.atan2(
                        targetPose.y() - camY,
                        targetPose.x() - camX);
                double tagToTargetRad = Math.atan2(
                        Math.sin(tpAbsRad - tagAbsRad),
                        Math.cos(tpAbsRad - tagAbsRad));

                // Limelight tx is positive to the right, i.e. clockwise.
                double targetLLRad = txRad - tagToTargetRad;
                double targetAbsDirRad = heading - targetLLRad;
                double estDist = Math.hypot(
                        targetPose.x() - camX,
                        targetPose.y() - camY);
                double estTargetX = camX + estDist * Math.cos(targetAbsDirRad);
                double estTargetY = camY + estDist * Math.sin(targetAbsDirRad);

                double turretAbsToTarget = Math.atan2(
                        estTargetY - turretFieldY, estTargetX - turretFieldX);
                turretAprilTagOffset = normalizeSignedDeg(
                        Math.toDegrees(turretAbsToTarget - heading) - rawTargetAngle);
                return;
            }
        }
    }

    @Override
    protected void write() {
        if (getState(TurretState.class) == TurretState.OFF) {
            return;
        }

        double frontPos = Math.max(0.0, Math.min(1.0, targetServoPosition - TENSION_OFFSET + frontServoOffset));
        double backPos = Math.max(0.0, Math.min(1.0, targetServoPosition + TENSION_OFFSET + backServoOffset));

        if (!requireEndgame().disableServosForEndgame) {
            turretServoFront.setPosition(frontPos);
            turretServoBack.setPosition(backPos);
        } else {
            turretServoFront.setPwmDisable();
            turretServoBack.setPwmDisable();
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    @Override
    protected void onTelemetry() {
        if (turretTelemetry.TOGGLE) {
            if (turretTelemetry.position) {
                logDashboard("Turret State", getState(TurretState.class));
                logDashboard("Raw Target Angle (deg)", "%.1f", rawTargetAngle);
                logDashboard("Delta Raw Target (deg)", "%.1f", deltaRawTarget);
                log("Target Angle (deg)", "%.1f", targetAngle);
                logDashboard("Ahead Target Angle (deg)", "%.1f", aheadTargetAngle);
                log("Turret Offset (deg)", "%.1f", turretManualOffset);
                log("Turret AprilTag Offset (deg)", "%.1f", turretAprilTagOffset);
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
        }
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

            if (targetAngle < 90) {
                targetAngle += leftSideOffset;
            }

            double signedAngle = targetAngle > 180 ? targetAngle - 360 : targetAngle;
            withinRange = Math.abs(signedAngle) <= maxTurretAngle;

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
            turretAprilTagOffset = 0;
        } else if (targetServoPosition < minServo) {
            targetServoPosition = minServo;
            turretAprilTagOffset = 0;
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
        if (result == null || !result.isValid()) {
            relocalizationStatus = "NO_VALID_LL_RESULT";
            return null;
        }

        if (result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) {
            relocalizationStatus = "STALE_RESULT";
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

    public boolean isWithinRange() {
        return withinRange;
    }

    public void lock(Pose target) {
        double heading = target.heading();

        double lockTurretFieldX = target.x() + turretX * Math.cos(heading) - turretY * Math.sin(heading);
        double lockTurretFieldY = target.y() + turretX * Math.sin(heading) + turretY * Math.cos(heading);

        double absoluteAngle = Math.toDegrees(
                Math.atan2(DecodeContext.targetY - lockTurretFieldY, DecodeContext.targetX - lockTurretFieldX));
        double angle = normalizeAngle(absoluteAngle - Math.toDegrees(heading)) + autoTurretOffset;
        if (angle < 90) {
            angle += leftSideOffset;
        }

        double servoPosition = clampServoPosition(angleToServoPosition(angle));
        TurretState.MANUAL.setValue(servoPosition);
        TurretState.MANUAL.activate();
    }

    public double robotAngleToPoseDeg(Pose target) {
        Pose robotPose = requireFollower().pose();
        return Math.toDegrees(Math.atan2(
                target.y() - robotPose.y(),
                target.x() - robotPose.x()));
    }

    public void unlock() {
        TurretState.AUTOAIM.activate();
    }

    public boolean detectObelisk() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            switch (fr.getFiducialId()) {
                case 21:
                    DecodeContext.motif = new MagazineState(GREEN, PURPLE, PURPLE);
                    detectingObelisk = false;
                    return true;
                case 22:
                    DecodeContext.motif = new MagazineState(PURPLE, GREEN, PURPLE);
                    detectingObelisk = false;
                    return true;
                case 23:
                    DecodeContext.motif = new MagazineState(PURPLE, PURPLE, GREEN);
                    detectingObelisk = false;
                    return true;
            }
        }
        return false;
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
        if (result == null || !result.isValid()) return;
        if (result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) return;

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

    private double clampServoPosition(double position) {
        double lo = Math.min(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        double hi = Math.max(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        return Math.max(lo, Math.min(hi, position));
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

    private double normalizeSignedDeg(double deg) {
        double wrapped = deg % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
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
}
