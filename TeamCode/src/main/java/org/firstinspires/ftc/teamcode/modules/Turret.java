package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;

import java.util.function.Supplier;

public class Turret extends Module {

    @Config("Turret")
    public static class Tuning {
        public static TurretTuning nectar = new TurretTuning();
        public static TurretTuning pollen = new TurretTuning();
        public static double flipBandDeg = 10;
        public static double cellSwitchBandIn = 3;
    }

    public static class TurretTuning {
        public double x = 0;
        public double y = 0;
        public double leftPosition = 0;
        public double backPosition = 0.5;
        public double rightPosition = 1;
        public double offset = 0.01;
    }

    public enum TurretState implements State {
        BACK(0),
        LEFT(-90),
        RIGHT(90),
        AUTOAIM(0),
        HOLD(0);

        TurretState(double angleFromBackDeg) {
            setValue(angleFromBackDeg);
        }
    }

    private final TurretServos nectar;
    private final TurretServos pollen;

    private final Follower follower;
    private Supplier<Pose> goal;
    private Boolean audienceCell;

    public Turret(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        goal = this::ownCell;

        nectar = new TurretServos(hardwareMap, "NectarTurretL", "NectarTurretR", Tuning.nectar);
        pollen = new TurretServos(hardwareMap, "PollenTurretL", "PollenTurretR", Tuning.pollen);
    }

    public void setGoal(Supplier<Pose> goal) {
        this.goal = goal;
    }

    private Pose ownCell() {
        double fromMidline = follower.pose().y() - FieldConfig.fieldWidthInches / 2;
        // Hold the chosen CELL near the midline, or pose noise there swings the turrets between the two.
        if (audienceCell == null || Math.abs(fromMidline) > Tuning.cellSwitchBandIn) audienceCell = fromMidline < 0;
        return BioBuzzField.ownCell(audienceCell);
    }

    @Override
    protected void initStates() {
        setStates(TurretState.AUTOAIM);
    }

    @Override
    protected void read() {
        TurretState state = getState(TurretState.class);
        if (state == TurretState.AUTOAIM) {
            Pose robot = follower.pose();
            Pose target = goal.get();
            nectar.aim(robot, target);
            pollen.aim(robot, target);
        } else if (state != TurretState.HOLD) {
            nectar.point(state.getValue());
            pollen.point(state.getValue());
        }
    }

    @Override
    protected void write() {
        nectar.write();
        pollen.write();
    }

    @Override
    public void stop() {
        nectar.disable();
        pollen.disable();
    }

    @Override
    protected void onTelemetry() {
        log("Nectar Angle From Back (deg)", "%.1f (goal %.1f)", nectar.angleDeg, nectar.goalAngleDeg);
        log("Pollen Angle From Back (deg)", "%.1f (goal %.1f)", pollen.angleDeg, pollen.goalAngleDeg);
        logDashboard("Nectar Position", "%.3f", nectar.position);
        logDashboard("Pollen Position", "%.3f", pollen.position);
    }

    public static double angleFromBackDeg(Pose robot, double turretX, double turretY, Pose goal) {
        double h = robot.heading();
        double px = robot.x() + turretX * Math.cos(h) - turretY * Math.sin(h);
        double py = robot.y() + turretX * Math.sin(h) + turretY * Math.cos(h);
        double fieldAngle = Math.toDegrees(Math.atan2(goal.y() - py, goal.x() - px));
        return wrapDeg(fieldAngle - Math.toDegrees(h) - 180);
    }

    static double reachableAngleDeg(double goalAngleDeg, double currentAngleDeg, double flipBandDeg) {
        if (Math.abs(goalAngleDeg) <= 90) return goalAngleDeg;
        // Near dead ahead, keep the side the turret is on: heading noise would otherwise swing it end to end.
        boolean nearDeadAhead = 180 - Math.abs(goalAngleDeg) < flipBandDeg;
        if (nearDeadAhead && currentAngleDeg * goalAngleDeg < 0) return Math.copySign(90, currentAngleDeg);
        return Math.copySign(90, goalAngleDeg);
    }

    private static double wrapDeg(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static final class TurretServos {
        private final EnhancedServo left;
        private final EnhancedServo right;
        private final TurretTuning tuning;
        private double goalAngleDeg;
        private double angleDeg;
        private double position;

        TurretServos(HardwareMap hardwareMap, String leftName, String rightName, TurretTuning tuning) {
            left = new EnhancedServo(hardwareMap, leftName).withCachingTolerance(0.001);
            right = new EnhancedServo(hardwareMap, rightName).withCachingTolerance(0.001);
            this.tuning = tuning;
            position = tuning.backPosition;
        }

        void aim(Pose robot, Pose goal) {
            goalAngleDeg = angleFromBackDeg(robot, tuning.x, tuning.y, goal);
            turnTo(reachableAngleDeg(goalAngleDeg, angleDeg, Tuning.flipBandDeg));
        }

        void point(double angleFromBackDeg) {
            goalAngleDeg = angleFromBackDeg;
            turnTo(angleFromBackDeg);
        }

        private void turnTo(double angleFromBackDeg) {
            angleDeg = angleFromBackDeg;
            double end = angleFromBackDeg < 0 ? tuning.leftPosition : tuning.rightPosition;
            double raw = tuning.backPosition + Math.abs(angleFromBackDeg) / 90 * (end - tuning.backPosition);
            position = clamp(raw, Math.max(0, -tuning.offset), Math.min(1, 1 - tuning.offset));
        }

        void write() {
            left.setPosition(position);
            right.setPosition(position + tuning.offset);
        }

        void disable() {
            left.setPwmDisable();
            right.setPwmDisable();
        }
    }
}
