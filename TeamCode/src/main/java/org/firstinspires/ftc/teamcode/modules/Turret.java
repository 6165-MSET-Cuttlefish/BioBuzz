package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

import java.util.function.Supplier;

public class Turret extends Module {

    @Config("Turret")
    public static class Tuning {
        public static double offset = 0.01;

        public static double nectarX = 0;
        public static double nectarY = 0;
        public static double pollenX = 0;
        public static double pollenY = 0;

        public static double leftPosition = 0;
        public static double backPosition = 0.5;
        public static double rightPosition = 1;
    }

    public enum TurretState implements State {
        BACK(Tuning.backPosition),
        LEFT(Tuning.leftPosition),
        RIGHT(Tuning.rightPosition),
        AUTOAIM(-1),
        HOLD(-1);

        TurretState(double v) {
            setValue(v);
        }
    }

    private final EnhancedServo nectarL;
    private final EnhancedServo nectarR;
    private final EnhancedServo pollenL;
    private final EnhancedServo pollenR;

    private final Follower follower;
    private Supplier<Pose> goal;

    private double nectarPosition = 0.5;
    private double pollenPosition = 0.5;
    private double nectarAngleDeg;
    private double pollenAngleDeg;

    public Turret(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        goal = () -> BiobuzzContext.getHive(follower.pose());

        nectarL = new EnhancedServo(hardwareMap, "NectarTurretL").withCachingTolerance(0.001);
        nectarR = new EnhancedServo(hardwareMap, "NectarTurretR").withCachingTolerance(0.001);
        pollenL = new EnhancedServo(hardwareMap, "PollenTurretL").withCachingTolerance(0.001);
        pollenR = new EnhancedServo(hardwareMap, "PollenTurretR").withCachingTolerance(0.001);
    }

    public void setGoal(Supplier<Pose> goal) {
        this.goal = goal;
    }

    @Override
    protected void initStates() {
        setStates(TurretState.AUTOAIM);
        bindTunable(TurretState.LEFT, () -> Tuning.leftPosition);
        bindTunable(TurretState.BACK, () -> Tuning.backPosition);
        bindTunable(TurretState.RIGHT, () -> Tuning.rightPosition);
    }

    @Override
    protected void read() {
        TurretState state = getState(TurretState.class);
        if (state == TurretState.AUTOAIM) {
            Pose robot = follower.pose();
            Pose target = goal.get();
            nectarAngleDeg = angleFromBackDeg(robot, Tuning.nectarX, Tuning.nectarY, target);
            pollenAngleDeg = angleFromBackDeg(robot, Tuning.pollenX, Tuning.pollenY, target);
            nectarPosition = servoPosition(nectarAngleDeg);
            pollenPosition = servoPosition(pollenAngleDeg);
        } else if (state != TurretState.HOLD) {
            nectarPosition = state.getValue();
            pollenPosition = state.getValue();
        }
    }

    @Override
    protected void write() {
        nectarL.setPosition(nectarPosition);
        nectarR.setPosition(clamp(nectarPosition + Tuning.offset, 0, 1));
        pollenL.setPosition(pollenPosition);
        pollenR.setPosition(clamp(pollenPosition + Tuning.offset, 0, 1));
    }

    @Override
    public void stop() {
        nectarL.setPwmDisable();
        nectarR.setPwmDisable();
        pollenL.setPwmDisable();
        pollenR.setPwmDisable();
    }

    @Override
    protected void onTelemetry() {
        log("Nectar Angle From Back (deg)", "%.1f", nectarAngleDeg);
        log("Pollen Angle From Back (deg)", "%.1f", pollenAngleDeg);
        logDashboard("Nectar Position", "%.3f", nectarPosition);
        logDashboard("Pollen Position", "%.3f", pollenPosition);
    }

    public static double angleFromBackDeg(Pose robot, double turretX, double turretY, Pose goal) {
        double h = robot.heading();
        double px = robot.x() + turretX * Math.cos(h) - turretY * Math.sin(h);
        double py = robot.y() + turretX * Math.sin(h) + turretY * Math.cos(h);
        double fieldAngle = Math.toDegrees(Math.atan2(goal.y() - py, goal.x() - px));
        double fromBack = wrapDeg(fieldAngle - Math.toDegrees(h) - 180);
        return clamp(fromBack, -90, 90);
    }

    private static double servoPosition(double angleFromBackDeg) {
        double back = TurretState.BACK.getValue();
        double end = angleFromBackDeg < 0 ? TurretState.LEFT.getValue() : TurretState.RIGHT.getValue();
        return back + Math.abs(angleFromBackDeg) / 90 * (end - back);
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
}
