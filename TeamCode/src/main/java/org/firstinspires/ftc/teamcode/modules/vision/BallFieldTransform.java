package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class BallFieldTransform {

    @Config("CameraMount")
    public static class Mount {
        /** Camera-frame origin (the pipeline's crosshair) from robot center; +X forward, +Y left. */
        public static double xIn = 29;
        public static double yIn = 1;
        /** Robot axes to camera-frame axes, CCW. */
        public static double headingDeg = 0;
        /** A chessboard-derived camera frame can come out left-handed, which no rotation fixes. */
        public static boolean mirrorY = false;
        public static boolean compensateRobotMotion = true;
    }

    private BallFieldTransform() {}

    public static List<FieldBall> toField(List<TrackedBall> balls, RobotStateHistory.Sample robot) {
        if (balls.isEmpty() || robot == null) return Collections.emptyList();

        List<FieldBall> fieldBalls = new ArrayList<>(balls.size());
        for (TrackedBall ball : balls) fieldBalls.add(toField(ball, robot));
        return Collections.unmodifiableList(fieldBalls);
    }

    public static FieldBall toField(TrackedBall ball, RobotStateHistory.Sample robot) {
        double[] offset = cameraVectorToFieldAxes(ball.x, ball.y, robot.heading, true);
        double[] velocity = cameraVectorToFieldAxes(ball.vx, ball.vy, robot.heading, false);

        double vx = velocity[0];
        double vy = velocity[1];
        if (Mount.compensateRobotMotion) {
            // v_robot + omega x r: a spinning robot induces velocity on every off-center ball.
            vx += robot.vx - robot.omega * offset[1];
            vy += robot.vy + robot.omega * offset[0];
        }

        return new FieldBall(ball.id, ball.type, robot.x + offset[0], robot.y + offset[1], vx, vy, ball.visible);
    }

    // Mount offset is for points only: a rigid translation moves a position, not a velocity.
    private static double[] cameraVectorToFieldAxes(double cameraX, double cameraY,
                                                    double robotHeading, boolean applyMountOffset) {
        double y = Mount.mirrorY ? -cameraY : cameraY;

        double mountRad = Math.toRadians(Mount.headingDeg);
        double mountCos = Math.cos(mountRad);
        double mountSin = Math.sin(mountRad);
        double robotX = cameraX * mountCos - y * mountSin;
        double robotY = cameraX * mountSin + y * mountCos;
        if (applyMountOffset) {
            robotX += Mount.xIn;
            robotY += Mount.yIn;
        }

        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);
        return new double[] { robotX * cos - robotY * sin, robotX * sin + robotY * cos };
    }
}
