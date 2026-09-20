package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Camera-relative to field-relative, for both position and velocity.
 *
 * <p>The homography gives inches in a frame rigidly attached to the camera, so a ball sitting still
 * on the carpet appears to move whenever the robot does. Two chained rigid transforms fix the
 * position — camera frame to robot frame via the mount extrinsics, robot frame to field frame via
 * the robot's pose — and the velocity additionally picks up the robot's own motion:
 *
 * <pre>v_field = v_robot + omega x r + R(heading + mountHeading) * v_camera</pre>
 *
 * <p>The {@code omega x r} term is the one that is easy to forget and impossible to ignore: a ball
 * two feet off to the side of a robot spinning at 3 rad/s reads 6 in/s of pure rotation-induced
 * velocity that has nothing to do with the ball.
 */
public final class BallFieldTransform {

    /**
     * Where the camera's coordinate frame sits on the robot. The pipeline draws a crosshair at its
     * origin — put a ball on that crosshair, measure to robot center, and these are the numbers.
     */
    @Config("CameraMount")
    public static class Mount {
        /** Robot-frame position of the camera frame's origin, inches. +X forward, +Y left. */
        public static double xIn = 0;
        public static double yIn = 0;
        /** Rotation from robot axes to camera-frame axes, degrees CCW. */
        public static double headingDeg = 0;
        /**
         * Mirrors the camera frame's Y axis. A chessboard-derived frame is as likely to come out
         * left-handed as right-handed, and no rotation can fix a handedness mismatch — if balls
         * land on the wrong side of the robot after {@link #headingDeg} is correct, flip this.
         */
        public static boolean mirrorY = false;
        /**
         * False leaves ball velocity in the robot's frame (rotated into field axes but with the
         * robot's own motion left in). Useful for checking the mount numbers in isolation.
         */
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
            vx += robot.vx - robot.omega * offset[1];
            vy += robot.vy + robot.omega * offset[0];
        }

        return new FieldBall(ball.id, ball.type, robot.x + offset[0], robot.y + offset[1], vx, vy, ball, ball.visible);
    }

    /** Field-frame position of an arbitrary camera-frame point, in inches. */
    public static Pose cameraPointToField(double cameraX, double cameraY,
                                          RobotStateHistory.Sample robot) {
        double[] offset = cameraVectorToFieldAxes(cameraX, cameraY, robot.heading, true);
        return new Pose(robot.x + offset[0], robot.y + offset[1]);
    }

    /**
     * Rotates a camera-frame vector into field axes. {@code applyMountOffset} adds the camera's
     * position on the robot, which is right for points and wrong for velocities — a rigid
     * translation shifts where a thing is, not how fast it is going.
     */
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
